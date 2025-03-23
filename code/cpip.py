import time
import ctypes
import threading
import pysoem


MODES_OF_OPERATION = 0x6060
CTRL_WRD = 0x6040
TORQUE_SLOPE = 0x6087
TARGET_TORQUE = 0x6071
MOTOR_ENCODER = 0x6064
EXTERNAL_ENCODER = 0x2511
SUBINDEX = 0x00


def enc_uint8(value):
    return bytes(ctypes.c_uint8(value))

def enc_uint16(value):
    return bytes(ctypes.c_uint16(value))

def enc_uint32(value):
    return bytes(ctypes.c_uint32(value))


def list_adapters():
    for i, adapter in pysoem.find_adapters():
        print(f'{i}: {adapter}')


class MotorDriver:

    def __init__(self, e_adapter_name='default', controller=None, limit_theta_abs=12, max_torque=300):
        if e_adapter_name == 'default':
            self.e_adapter_name = self.decide_the_adapter().name
        else:
            self.e_adapter_name = e_adapter_name
        self.controller = controller
        self.limit_theta_abs = limit_theta_abs
        self.limit_delta_abs = 1
        self.max_torque = max_torque
        self._online = True
        self._balancing = False
        self.init_motor_driver()
        self.t_watchdog = threading.Thread(target=self.watchdog)
        self.t_watchdog.start()
    
    @property
    def online(self):
        """ Acts as a turn on/off switch. """
        return self._online

    @online.setter
    def online(self, value):
        if type(value) is not bool:
            raise Exception('Value for online has to be of type bool.')
        self._online = value

    def set_controller(self, controller):
        self.controller = controller

    def choose_adapter(self, index):
        adapter = pysoem.find_adapters()[-1]
        self.init_motor_driver(adapter)

    def decide_the_adapter(self):
        e_adapters = [a for a in pysoem.find_adapters()]
        if not e_adapters:
            raise Exception('Нема етернет адаптер.')
        elif len(e_adapters) > 1:
            adapter = e_adapters[-1]
            print(f'Пронајдени се повеќе етернет адаптери, го користиме последниот во листата {adapter.name}.')
            return adapter
        else:
            adapter = e_adapters[0]
            print(f'Пронајден е етернет адаптер {adapter.name}.')
            return adapter

    def init_motor_driver(self):    
        self.master = pysoem.Master()
        self.master.open(self.e_adapter_name)
        if self.master.config_init() > 0:
            print('Пронајден е двигателот на моторот.')
            self.master.config_map()
            self.master.config_dc()
        else:
            raise Exception('Двигателот на моторот не е поврзан.')
        self.slave = self.master.slaves[0]
        # torque mode on
        self.slave.sdo_write(MODES_OF_OPERATION, SUBINDEX, enc_uint8(4))
        # servo on
        self.slave.sdo_write(CTRL_WRD, SUBINDEX, enc_uint8(6))
        # enable the motor
        self.slave.sdo_write(CTRL_WRD, SUBINDEX, enc_uint8(7))
        self.slave.sdo_write(CTRL_WRD, SUBINDEX, enc_uint8(15))
        # target slope
        self.slave.sdo_write(TORQUE_SLOPE, SUBINDEX, enc_uint16(250))
        self.slave.sdo_write(TARGET_TORQUE, SUBINDEX, enc_uint16(0))
        self.set_current_theta_as_zero()
        
    def set_current_theta_as_zero(self):
        self.slave.sdo_write(EXTERNAL_ENCODER, SUBINDEX, enc_uint32(0))

    def write_torque(self, value):
        self.slave.sdo_write(TARGET_TORQUE, SUBINDEX, enc_uint16(value))
            
    def read_theta_encoder(self):
        pulses = self.slave.sdo_read(EXTERNAL_ENCODER, SUBINDEX)
        pulses = int.from_bytes(pulses, byteorder='little')
        return pulses

    def read_theta(self):
        """ Read theta as angle in range [0, 360). """
        u_pulses = self.read_theta_encoder()
        s_pulses = u_pulses if u_pulses < 2 ** 31 else u_pulses - 2 ** 32
        angle = (s_pulses * 360 / 10000) % 360
        return angle

    def read_x_encoder(self):
        pulses = self.slave.sdo_read(MOTOR_ENCODER, SUBINDEX)
        pulses = int.from_bytes(pulses, byteorder='little')
        return pulses

    def read_x(self):
        """ Read x as distance in range [0, ?). """
        u_pulses = self.read_x_encoder()
        x = u_pulses * 40 / 1_280_000
        return x

    def watchdog(self):
        """ Waits for someone to hand-balance the pendulum. """
        self.last_theta = self.read_theta()
        while self._online:
            if self._balancing:
                self.control_loop()
                self.write_torque(0)
                self._balancing = False
                time.sleep(1)
            else:
                theta = self.read_theta()
                if not (45 < theta < 360-45):
                    self.last_theta = theta
                    time.sleep(0.5)
                    continue
                r2l_crossing = self.last_theta > 180 and theta <= 180
                l2r_crossing = self.last_theta < 180 and theta >= 180
                if r2l_crossing or l2r_crossing:
                    self._balancing = True
                else:
                    time.sleep(1)

    def control_loop(self):
        """ Lets the controller to control the pendulum. """
        if self.controller is None:
            raise Exception('Немаш поставено управувач за процесот.')
        new_turn = True
        while self._balancing:
            theta = self.read_theta()
            x = self.read_x()
            delta_abs = abs(theta - 180)
            if self._balancing and delta_abs > self.limit_theta_abs:
                return
            torque = self.controller(theta, x, new_turn)
            new_turn = False
            if torque > self.max_torque:
                print('Корисникот претерал со вртежниот момент.')
                return
            self.write_torque(int(torque))
            
            
                













    