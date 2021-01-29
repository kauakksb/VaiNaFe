from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class ClassColorSensor:
    def __init__(self, port):
        self.sensor = ColorSensor(port) # Instanciando a classe de sensor
    
    # Obtendo o valor do sensor de cor com o tipo determinado
    def get_value(self, type):
        if type == 'color':
            return self.sensor.color()
        elif type == 'ambient':
            return self.sensor.ambient()
        elif type == 'reflection':
            return self.sensor.reflection()
        else:
            return self.sensor.rgb()


