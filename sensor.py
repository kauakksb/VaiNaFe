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

class ClassGyroSensor:
    def __init__(self,port,positive_direction = Direction.CLOCKWISE):
        self.gsensor = GyroSensor(port,positive_direction) # Instanciando a classe de sensor giroscópico


    # Obtendo valores de velocidade de rotação
    def get_gyro_speed(self):
        return self.gsensor.speed()

    # Obtendo valores de ângulo de rotação 
    def get_gyro_angle(self):
        return self.gsensor.angle()

    # Reseta os valores do ângulo de rotação
    def reset_angle(self,value = 0):
        self.gsensor.reset_angle(value)
