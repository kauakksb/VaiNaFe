#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class ClassDriveBase:
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        # Os dois motores (Da classe criada: ClassMotor)
        self.left_motor = left_motor
        self.right_motor = right_motor

        # Instanciando a classe de DriveBase
        self.drive = DriveBase(left_motor.motor, right_motor.motor, wheel_diameter, axle_track)

        # Valores de configuração
        self.straight_speed = 600
        self.straight_acceleration = 200
        self.turn_rate = 600
        self.turn_acceleration = 200

        # Definindo os valores de configuração
        self.set_state(
            self.straight_speed, self.straight_acceleration, self.turn_rate, self.turn_acceleration) 

    # Curva no eixo do robô até um determinado ângulo
    def turn_angle(self, angle):
        self.drive.turn(angle)

    # Corrida retilínea em uma determinada distância em mm
    def run_straight(self, distance):
        self.drive.straight(distance)

    # Definindo as configurações
    def set_state(self, straight_speed, straight_acceleration, turn_rate, turn_acceleration):
        # Caso algum valor mude, mude a propriedade da classe também
        self.straight_speed = straight_speed
        self.straight_acceleration = straight_acceleration
        self.turn_rate = turn_rate
        self.turn_acceleration = turn_acceleration

        self.drive.settings(self.straight_speed, self.straight_acceleration, self.turn_rate, self.turn_acceleration)

    # Definindo a propriedade de velocidade na reta
    def set_speed(self, straight_speed):
        self.straight_speed = straight_speed
        self.drive.settings(self.straight_speed, self.straight_acceleration, self.turn_rate, self.turn_acceleration)

    # Definindo a propriedade de aceleração na reta
    def set_acceleraton(self, straight_acceleraton):
        self.straight_acceleraton = straight_acceleraton
        self.drive.settings(self.straight_speed, self.straight_acceleration, self.turn_rate, self.turn_acceleration)

    # Definindo a propriedade de velocidade de curva
    def set_rate(self, turn_rate):
        self.turn_rate = turn_rate
        self.drive.settings(self.straight_rate, self.straight_acceleration, self.turn_rate, self.turn_acceleration)

    # Definindo a propriedade de aceleração de curva
    def set_turn_acceleration(self, turn_acceleration):
        self.turn_acceleration = turn_acceleration
        self.drive.settings(self.straight_speed, self.straight_acceleration, self.turn_rate, self.turn_acceleration)
    
    # Obtendo o estado atual
    def get_state(self):
        return self.drive.state()

    # Obtendo a distância
    def get_distance(self):
        return self.drive.distance()

    # Obtendo o ângulo do robô
    def get_angle(self):
        return self.drive.angle()

    # Resetando os valores de distância e ângulo
    def reset(self):
        self.drive.reset()