#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from sensor import ClassColorSensor

class ClassDriveBase:
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        # Os dois motores (Da classe criada: ClassMotor)
        self.left_motor = left_motor
        self.right_motor = right_motor

        # Instanciando a classe de DriveBase
        self.drive = DriveBase(left_motor.motor, right_motor.motor, wheel_diameter, axle_track)

        # Valores de configuração
        self.straight_rate = 600
        self.straight_acceleration = 250
        self.turn_rate = 450
        self.turn_acceleration = 250

        self.wheel_diameter = 56
        self.axle_track = 145

        self.black = 0
        self.white = 0

        self.error_correction = 0

        self.front_s_color = ClassColorSensor(Port.S4)

        # Definindo os valores de configuração
        self.set_state(
            self.straight_rate, self.straight_acceleration, self.turn_rate, self.turn_acceleration,self.error_correction) 

    # Curva no eixo do robô até um determinado ângulo
    def turn_angle(self, angle, speed):
        self.set_rate(speed)
        self.drive.turn(angle)
        self.drive.stop()

    # Corrida retilínea em uma determinada distância em mm
    def run_straight(self, distance, speed):
        self.set_speed(speed)
        self.drive.straight(distance)
        self.drive.stop()

    # Definindo as configurações
    def set_state(self, straight_rate, straight_acceleration, turn_rate, turn_acceleration,error_correction):
        # Caso algum valor mude, mude a propriedade da classe também
        self.straight_rate = straight_rate
        self.straight_acceleration = straight_acceleration
        self.turn_rate = turn_rate
        self.turn_acceleration = turn_acceleration
        self.error_correction = error_correction


        self.drive.settings(self.straight_rate, self.straight_acceleration, self.turn_rate, self.turn_acceleration)

    # Definindo a propriedade de velocidade na reta
    def set_speed(self, straight_rate):
        self.straight_rate = straight_rate
        self.drive.settings(self.straight_rate, self.straight_acceleration, self.turn_rate, self.turn_acceleration)

    # Definindo a propriedade de aceleração na reta
    def set_acceleraton(self, straight_acceleraton):
        self.straight_acceleraton = straight_acceleraton
        self.drive.settings(self.straight_rate, self.straight_acceleration, self.turn_rate, self.turn_acceleration)

    # Definindo a propriedade de velocidade de curva
    def set_rate(self, turn_rate):
        self.turn_rate = turn_rate
        self.drive.settings(self.straight_rate, self.straight_acceleration, self.turn_rate, self.turn_acceleration)

    # Definindo a propriedade de aceleração de curva
    def set_turn_acceleration(self, turn_acceleration):
        self.turn_acceleration = turn_acceleration
        self.drive.settings(self.straight_rate, self.straight_acceleration, self.turn_rate, self.turn_acceleration)
            
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

    # Função de seguir linha
    def line_follow(self,distance,speed,proportional = 1.4):
        self.reset()
        self.drive.distance()
        self.set_speed(speed)

        # Define os valores de preto e de branco caso eles não tenham sido definidos na calibração
        if self.black == 0:
            self.black = 6
            self.white = 91
        
        threshold = (self.black + self.white) / 2 # Calcula a méida dos valores de preto e branco
        pi = 3.14 # Define o valor de pi

        # Executará o código enquanto o robô não tiver percorrido a distância definida
        while distance > self.drive.distance():
            
            # Mede o desvio,calcula a correção a ser feita e então a executa
            reflection_line_follower = self.front_s_color.get_value('reflection') # Valor de reflexão do sensor
            deviation = reflection_line_follower - threshold # Calculando o desvio feito pelo robô
            self.error_correction = deviation * proportional # Calculando a correção a ser feita pelo robô
            self.drive.drive(speed,self.error_correction) # Executando a correção
            print(deviation)
            print('cor:',self.front_s_color.get_value('reflection'))
        self.drive.stop() 

    # Move-se de forma retilínea até identificar a cor preta
    def run_until_line(self,speed,sensor,line):
        self.set_speed(speed)

        if line < 30:
            while sensor.get_value('reflection') > line:
                self.drive.drive(speed,0)# O robô se locomove pra frente
            self.drive.stop()

        elif line > 55:
            while sensor.get_value('reflection') < line:
                self.drive.drive(speed,0)# O robô se locomove pra frente
            self.drive.stop()
    