#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class ClassMotor:
    def __init__(self, port):
        self.motor = Motor(port)

        self.pi = 3.14
        self.wheel_diameter = 56
    
    # Função de corrida (infinita) 
    def run(self, speed):
        self.motor.run(speed)

    def run_dist(self, speed, dist, stop_type = 'hold'):
        self.motor.reset_angle(0)
        pi = 3.14

        if stop_type == 'hold':

            if dist > 0:

                while dist > (self.motor.angle() *(self.pi * self.wheel_diameter) / 360) * 10:
                    self.motor.run(speed)
                self.stop('hold')

            if dist < 0:
                while dist < (self.motor.angle() *(self.pi * self.wheel_diameter) / 360) * 10:
                    self.motor.run(speed)
                self.stop('hold')

        if stop_type == 'stop':

            if dist > 0:

                while dist > (self.motor.angle() *(self.pi * self.wheel_diameter) / 360) * 10:
                    self.motor.run(speed)
                self.stop('stop')

            if dist < 0:
                while dist < (self.motor.angle() *(self.pi * self.wheel_diameter) / 360) * 10:
                    self.motor.run(speed)
                self.stop('stop')


    # Correndo com detecção de parada inifito até detectar
    def run_with_detection_stop_infinity(self, speed, torque):
        self.motor.run_until_stalled(speed=speed, duty_limit=torque)

    # Correndo com detecção de parada inifito até detectar ou cumprir o limite de ângulo
    def run_with_detection_stop_and_drop(self, speed, torque, degree):
        pass

    def move_grab(self,speed,degree):
        self.reset_angle()
        if speed > 0:
            while self.motor.angle() < degree:
                self.motor.run(speed)
            self.stop('stop')
        else:
            while self.motor.angle() > degree*-1:
                self.motor.run(speed)
            self.stop('stop')

    # Retorna a velocidade atual do motor
    def get_speed(self):
        return self.motor.speed()

    # Retorna o ângulo do robô
    def get_angle(self):
        return self.motor.angle()

    # Resetar o ângulo do motor para algum valor
    def reset_angle(self, value = 0):
        self.motor.reset_angle(value)
    
    # Parando o motor, de acordo com o tipo selecionado
    def stop(self, type):
        if type == 'stop':
            self.motor.stop()
        elif type == 'hold':
            self.motor.hold()
        else:
            self.motor.brake()
