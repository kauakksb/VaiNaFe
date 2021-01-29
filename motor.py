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
    
    # Função de corrida (infinita) 
    def run(self, speed):
        self.motor.run(speed)

    # Correndo com detecção de parada inifito até detectar
    def run_with_detection_stop_infinity(self, speed, torque):
        self.motor.run_until_stalled(speed=speed, duty_limit=torque)

    # Correndo com detecção de parada inifito até detectar ou cumprir o limite de ângulo
    def run_with_detection_stop_and_drop(self, speed, torque, degree):
        pass

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