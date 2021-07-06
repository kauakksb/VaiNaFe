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

    # Função de movimento do motor usando força como parâmetro
    def dc(self, force):
        self.motor.dc(force)

    # Move um motor grande do robõ por uma distância definida
    def run_dist(self, speed, dist, stop_type = 'hold'):
        self.motor.reset_angle(0) # Resetando o motor
        pi = 3.14

        # Define a direção da rotação do motor
        if dist > 0: # Se a distância for positiva

            while dist > (self.motor.angle() *(self.pi * self.wheel_diameter) / 360) * 10: # Converte os graus rotacionados pelo motor em milímetros
                self.motor.run(speed)

        elif dist < 0: # Se a distância for negativa
            while dist < (self.motor.angle() *(self.pi * self.wheel_diameter) / 360) * 10: # Converte os graus rotacionados pelo motor em milímetros
                self.motor.run(-speed)

        # Tipos de parada
        if stop_type == 'hold': # Trava o motor após a execução do movimento
            self.motor.stop('hold')
        elif stop_type == 'stop':
            self.motor.stop('stop') # deixa os motores livres após a execução do movimento

    # Correndo com detecção de parada inifito até detectar
    def run_with_detection_stop_infinity(self, speed, torque):
        self.motor.run_until_stalled(speed=speed, duty_limit=torque)

    # Correndo com detecção de parada inifito até detectar ou cumprir o limite de ângulo
    def run_with_detection_stop_and_drop(self, speed, torque, degree):
        pass
    
    # Move um motor médio do robô por um número determinado de graus
    def move_grab(self,speed,degree):
        self.reset_angle() # Resetando o motor

        # Se move para cada lado dependendo do sinal da velocidade, se é negativo ou positivo

        # Se a velocidade for positiva
        if speed > 0:
            while self.motor.angle() < degree: # Move o motor enquanto o valor de graus dele for menor que o determinado
                self.motor.run(speed)
            self.stop('stop') # Interrompe o movimento do motor

        # Se a velocidade for negativa
        else:
            while self.motor.angle() > degree*-1: # Move o motor enquanto o valor de graus dele for maior que o determinado
                self.motor.run(speed)
            self.stop('stop') # Interrompe o movimento do motor

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
