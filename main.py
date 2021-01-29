#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font

from motor import ClassMotor
from drive import ClassDriveBase
from sensor import ClassColorSensor

class Robot:
    def __init__(self):
        self.name = "Legosvaldo" # Propriedade de nome

        self.brick = EV3Brick() # Bloco EV3
        """
            Limpar o visor! (a partir do self.brick)
        """

        # Tentando declarar as variáveis de sensor e robô
        # Motores
        try:
            self.right_t_motor = ClassMotor(Port.B) # Motor de tração direito
            self.left_t_motor = ClassMotor(Port.C) # Motor de tração esquerdo
            self.right_g_motor = ClassMotor(Port.A) # Motor de garra direito
            self.left_g_motor = ClassMotor(Port.D) # Motor de garra esquerdo

            # Sensores
            self.front_s_color = ClassColorSensor(Port.S4) # Sensor de cor frontal
            self.right_s_color = ClassColorSensor(Port.S2) # Sensor de cor direito
            self.left_s_color = ClassColorSensor(Port.S3) # Sensor de cor esquerdo
            self.gyro_sensor = GyroSensor(Port.S1) # Sensor giroscópio
            
        except:
            """
                Dever de Casa: Erro nas portas

                erros = []
                try:
                    trash = self.right_t_motor.get_speed()
                except:
                    erros.append('B')

                ...

                - No final da verificação, imprima os erros de alguma forma no visor
                    e faça algum barulho
            """

        # Valores úteis
        self.wheel_diameter = 56 # Diâmetro da roda em mm
        self.distance_between_wheels = 145 # Distância entre as rodas em mm
        self.white = 80
        self.black = 8

        # Drive Base
        self.drive = ClassDriveBase(
            self.left_t_motor, self.right_t_motor, self.wheel_diameter, self.distance_between_wheels) 

    # Método de inicialização do robô
    def start(self) -> None:
        self.calibration() # Ativando o método de checkagem

        # Máquina de Estado
        while True:
            # Botões pressionados
            buttons_pressed = self.brick.buttons.pressed()

            # Switch de casos de botões pressionados
            if Button.UP in buttons_pressed:
                self.launch_one() # Lançamento um
            elif Button.DOWN in buttons_pressed:
                self.launch_two() # Lançamento dois
            elif Button.CENTER in buttons_pressed:
                break
            
            wait(100) # Tempo de 200ms para a não ativação desproposital de alguma função

    # Função de lançamento um
    def launch_one(self):
        self.drive.run_straight(800)

    # Função de lançamento dois
    def launch_two(self):
        self.drive.turn_angle(95)

    # Checkagem dos cabos, valores de sensor e calibração
    def calibration(self) -> None:
        """
            Dever de casa: Calibração

            - Resetar o ângulo do sensor de giro para 0 (não esqueça de mudar
                a classe lá em cima)

            - Esperar apertar o botão do centro
            - Definir a propriedade 'self.black' igual ao valor do sensor da frente (reflection)
            - Espere 500ms

            - Esperar apertar o botão do centro
            - Definir a propriedade 'self.white' igual ao valor do sensor da frente (reflection)
            - Espere 500ms
        """
        pass
    
    # Retornando o valor de voltagem e corrente da bateria
    def get_battery(self) -> list:
        return [self.brick.battery.voltage(), self.brick.battery.current()]

    

robot = Robot() # Instanciando objeto de robô
robot.start() # Startando a programação