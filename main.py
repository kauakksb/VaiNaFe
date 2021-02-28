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
from sensor import ClassColorSensor, ClassGyroSensor

tiny_font = Font(size = 16, bold = True)

class Robot:
    def __init__(self):
        self.name = "Legosvaldo" # Propriedade de nome

        self.brick = EV3Brick() # Bloco EV3
        self.brick.screen.clear()    
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
            self.gyro_sensor = ClassGyroSensor(Port.S1) # Sensor giroscópio

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
        self.distance_between_wheels = 121 # Distância entre as rodas em mm
        self.white = 90
        self.black = 7
        self.white_back_sensors = 60

        self.threshold = (self.black + self.white) / 2

        # Drive Base
        self.drive = ClassDriveBase(
            self.left_t_motor, self.right_t_motor, self.wheel_diameter, self.distance_between_wheels)


    # Método de inicialização do robô
    def start(self) -> None:
        # Ativando o método de checkagem

        # Máquina de Estado
        while True:
            # Botões pressionados
            buttons_pressed = self.brick.buttons.pressed()

            self.brick.screen.set_font(tiny_font)#Define a fonte dos textos
            #Escreve a função de cada botão 
            self.brick.screen.draw_text(
                0,10,'Esquerda: saida ',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,25,'Cima: Saída 1',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,40,'Baixo: Saída 2',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,55,'Direita: Saída 3',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,70,'Meio: Fim',text_color = Color.BLACK ,background_color = None)

            # Switch de casos de botões pressionados
            if Button.UP in buttons_pressed:
                self.launch_one() # Lançamento um
            elif Button.DOWN in buttons_pressed:
                self.launch_two() # Lançamento dois
            elif Button.RIGHT in buttons_pressed:
                self.launch_three()# Lançamento três
            elif Button.LEFT in buttons_pressed:
                self.calibration() # Calibração
            elif Button.CENTER in buttons_pressed:
                break
            
            wait(100) # Tempo de 200ms para a não ativação desproposital de alguma função

    # Função de lançamento um
    def launch_one(self):
        self.drive.run_straight(350, 300)
        wait(100)
        self.drive.turn_angle(16, 150)
        wait(100)
        self.drive.run_straight(140, 100)
        wait(100)
        self.drive.run_straight(-480, 900)

    # Função de lançamento dois
    def launch_two(self):
        self.gyro_sensor.reset_angle(0)
        self.left_g_motor.run_with_detection_stop_infinity(1000,50)
        self.drive.pid_run_straight(800,300)
        wait(500)
        self.drive.run_until_line(100,self.left_s_color,self.black)
        wait(250)
        self.drive.line_follow(50,50)
        self.drive.line_follow(150,140)
        self.drive.line_follow(130,100)
        self.drive.line_follow(40,25)
        self.drive.run_until_line(275,self.left_s_color,self.white_back_sensors)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.right_g_motor.move_grab(1500,3800)
        self.drive.run_during_line(-150,self.left_s_color,self.black)
        self.drive.run_during_line(-150,self.left_s_color,self.white_back_sensors)
        self.drive.run_until_line(-125,self.left_s_color,self.white_back_sensors)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.gyro_turn_degree(100,30)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_straight(250,150)
        self.drive.run_until_line(125,self.right_s_color,self.white_back_sensors)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_until_line(50,self.right_s_color,self.black)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_until_line(50,self.right_s_color,self.white_back_sensors)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.left_g_motor.run_with_detection_stop_infinity(-1500,75)
        self.drive.run_until_line(-50,self.right_s_color,self.black)
        self.drive.run_until_line(-50,self.right_s_color,self.white_back_sensors)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.gyro_turn_degree(100,45)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_straight(30,15)
        self.left_g_motor.run_with_detection_stop_infinity(1500,65)
        self.drive.run_straight(-30,15)
        self.drive.gyro_turn_degree(75,-45)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_until_line(-125,self.left_s_color,self.white_back_sensors)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.gyro_turn_degree(100,-32.5)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_straight(-1750,1000)


    def launch_three(self):
        self.left_g_motor.run_with_detection_stop_infinity(1000,35)
        wait(50)
        self.drive.run_straight(450,300)
        wait(50)
        self.left_g_motor.move_grab(-800,900)
        wait(50)
        self.drive.run_straight(100,300)
        wait(50)
        self.drive.run_straight(-95,300)
        wait(50)
        self.drive.turn_angle(-55,100)
        wait(50)
        self.drive.run_straight(20,100)
        self.drive.line_follow(275,135)
        self.drive.run_until_line(75,self.left_s_color,self.black)
        wait(50)
        self.drive.pid_run_straight(120,100)
        self.left_g_motor.move_grab(-1000,1000)
        

    def launch_four(self):
        self.left_g_motor.move_grab(-1000,2500)
        self.left_g_motor.move_grab(1000,100)
        
 
    def calibration(self) -> None:
        self.gyro_sensor.reset_angle()

        conditional = True

        buttons_pressed = self.brick.buttons.pressed()

        while conditional == True:

            if Button.LEFT in buttons_pressed:
                self.brick.screen.clear()
                self.brick.screen.draw_text(0,50,'Coloque no Preto', text_color = Color.BLACK, background_color = None)
                wait(5000)

                if Button.LEFT in buttons_pressed:
                    self.brick.screen.clear()
                    self.black = self.front_s_color.get_value('reflection')
                    self.brick.screen.draw_text(0,50,'Preto: {}'.format(self.black))
                    wait(5000)

            elif Button.RIGHT in buttons_pressed:
                self.brick.screen.clear()
                self.brick.screen.draw_text(0,50,'Coloque no Branco', text_color = Color.BLACK, background_color = None)
                wait(5000)

                if Button.RIGHT in buttons_pressed:
                    self.brick.screen.clear()
                    self.white = self.front_s_color.get_value('reflection')
                    self.brick.screen.draw_text(0,50,'Branco: {}'.format(self.white))
                    wait(5000)

            elif Button.CENTER in buttons_pressed:
                conditional = False


    # Retornando o valor de voltagem e corrente da bateria
    def get_battery(self) -> list:
        return [self.brick.battery.voltage(), self.brick.battery.current()]

    

robot = Robot() # Instanciando objeto de robô
robot.start() # Startando a programação
