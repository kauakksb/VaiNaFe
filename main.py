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
import threading

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
        erros = []
        self.wheel_diameter = 56 # Diâmetro da roda em mm
        self.distance_between_wheels = 121 # Distância entre as rodas em mm
        self.white = 90
        self.black = 12
        self.white_back_sensors = 62

        self.threshold = (self.black + self.white) / 2

        try:
            self.right_t_motor = ClassMotor(Port.B) # Motor de tração direito
            self.left_t_motor = ClassMotor(Port.C) # Motor de tração esquerdo
            self.right_g_motor = ClassMotor(Port.A) # Motor de garra direito
            self.left_g_motor = ClassMotor(Port.D) # Motor de garra esquerdo
            self.front_s_color = ClassColorSensor(Port.S4) # Sensor de cor frontal
            self.right_s_color = ClassColorSensor(Port.S2) # Sensor de cor direito
            self.left_s_color = ClassColorSensor(Port.S3) # Sensor de cor esquerdo
            self.gyro_sensor = ClassGyroSensor(Port.S1) # Sensor giroscópio
            self.drive = ClassDriveBase(
                self.left_t_motor, self.right_t_motor, self.wheel_diameter, self.distance_between_wheels)
        
        except:
            '''nada''' 


    def check(self):

        erros = []
        
        try:
            self.right_t_motor = ClassMotor(Port.B) # Motor de tração direito
        
        except:
            erros.append('B')
            condition = True

        try:
            self.left_t_motor = ClassMotor(Port.C) # Motor de tração esquerdo
        
        except:
            erros.append('C')

        try:
            self.right_g_motor = ClassMotor(Port.A) # Motor de garra direito
        
        except:
            erros.append('A')
        
        try:
            self.left_g_motor = ClassMotor(Port.D) # Motor de garra esquerdo
        
        except:
            erros.append('D')
        
        try:
            self.front_s_color = ClassColorSensor(Port.S4) # Sensor de cor frontal
        
        except:
            erros.append('4')
    
        try:
            self.right_s_color = ClassColorSensor(Port.S2) # Sensor de cor direito
        
        except:
            erros.append('2')

        try:
            self.left_s_color = ClassColorSensor(Port.S3) # Sensor de cor esquerdo
        
        except:
            erros.append('3')

        try:
            self.gyro_sensor = ClassGyroSensor(Port.S1) # Sensor giroscópio
        
        except:
            erros.append('1')

        try:
            self.drive = ClassDriveBase(
                self.left_t_motor, self.right_t_motor, self.wheel_diameter, self.distance_between_wheels)
        except:
            """nada"""

        finally:
            self.brick.screen.clear()
            self.brick.screen.draw_text(
                0,50,'Erros:{}'.format(erros),text_color = Color.BLACK ,background_color = None)
            wait(5000)
            self.brick.screen.clear()

        # Valores úteis
        

        # Drive Base

    # Método de inicialização do robô
    def start(self) -> None:
        # Ativando o método de checkagem
        self.brick.screen.clear()

        # Máquina de Estado
        while True:
            # Botões pressionados
            buttons_pressed = self.brick.buttons.pressed()

            self.brick.screen.set_font(tiny_font)#Define a fonte dos textos
            #Escreve a função de cada botão 
            self.brick.screen.draw_text(
                0,10,'Esquerda: Calibração ',text_color = Color.BLACK ,background_color = None)
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
                self.launch_three_test()# Lançamento três
            elif Button.LEFT in buttons_pressed:
                self.check() # Calibração
            elif Button.CENTER in buttons_pressed:
                break
            
            wait(100) # Tempo de 200ms para a não ativação desproposital de alguma função

    # Função de lançamento um
    def launch_one(self):
        self.drive.run_straight(350, 300)
        wait(100)
        self.drive.turn_angle(16, 150)
        wait(100)
        self.drive.run_straight(140, 75)
        wait(100)
        self.drive.run_straight(-480, 900)

    # Função de lançamento dois
    def launch_two(self):
        self.gyro_sensor.reset_angle(0)
        self.left_g_motor.run_with_detection_stop_infinity(1000,75)
        self.drive.run_straight(920,600)
        self.drive.run_until_line(100,self.left_s_color,self.black, 'stop')
        wait(100)
        self.drive.line_follow(50,50,'stop')
        self.drive.line_follow(150,140,'stop')
        self.drive.line_follow(130,100, 'stop')
        self.drive.line_follow(85,25, 'stop')

        self.run_while_moving_grab_1()

        self.right_g_motor.move_grab(1500,4250)

        if self.left_s_color.get_value('reflection') < self.black:
            self.drive.run_during_line(-50,self.left_s_color,self.black)
            self.drive.run_during_line(-50,self.left_s_color,self.white_back_sensors)
            self.drive.pid_run_straight(-50,75)
            self.drive.run_until_line(-100,self.left_s_color,self.white_back_sensors)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

        elif self.left_s_color.get_value('reflection') > 30:
            self.drive.run_during_line(-50,self.left_s_color,self.white_back_sensors)
            self.drive.pid_run_straight(-50,75)
            self.drive.run_until_line(-100,self.left_s_color,self.white_back_sensors)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')
        
        self.drive.gyro_turn(75, 175,25)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_straight(250,150)
        self.drive.run_until_line(125,self.right_s_color,self.white_back_sensors)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_until_line(50,self.right_s_color,self.black)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_until_line(50,self.right_s_color,56)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.left_g_motor.run_with_detection_stop_infinity(-1500,75)
        self.drive.run_until_line(-75,self.right_s_color,self.black)
        self.drive.run_until_line(-50,self.right_s_color,self.white_back_sensors)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.gyro_turn(75, 150, 37.5)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_straight(22.5,50)
        self.left_g_motor.run_with_detection_stop_infinity(1500,65)
        self.drive.gyro_turn(75,150,-30)
        self.drive.run_until_line(-75, self.right_s_color, self.black)
        self.drive.run_until_line(-75, self.right_s_color, self.white_back_sensors)
        self.drive.run_until_line(-75, self.right_s_color, self.black)
        #self.drive.line_squaring(-75, 'left_motor')
        self.drive.gyro_turn(50, 150, -35)
        self.drive.run_straight(-1700,600)
        
        
    def launch_three(self):
        self.black += 2
        self.left_g_motor.run_with_detection_stop_infinity(1000,70)
        self.run_while_moving_grab_2()
        self.drive.run_straight(-30,50)
        self.drive.run_straight(90,50)
        self.drive.run_straight(-135,400)
        self.drive.turn_angle(-55, 100)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_straight(35,100)
        self.drive.line_follow(100,50)
        self.drive.run_straight(200,300)
        self.drive.run_until_line(100,self.right_s_color,self.black)
        self.drive.run_until_line(50,self.right_s_color,self.white_back_sensors)
        self.drive.run_until_line(50,self.left_s_color,self.black)
        self.drive.run_until_line(50,self.left_s_color,self.white_back_sensors)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.left_g_motor.run_with_detection_stop_infinity(1000,85)
        self.drive.gyro_turn_degree(0,-75, 42.5)
        self.drive.run_straight(220,100)
        self.left_g_motor.move_grab(-1000,2700)
        self.left_g_motor.move_grab(1000,1400)
        self.drive.run_until_line(-50, self.right_s_color, self.black)
        self.drive.run_until_line(-50, self.right_s_color, self.white_back_sensors)
        self.drive.run_until_line(-50, self.front_s_color, self.black)
        self.drive.run_until_line(50, self.front_s_color, self.white)
        self.drive.gyro_turn_degree(0,75, -42.5)
        self.drive.line_follow(125, 40)
        self.drive.turn_angle( 15, 25)
        self.left_g_motor.move_grab(-1000, 800)
        
    def launch_three_test(self):
        self.left_g_motor.run_with_detection_stop_infinity(1500, 80)
        self.drive.run_straight(265, 900)
        self.drive.line_follow(50,50,'stop')
        self.drive.line_follow(25,25,'stop')
        self.drive.run_straight(135, 900)
        self.drive.line_follow(100,50,'stop')
        self.drive.line_follow(50,25,'stop')
        self.run_while_moving_grab_3()
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.gyro_turn(75, 150, -15)
        self.left_g_motor.move_grab(-1000, 400)
        self.left_g_motor.move_grab(1000, 400)
        self.drive.gyro_turn(75, 150, 15)
        self.run_while_moving_grab_4()
        self.drive.run_until_line(50, self.left_s_color, self.white_back_sensors)
        self.left_g_motor.run_with_detection_stop_infinity(1500, 80)
        self.drive.run_until_line(-25, self.left_s_color, self.black)
        self.drive.gyro_turn(80, 250, 42.5)
        self.drive.pid_run_straight(240, 100)
        wait(50)
        self.left_g_motor.move_grab(-1500, 2800)
        self.left_g_motor.move_grab(1500, 150)
        self.drive.pid_run_straight(-100, 50)
        wait(50)
        self.drive.run_until_line(-75, self.right_s_color, self.black)
        self.drive.run_until_line(-75, self.front_s_color, self.black)
        self.drive.run_until_line(50, self.front_s_color, 78)
        self.drive.run_until_line(-25, self.front_s_color, self.black)
        self.drive.gyro_turn(75, 150, -42.5)
        self.drive.run_until_line(-50, self.left_s_color, self.white_back_sensors)
        self.drive.run_until_line(50, self.left_s_color, self.black)
        self.drive.run_until_line(50, self.left_s_color, self.white_back_sensors)
        self.drive.run_until_line(-25, self.left_s_color, self.black)
        self.drive.gyro_turn(75, 150, -90)
        self.run_while_moving_grab_5()
        self.drive.run_until_line(75, self.front_s_color, self.white_back_sensors)
        self.drive.run_until_line(50, self.front_s_color, self.black)
        self.left_g_motor.run_with_detection_stop_infinity(1000, 80)
        self.drive.run_straight(50, 75)
        self.drive.run_until_line(50, self.front_s_color, 78)
        self.drive.run_until_line(50, self.front_s_color, self.black)
        self.drive.run_until_line(25, self.front_s_color, 78)
        self.drive.run_until_line(-25, self.front_s_color, self.black)
        self.drive.run_until_line(-25, self.front_s_color, self.threshold)
        self.left_g_motor.move_grab(-500, 600)
        self.drive.gyro_turn(50,100, 10)
        self.left_g_motor.move_grab(-500, 500)



    def launch_four(self):
        self.drive.gyro_turn(80, 250, 90)
        
    def calibration(self) -> None:
        self.gyro_sensor.reset_angle()

        conditional = True
        self.brick.screen.clear()

        while conditional == True:

            buttons_pressed = self.brick.buttons.pressed()

            self.brick.screen.draw_text(0,10,'Cima: Preto', text_color = Color.BLACK, background_color = None)
            self.brick.screen.draw_text(0,25,'Baixo: Branco', text_color = Color.BLACK, background_color = None)
            self.brick.screen.draw_text(0,40,'Meio: Fim', text_color = Color.BLACK, background_color = None)
            
            if Button.UP in buttons_pressed:
                self.brick.screen.clear()
                self.brick.screen.draw_text(0,50,'Coloque no Preto', text_color = Color.BLACK, background_color = None)
                wait(5000)
                self.brick.screen.clear()
                self.black = self.front_s_color.get_value('reflection')
                self.brick.screen.draw_text(0,50,'Preto: {}'.format(self.black))
                wait(5000)
                self.brick.screen.clear()
            
            elif Button.DOWN in buttons_pressed:
                self.brick.screen.clear()
                self.brick.screen.draw_text(0,50,'Coloque no Branco', text_color = Color.BLACK, background_color = None)
                wait(5000)
                self.brick.screen.clear()
                self.white = self.front_s_color.get_value('reflection')
                self.brick.screen.draw_text(0,50,'Branco: {}'.format(self.white))
                wait(5000)
                self.brick.screen.clear()

            elif Button.CENTER in buttons_pressed:
                self.brick.screen.clear()
                conditional = False


    def run_while_moving_grab_1(self):
        global thing
        thing = False

        def moving_grab():
            while thing == False:
                self.right_g_motor.run(500)
            self.right_g_motor.stop('stop')

        threading.Thread(target = moving_grab).start()
        self.drive.run_until_line(200, self.left_s_color, self.white_back_sensors)
        thing = True

    def run_while_moving_grab_2(self):
        global thing
        thing = False

        def moving_grab_2():
            while thing == False:
                self.left_g_motor.run(-525)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_2).start()
        self.drive.run_straight(700, 900)
        thing = True

    def run_while_moving_grab_3(self):
        global thing
        thing = False

        def moving_grab_3():
            while thing == False:
                self.left_g_motor.run(-600)
            self.left_g_motor.stop('hold')

        threading.Thread(target = moving_grab_3).start()
        self.drive.run_straight(362.5, 900)
        thing = True

    def run_while_moving_grab_4(self):
        global thing
        thing = False

        def moving_grab_4():
            while thing == False:
                self.left_g_motor.run(1500)
            self.left_g_motor.stop('hold')

        threading.Thread(target = moving_grab_4).start()
        self.drive.run_until_line(-75, self.left_s_color, self.black)
        thing = True

    def run_while_moving_grab_5(self):
        global thing
        thing = False

        def moving_grab_5():
            while thing == False:
                self.left_g_motor.run(1500)
            self.left_g_motor.stop('hold')

        threading.Thread(target = moving_grab_5).start()
        self.drive.pid_run_straight(150, 550)
        wait(750)
        thing = True

        

    # Retornando o valor de voltagem e corrente da bateria
    def get_battery(self) -> list:
        return [self.brick.battery.voltage(), self.brick.battery.current()]

    

robot = Robot() # Instanciando objeto de robô
robot.start() # Startando a programação
