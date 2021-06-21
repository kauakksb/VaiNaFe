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
import time

tiny_font = Font(size = 14, bold = True)

class Robot:
    def __init__(self):
        self.name = "KAL" # Propriedade de nome

        self.brick = EV3Brick() # Bloco EV3
        self.brick.screen.clear()    
        """
            Limpar o visor! (a partir do self.brick)
        """

        # Tentando declarar as variáveis de sensor e robô
        # Motores

        # Valores Úteis
        self.wheel_diameter = 56 # Diâmetro da roda em mm
        self.distance_between_wheels = 121 # Distância entre as rodas em mm
        self.white = 90 # Valor de reflexão do preto
        self.black = 12 # Valor de reflexão do branco
        self.white_back_sensors = 62 # Valor de reflexão do branco nos sensores traseiros
        self.threshold = (self.black + self.white) / 2 # Valor de do meio da linha

        erros = [] # Lista de erros das portas na checkagem

        # Motores e sensores
        try:
            self.stopwatch = StopWatch()
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

    # Método de inicialização do robô
    def start(self) -> None:
        # Ativando o método de checkagem
        self.brick.screen.clear()
        conditional = True

        while conditional == True:

            first_buttons_pressed = self.brick.buttons.pressed()

            self.brick.screen.set_font(tiny_font)
            self.brick.screen.draw_text(
                0,10, 'Deseja fazer o check?',text_color = Color.BLACK ,background_color = None )
            self.brick.screen.draw_text(
                0,40, 'Sim: Direita',text_color = Color.BLACK ,background_color = None )
            self.brick.screen.draw_text(
                0,55, 'Não: Esquerda',text_color = Color.BLACK ,background_color = None )

            if Button.RIGHT in first_buttons_pressed:
                self.check()
                conditional = False

            elif Button.LEFT in first_buttons_pressed:
                self.brick.screen.clear()
                conditional = False

        wait(1000)
        

        # State Machine
        while True:
            
            # Buttons pressed
            buttons_pressed = self.brick.buttons.pressed()

            self.brick.screen.set_font(tiny_font)#Define a fonte dos textos
            # Escreve a função de cada botão 
            self.brick.screen.draw_text(
                0,10,'Meio e Baixo: Calibração',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,25,'Meio e Cima: Check',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,40,'Cima: Saída 1',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,55,'Baixo: Saída 2',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,70,'Esquerda: Saída 3',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,85,'Direita: Saída 4',text_color = Color.BLACK ,background_color = None)

            # Switch de casos de botões pressionados
            if Button.CENTER in buttons_pressed:
                if Button.UP in buttons_pressed:
                    self.check()
                elif Button.DOWN in buttons_pressed:
                    self.calibration()
                elif Button.RIGHT in buttons_pressed:
                    self.launch_test()
            elif Button.UP in buttons_pressed:
                wait(100)
                self.launch_one() # Launch one
            elif Button.DOWN in buttons_pressed:
                wait(100)
                self.launch_two() # Launch two
            elif Button.LEFT in buttons_pressed:
                wait(100)
                self.launch_three() # Launch three
            elif Button.RIGHT in buttons_pressed:
                wait(100)
                self.launch_four() # Launch four
            

            
            wait(200) # Tempo de 200ms para a não ativação desproposital de alguma função

    # Calibrates the sensors of the robot. Measure the reflection values at the black and the white lines.
    def calibration(self) -> None:
        wait(1000)
        self.gyro_sensor.reset_angle() # Reset the gyroscope sensor value

        conditional = True # Conditional variable of calibration
        self.brick.screen.clear() # Clear the screen of the brick

        # Do the calibration while conditional is 'True'
        while conditional == True:

            # Buttons pressed in the brick
            buttons_pressed = self.brick.buttons.pressed() 

            # Print on brick's screen what each button do
            self.brick.screen.draw_text(0,10,'Cima: Preto', text_color = Color.BLACK, background_color = None)
            self.brick.screen.draw_text(0,25,'Baixo: Branco', text_color = Color.BLACK, background_color = None)
            self.brick.screen.draw_text(0,40,'Meio: Fim', text_color = Color.BLACK, background_color = None)
            
            # Measure the reflection value of the black line if the button up is pressed
            if Button.UP in buttons_pressed:
                self.brick.screen.clear()
                self.brick.screen.draw_text(0,50,'Coloque no Preto', text_color = Color.BLACK, background_color = None)
                wait(5000)
                self.brick.screen.clear()
                self.black = self.front_s_color.get_value('reflection')
                self.brick.screen.draw_text(0,50,'Preto: {}'.format(self.black))
                wait(5000)
                self.brick.screen.clear()
            
            # Measure the reflection value of the white line if the button up is pressed
            elif Button.DOWN in buttons_pressed:
                self.brick.screen.clear()
                self.brick.screen.draw_text(0,50,'Coloque no Branco', text_color = Color.BLACK, background_color = None)
                wait(5000)
                self.brick.screen.clear()
                self.white = self.front_s_color.get_value('reflection')
                self.brick.screen.draw_text(0,50,'Branco: {}'.format(self.white))
                wait(5000)
                self.brick.screen.clear()

            # If the left button is pressed, the calibration is finished
            elif Button.CENTER in buttons_pressed:
                self.brick.screen.clear()
                conditional = False # Change the conditional value to False and finish the calibration
            
        wait(500)

    # Check the EV3 ports and inform what port is disconnected
    def check(self):

        erros = []
        
        try:
            self.right_t_motor = ClassMotor(Port.B) # Right traction motor
        
        except:
            erros.append('B') # Add port B to list 'erros' if it is disconnected

        try:
            self.left_t_motor = ClassMotor(Port.C) # Left traction motor
        
        except:
            erros.append('C') # Add port C to list 'erros' if it is disconnected 

        try:
            self.right_g_motor = ClassMotor(Port.A) # Right grab motor
        
        except:
            erros.append('A') # Add port A to list 'erros' if it is disconnected
        
        try:
            self.left_g_motor = ClassMotor(Port.D) # Left grab motor
        
        except:
            erros.append('D') # Add port D to list 'erros' if it is disconnected
        
        try:
            self.front_s_color = ClassColorSensor(Port.S4) # Front luminosity sensor
        
        except:
            erros.append('4') # Add port 4 to list 'erros' if it is disconnected
    
        try:
            self.right_s_color = ClassColorSensor(Port.S2) # Right luminosity sensor
        
        except:
            erros.append('2') # Add port 2 to list 'erros if it is disconnected a porta 2 a lista 'erros' caso ela esteja desconectada

        try:
            self.left_s_color = ClassColorSensor(Port.S3) # Left luminosity sensor
        
        except:
            erros.append('3') # Add port 3 to list 'erros' if it is disconnected

        try:
            self.gyro_sensor = ClassGyroSensor(Port.S1) # Gyroscope sensor
        
        except:
            erros.append('1') # Add port 1 to list 'erros' if it is disconnected

        try:
            self.drive = ClassDriveBase(
                self.left_t_motor, self.right_t_motor, self.wheel_diameter, self.distance_between_wheels) # Drive Base
        except:
            """nothing"""

        finally:
            self.brick.screen.clear()
            self.brick.screen.draw_text(
                0,50,'Erros:{}'.format(erros),text_color = Color.BLACK ,background_color = None)
            wait(5000)
            self.brick.screen.clear()

    def move_grab_with_correction(self, speed, degree, motor):
        motor.reset_angle(0)
        
        while motor.get_speed() < 900:
                motor.dc(speed)

        while motor.get_angle() < degree:

            while motor.get_speed() > 1000  and motor.get_angle() < degree:
                print('angl:',motor.get_angle())
                print('vel:',motor.get_speed())
                motor.dc(speed)
            
            if motor.get_speed() > 800:
                pass
                
            elif motor.get_speed() < 800:

                if motor.get_angle() < 5000:

                    angle = motor.get_angle()
                    while motor.get_angle() > angle - 360:
                        motor.dc(-speed)

                    self.drive.run_straight(-15, 50)
                    self.drive.run_straight(35, 50)

                    while motor.get_speed() < 900:
                        motor.dc(speed)

                
                elif motor.get_angle() < 14000:

                    angle = motor.get_angle()
                    while motor.get_angle() > angle - 400:
                        motor.dc(-speed)

                    self.drive.run_straight(-15, 50)

                    while motor.get_speed() < 900:
                        motor.dc(speed)

        motor.stop('hold')


    def run_while_moving_grab_1(self):
        global thing
        thing = False

        def moving_grab():
            while thing == False:
                self.right_g_motor.run(-1500)
            self.right_g_motor.stop('stop')

        threading.Thread(target = moving_grab).start()
        self.drive.run_until_line(200, self.left_s_color, 70, 'hold')
        thing = True


    def run_while_moving_grab_2(self):
        global thing
        thing = False

        def moving_grab_2():
            while thing == False:
                self.left_g_motor.run(225)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_2).start()
        self.drive.run_straight(-150, 175)
        thing = True
        

    def run_while_moving_grab_3(self):
        global thing
        thing = False

        def moving_grab_3():
            while thing == False:
                self.left_g_motor.run(300)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_3).start()
        self.drive.run_straight(280, 700)
        thing = True
        

    def run_while_moving_grab_4(self):
        global thing
        thing = False

        def moving_grab_4():
            while self.left_g_motor.get_angle() > 1675:
                self.left_g_motor.run(-1500)
            self.left_g_motor.stop('hold')

        threading.Thread(target = moving_grab_4).start()
        self.drive.run_until_line(125, self.left_s_color, 70)

    def run_while_moving_grab_5(self):
        global thing
        thing = False

        def moving_grab_5():
            while thing == False:
                self.left_g_motor.run(0)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_5).start()
        self.drive.turn(150, -86.25)
        thing = True

    def run_while_moving_grab_8(self):
        global thing
        thing = False

        def moving_grab_8():
            while thing == False:
                self.left_g_motor.run(-1500)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_8).start()
        self.drive.line_follow(80, 70, 'stop', -1.5)
        thing = True

    def run_while_moving_grab_81(self):
        global thing
        thing = False

        def moving_grab_81():
            while thing == False:
                self.left_g_motor.run(-1500)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_81).start()
        self.drive.line_follow(60, 20, 'stop', -1.5)
        thing = True

    def run_while_moving_grab_9(self):
        global thing
        thing = False

        def moving_grab_9():
            while thing == False:
                self.left_g_motor.run(-1500)
            self.right_g_motor.stop('stop')

        threading.Thread(target = moving_grab_9).start()
        self.drive.run_straight(-390, 400)
        thing = True

    def run_while_moving_grab_10(self):
        global thing
        thing = False

        def moving_grab_10():
            while thing == False:
                self.right_g_motor.run(-1500)
            self.right_g_motor.stop('stop')

        threading.Thread(target = moving_grab_10).start()
        self.drive.run_straight(-200, 225)
        thing = True
    
    # Função de lançamento Um

    """ Robô se locomove para frente e realiza uma curva para se posicionar em frente ao banco. Em seguida 
        se locomove para frente para liberar os cubos nos espaços e deixa o projeto de inovação ao lado do 
        banco. Após isso, se move para trás em direção à área de lançamento."""
    def launch_one(self):
        self.drive.run_straight(350, 750)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        wait(100)
        self.drive.turn(150, -8.125)
        wait(100)
        self.drive.run_straight(110, 55)
        wait(100)
        self.left_g_motor.move_grab(1500, 500)
        self.drive.run_straight(-480, 900)

    # Função de lançamento Dois
    """ Robô vai para frente, no rumo do escorregador, para que o anexo remova os bonequinhos. Em seguida 
        locomove-se para trás, assim, voltando para a área de lançamento. """
    def launch_two(self):
        self.drive.run_straight(110, 250)
        self.drive.turn(225, 30)
        self.drive.run_straight(600, 750)
        self.left_g_motor.move_grab(1500, 360)
        wait(250)
        self.drive.run_straight(-480, 350)
        self.drive.turn(225, -31.5)
        self.drive.run_straight(-300, 1000)

    # Função de lançamento Três
    def launch_three(self):
        self.gyro_sensor.reset_angle()
        self.drive.run_straight(1200,1000)
        self.drive.run_until_line(150, self.left_s_color, 60, 'stop')
        self.drive.run_until_line(125, self.left_s_color, 10, 'stop')
        self.drive.line_follow(135, 100, 'stop', 1.5)
        self.drive.line_follow(75, 20, 'stop', 1.5)
        self.run_while_moving_grab_1()

                
        self.right_g_motor.reset_angle()
        
        while self.right_g_motor.get_angle() > -6600:
            self.right_g_motor.run(-100)
        self.right_g_motor.stop('stop')

        self.run_while_moving_grab_2()
        
        self.drive.turn(125, 23)
        self.drive.run_straight(146.25, 300)

        self.left_g_motor.run_with_detection_stop_infinity(-1500, 90)
        wait(100)

        self.gyro_sensor.reset_angle()

        self.drive.turn(125, 32)
        wait(100)
        self.left_g_motor.move_grab(1500, 400)
        wait(100)
        
        angle = self.gyro_sensor.get_gyro_angle()
        self.gyro_sensor.reset_angle()

        while self.gyro_sensor.get_gyro_angle() > -angle:
            self.right_t_motor.run(-150)
            self.left_t_motor.run(150)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.drive.run_until_line(-175, self.front_s_color, 10, 'stop')
        self.drive.run_until_line(-150, self.front_s_color, 70, 'stop')
        self.drive.run_until_line(-125, self.front_s_color, 10, 'hold')
        self.drive.run_until_line(-100, self.front_s_color, 65)
        
        angle = 90 - 23

        self.gyro_sensor.reset_angle()

        while self.gyro_sensor.get_gyro_angle() < angle:
            self.left_t_motor.stop('hold')
            self.right_t_motor.run(225)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.drive.run_until_line(100, self.right_s_color, 10)
    
        self.drive.run_until_line(100, self.right_s_color, 10)
        if self.left_s_color.get_value('reflection') > 40:
            self.drive.line_correction(175, 'black', 'left_motor', 'front', 'front')

        elif self.left_s_color.get_value('reflection') < 40:
            self.drive.line_correction(175, 'black', 'no one', 'no one', 'front')
        wait(100)
        self.run_while_moving_grab_3()
        motor_angle = self.left_g_motor.get_angle()
        degree = 2200 - motor_angle
        self.left_g_motor.move_grab(1500, degree)
        self.left_g_motor.move_grab(-1500, 525)
        self.run_while_moving_grab_4()
        self.drive.run_until_line(100, self.left_s_color, 10)
        
        while self.right_s_color.get_value('reflection') < 70:
            self.left_t_motor.stop('hold')
            self.right_t_motor.run(-125)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        while self.left_s_color.get_value('reflection') < 70:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(-125)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        while self.left_s_color.get_value('reflection') > 10:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(125)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.drive.line_correction(150, 'black', 'right_motor', 'front', 'front')

        self.gyro_sensor.reset_angle()

        while self.gyro_sensor.get_gyro_angle() > -60:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(175)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.drive.run_straight(80, 100)

        self.left_g_motor.move_grab(-1500, 1150)
        self.drive.run_straight(-15, 100)
        self.left_g_motor.move_grab(1500, 825)
        self.drive.run_straight(-45, 100)
        self.drive.turn(150, 57.5)
        self.drive.run_until_line(-150, self.left_s_color, 10)

        while self.right_s_color.get_value('reflection') > 70:
            self.left_t_motor.stop('hold')
            self.right_t_motor.run(200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.drive.line_correction(125, 'black', 'right_motor', 'back', 'back')
        self.drive.run_straight(-35, 100)
        self.drive.turn(150, 90)
        self.drive.run_until_line(-150, self.right_s_color, 10)

        if self.front_s_color.get_value('reflection') < 30:

            while self.right_s_color.get_value('reflection') < 70:
                self.left_t_motor.stop('hold')
                self.right_t_motor.run(-175)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

        else: pass

        self.drive.run_straight(-57.5, 100)
        self.drive.run_until_line(110, self.right_s_color, 10)
        self.run_while_moving_grab_8()
        self.run_while_moving_grab_81()
        self.drive.run_until_line(110, self.right_s_color, 10, 'stop') # Transformar em um único movimento (reduzir tempo)
        self.drive.run_straight(345, 500)
        self.drive.run_until_line(110, self.right_s_color, 70, 'stop')
        self.drive.run_until_line(110, self.right_s_color, 10)
        self.drive.run_straight(191.25, 300)

        self.gyro_sensor.reset_angle()

        while self.gyro_sensor.get_gyro_angle() > -40.75:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')


        self.drive.run_straight(300, 250)
        self.drive.run_straight(50, 100)
        
        self.left_g_motor.reset_angle()
        self.left_g_motor.move_grab(1500, 2850)
        self.left_g_motor.move_grab(-1500, 250)
        self.drive.run_straight(-120, 750)
        self.drive.run_until_line(-150, self.left_s_color, 10, 'hold')

        while self.right_s_color.get_value('reflection') < 70:
            self.left_t_motor.stop('hold')
            self.right_t_motor.run(200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.drive.line_correction(150, 'black', 'left_motor', 'front', 'back')

        self.drive.run_straight(-35, 100)
        self.drive.turn(175, 47.5)
        self.drive.run_straight(-35 ,100)
        self.drive.run_straight(1000, 1000)


    def launch_four(self):
        self.gyro_sensor.reset_angle()
        self.drive.newton(15, 90, 1150, 0.10, 0.001)
        self.drive.run_straight(-30, 100)
        self.drive.run_straight(200, 350)
        angle = self.gyro_sensor.get_gyro_angle()

        if abs(angle) < 37.5:
            new_angle = (37.5 - abs(angle)) * -1
            print(self.gyro_sensor.get_gyro_angle())
            self.gyro_sensor.reset_angle()
            while self.gyro_sensor.get_gyro_angle() > new_angle:
                self.left_t_motor.stop('hold')
                self.right_t_motor.run(-225)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

        elif abs(angle) > 37.5:
            new_angle = ((abs(angle) - 37.5)) * -1
            self.gyro_sensor.reset_angle()
            while self.gyro_sensor.get_gyro_angle() < new_angle:
                self.right_t_motor.stop('hold')
                self.left_t_motor.run(-225)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

        elif abs(angle) == 37.5:
            pass
        self.gyro_sensor.reset_angle()

        self.drive.run_until_line(-175, self.front_s_color, 70, 'hold')
        self.drive.run_until_line(-150, self.front_s_color, 10)
        self.drive.run_until_line(-125, self.front_s_color, 70)
        self.drive.run_until_line(-100, self.front_s_color, 10)
        
        self.drive.turn(200, 37.5)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.line_follow(80, 40)
        self.gyro_sensor.reset_angle()
        self.drive.run_until_line(125, self.right_s_color, 70, 'hold')
        while self.gyro_sensor.get_gyro_angle() < 90:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(-200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_until_line(125, self.left_s_color, 10)
        self.drive.line_correction(150, 'black', 'right_motor','back', 'front')
        self.drive.run_straight(635, 1000)
        self.right_g_motor.move_grab(-1500, 1200)
        self.run_while_moving_grab_9()
        wait(250)
        self.right_g_motor.move_grab(1500, 1200)
        
        while self.left_g_motor.get_angle() < 30000000:
            self.left_g_motor.run(100)
        self.left_g_motor.stop('hold')
        
    def launch_test(self):
        self.run_while_moving_grab_6()
        print(self.left_g_motor.get_angle())


    # Retornando o valor de voltagem e corrente da bateria
    def get_battery(self) -> list:
        return [self.brick.battery.voltage(), self.brick.battery.current()]

    

robot = Robot() # Instanciando objeto de robô
robot.start() # Startando a programação

