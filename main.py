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
                self.launch_three()# Lançamento três
            elif Button.LEFT in buttons_pressed:
                self.launch_four() # Lançamento quatro
            elif Button.CENTER in buttons_pressed:
               self.launch_five() # Lançamento cinco
            
            wait(100) # Tempo de 200ms para a não ativação desproposital de alguma função

    # Realiza a calibração dos sensores do robô, definindo os valores de reflexâo de preto e branco
    def calibration(self) -> None:
        self.gyro_sensor.reset_angle() # Reseta o valor do sensor giroscópio

        conditional = True # Variável de condicional da calibração
        self.brick.screen.clear() # Limpa o visor do bloco

        # Realiza a calibração enquanto a condicional for 'True'
        while conditional == True:

            # Botões pressionados 
            buttons_pressed = self.brick.buttons.pressed() 

            # Orientações sobre os botões pressionados
            self.brick.screen.draw_text(0,10,'Cima: Preto', text_color = Color.BLACK, background_color = None)
            self.brick.screen.draw_text(0,25,'Baixo: Branco', text_color = Color.BLACK, background_color = None)
            self.brick.screen.draw_text(0,40,'Meio: Fim', text_color = Color.BLACK, background_color = None)
            
            # Calibra o valor de preto se o botão de cima for pressionado
            if Button.UP in buttons_pressed:
                self.brick.screen.clear()
                self.brick.screen.draw_text(0,50,'Coloque no Preto', text_color = Color.BLACK, background_color = None)
                wait(5000)
                self.brick.screen.clear()
                self.black = self.front_s_color.get_value('reflection')
                self.brick.screen.draw_text(0,50,'Preto: {}'.format(self.black))
                wait(5000)
                self.brick.screen.clear()
            
            # Calibra o valor de branco se o botão de baixo for pressionado
            elif Button.DOWN in buttons_pressed:
                self.brick.screen.clear()
                self.brick.screen.draw_text(0,50,'Coloque no Branco', text_color = Color.BLACK, background_color = None)
                wait(5000)
                self.brick.screen.clear()
                self.white = self.front_s_color.get_value('reflection')
                self.brick.screen.draw_text(0,50,'Branco: {}'.format(self.white))
                wait(5000)
                self.brick.screen.clear()

            # Encerra a calibração se o botão do meio for pressionado
            elif Button.CENTER in buttons_pressed:
                self.brick.screen.clear()
                conditional = False # Muda o valor da condicional para falso e encerra a calibração

    # Realiza a checkagem das portas do EV3 e verifica se há algum cabo desconetado, informando na tela ddo bloco
    def check(self):

        erros = []
        
        try:
            self.right_t_motor = ClassMotor(Port.B) # Motor de tração direito
        
        except:
            erros.append('B') # Adiciona a porta B a lista 'erros' caso ela esteja desconectada

        try:
            self.left_t_motor = ClassMotor(Port.C) # Motor de tração esquerdo
        
        except:
            erros.append('C') # Adiciona a porta C a lista 'erros' caso ela esteja desconectada

        try:
            self.right_g_motor = ClassMotor(Port.A) # Motor de garra direito
        
        except:
            erros.append('A') # Adiciona a porta A a lista 'erros' caso ela esteja desconectada
        
        try:
            self.left_g_motor = ClassMotor(Port.D) # Motor de garra esquerdo
        
        except:
            erros.append('D') # Adiciona a porta D a lista 'erros' caso ela esteja desconectada
        
        try:
            self.front_s_color = ClassColorSensor(Port.S4) # Sensor de cor frontal
        
        except:
            erros.append('4') # Adiciona a porta 4 a lista 'erros' caso ela esteja desconectada
    
        try:
            self.right_s_color = ClassColorSensor(Port.S2) # Sensor de cor direito
        
        except:
            erros.append('2') # Adiciona a porta 2 a lista 'erros' caso ela esteja desconectada

        try:
            self.left_s_color = ClassColorSensor(Port.S3) # Sensor de cor esquerdo
        
        except:
            erros.append('3') # Adiciona a porta 3 a lista 'erros' caso ela esteja desconectada

        try:
            self.gyro_sensor = ClassGyroSensor(Port.S1) # Sensor giroscópio
        
        except:
            erros.append('1') # Adiciona a porta 1 a lista 'erros' caso ela esteja desconectada

        try:
            self.drive = ClassDriveBase(
                self.left_t_motor, self.right_t_motor, self.wheel_diameter, self.distance_between_wheels) # Drive Base
        except:
            """nada"""

        finally:
            self.brick.screen.clear()
            self.brick.screen.draw_text(
                0,50,'Erros:{}'.format(erros),text_color = Color.BLACK ,background_color = None)
            wait(5000)
            self.brick.screen.clear()



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
        self.drive.run_straight(700, 1500)
        thing = True

    def run_while_moving_grab_3(self):
        global thing
        thing = False

        def moving_grab_3():
            while thing == False:
                self.left_g_motor.run(-600)
            self.left_g_motor.stop('hold')

        threading.Thread(target = moving_grab_3).start()
        self.drive.run_straight(398, 1500)
        thing = True

    def run_while_moving_grab_4(self):
        global thing
        thing = False

        def moving_grab_4():
            while thing == False:
                self.left_g_motor.run(1500)
            self.left_g_motor.stop('hold')

        threading.Thread(target = moving_grab_4).start()
        self.drive.run_until_line(-100, self.left_s_color, self.black)
        thing = True

    def run_while_moving_grab_5(self):
        global thing
        thing = False

        def moving_grab_5():
            while thing == False:
                self.left_g_motor.run(1500)
            self.left_g_motor.stop('hold')

        threading.Thread(target = moving_grab_5).start()
        self.drive.run_straight(300, 400)
        thing = True

    def run_while_moving_grab_6(self):
        global thing
        thing = False

        def moving_grab_6():
            while thing == False:
                self.left_g_motor.run(1500)
            self.left_g_motor.stop('hold')

        threading.Thread(target = moving_grab_6).start()
        self.drive.line_follow(120, 30)
        thing = True


    
    # Função de lançamento Um

    """ Robô se locomove para frente e realiza uma curva para se posicionar em frente ao banco. Em seguida 
        se locomove para frente para liberar os cubos nos espaços e deixa o projeto de inovação ao lado do 
        banco. Após isso, se move para trás em direção à área de lançamento."""
    def launch_one(self):
        self.drive.run_straight(350, 300)
        wait(100)
        self.drive.turn_angle(9, 150)
        wait(100)
        self.drive.run_straight(105, 60)
        wait(100)
        self.drive.run_straight(-480, 900)

    # Função de lançamento Dois
    """ Robô vai para frente, no rumo do escorregador, para que o anexo remova os bonequinhos. Em seguida 
        locomove-se para trás, assim, voltando para a área de lançamento. """
    def launch_two(self):
        self.drive.run_straight(500, 900)
        self.drive.run_straight(250, 900)
        self.drive.run_straight(-800, 900)

    # Função de lançamento Três
    def launch_three(self):

        """ O sensor giroscópio é resetado e a garra do anexo é totalmente levantada para não interferir na 
            movimentação do robô """
        self.gyro_sensor.reset_angle(0)
        self.left_g_motor.run_with_detection_stop_infinity(1000,75)

        """ Robô se move para frente com velocidade constante e finaliza o movimento com os motoresse movendo 
            livremente, fazendo com que o atrito e a resistência do ar reduzam sua velocidade."""
        self.drive.pid_run_straight(925,300, 'stop')
        wait(150)
        self.drive.run_until_line(100,self.right_s_color,8, 'stop') # Locomove-se para frente até a linha preta
        wait(100)

        """ Realiza um seguir linha, sendo que, inicialmente move-se lentamente para corrigir a posição do robô 
            e depois se locomove mais rapidamente para poupar tempo e vai reduzindo sua velocidade para corrigir
             sua posição na entrada da esteira"""
        self.drive.line_follow(50, 25, 'stop')
        self.drive.line_follow(100, 150, 'stop')
        self.drive.line_follow(75, 100, 'stop')
        self.drive.line_follow(75, 50, 'stop')
        self.drive.line_follow(50, 25, 'stop')

        self.run_while_moving_grab_1() # Thread para ir em frente até a linha enquanto usa roda do anexo para facilitar a entrada na esteira.

        self.right_g_motor.move_grab(1500,4250) # Move a roda do anexo para realizar a missão esteira
        
        # Verifica em que linha o robô parou pelo sensor de cor esquerdo e realiza uma ré a partir disso
        if self.left_s_color.get_value('reflection') < self.black:
            self.drive.run_during_line(-50,self.left_s_color,self.black)
            self.drive.run_during_line(-50,self.left_s_color,self.white_back_sensors)
            self.drive.run_straight(-50,100)
            self.drive.run_until_line(-100,self.left_s_color,self.white_back_sensors)
        
        elif self.left_s_color.get_value('reflection') > 30:
            self.drive.run_during_line(-50,self.left_s_color,self.white_back_sensors)
            self.drive.run_straight(-50,100)
            self.drive.run_until_line(-100,self.left_s_color,self.white_back_sensors)
        
        # Realiza uma curva e vai reto até o remo fazendo uso das lihas guias para se orientar
        self.drive.gyro_turn(75, 175,25)
        self.drive.run_straight(250,150)
        self.drive.run_until_line(125,self.right_s_color,self.white_back_sensors)
        self.drive.run_until_line(50,self.right_s_color,self.black)
        self.drive.run_until_line(50,self.right_s_color,56)

        # Abaixa a garra, prende a roda do remo, se posiciona através da linhas guias e se move para colocar a roda no circulo
        self.left_g_motor.run_with_detection_stop_infinity(-1500,75)
        self.drive.run_until_line(-75,self.right_s_color,self.black)
        self.drive.run_until_line(-50,self.right_s_color,self.white_back_sensors)
        self.drive.gyro_turn(75, 150, 36.25)
        self.drive.run_straight(22.5,50)
        self.left_g_motor.run_with_detection_stop_infinity(1500,65)

        # Faz uma curva e dá ré até a linha guia onde posiciona-se e volta para a área de lançamento
        self.drive.gyro_turn(75,150,-30)
        self.drive.run_until_line(-75, self.right_s_color, self.black)
        self.drive.run_until_line(-75, self.right_s_color, self.white_back_sensors)
        self.drive.run_until_line(-75, self.right_s_color, self.black)
        self.drive.gyro_turn(50, 125, -33.75)
        self.drive.run_straight(-1700,600)
        
    # Função de lançamento Quatro
    def launch_four(self):
        self.left_g_motor.run_with_detection_stop_infinity(1500, 80)
        self.drive.run_straight(450, 1500)
        self.drive.line_follow(140, 25, 'stop')
        self.run_while_moving_grab_3()
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.gyro_turn(75, 125, -15)
        self.left_g_motor.move_grab(-1500, 500)
        self.left_g_motor.move_grab(1500, 500)
        self.drive.gyro_turn(75, 125, 15)
        self.run_while_moving_grab_4()
        self.drive.run_until_line(35, self.left_s_color, self.white_back_sensors)
        self.left_g_motor.run_with_detection_stop_infinity(1500, 80)
        self.drive.run_until_line(-50, self.left_s_color, self.black)
        self.drive.run_until_line(25, self.left_s_color, 75)
        self.drive.gyro_turn(75, 150, -86.75)
        wait(150)
        self.run_while_moving_grab_5()
        self.drive.run_until_line(100, self.right_s_color, self.white_back_sensors)

        if self.left_s_color.get_value('reflection') < 70:
            self.drive.gyro_turn(75, 125, -4)
            self.drive.run_until_line(50, self.right_s_color, 10)
            self.left_g_motor.run_with_detection_stop_infinity(1000, 80)
            self.drive.run_straight(60, 300)
            self.drive.run_until_line(75, self.front_s_color, 78)
            self.drive.run_until_line(75, self.front_s_color, self.black)
            self.drive.run_until_line(-50, self.front_s_color, 70)
            self.left_g_motor.move_grab(-500, 600)
            self.drive.gyro_turn(75, 125, 12.25)
            self.left_g_motor.move_grab(-500, 600)

        else:
            self.drive.run_until_line(50, self.right_s_color, 10)
            self.left_g_motor.run_with_detection_stop_infinity(1000, 80)
            self.drive.run_straight(60, 300)
            self.drive.run_until_line(75, self.front_s_color, 78)
            self.drive.run_until_line(75, self.front_s_color, self.black)
            self.drive.run_until_line(-50, self.front_s_color, 70)
            self.left_g_motor.move_grab(-500, 600)
            self.drive.gyro_turn(50,100, 8.25)
            self.left_g_motor.move_grab(-500, 600)
        
        self.drive.run_straight(30, 50)
        self.drive.run_until_line(-50, self.right_s_color, 75)
        self.drive.gyro_turn(50, 100, -8.25)
        self.drive.run_until_line(50, self.front_s_color, 10)
        self.drive.gyro_turn(75, 125, -30)
        self.drive.run_straight(50, 200)
        self.drive.run_until_line(50, self.front_s_color, self.black)
        self.drive.gyro_turn(50,100, 35)
        self.drive.line_follow(75, 50, 'hold')
        self.right_g_motor.move_grab(-200, 90)
        wait(500)
        self.drive.line_follow(100, 50, 'stop')
        self.drive.line_follow(100, 100, 'stop')
        self.drive.line_follow(25, 25, 'stop')
        self.run_while_moving_grab_6()
        self.drive.run_until_line(100, self.left_s_color, 10)
        self.left_g_motor.run_with_detection_stop_infinity(1500, 80)
        self.drive.run_until_line(50, self.left_s_color, 72)
        self.drive.gyro_turn(75, 125, 60)
        self.left_g_motor.move_grab(-500,300)
        self.drive.run_straight(130, 300)
        self.left_g_motor.move_grab(-1250, 1900)
        self.left_g_motor.move_grab(1500, 1300)
        self.drive.run_until_line(-50, self.left_s_color, 72)
        self.drive.gyro_turn(75, 125, 30)
        self.drive.run_until_line(-50, self.left_s_color, 9)
        self.drive.gyro_turn(75, 125, 40)
        self.drive.run_straight(375, 300)
        self.left_g_motor.run_with_detection_stop_infinity(1500, 70)
        self.drive.run_straight(-175, 300)
        self.drive.run_until_line(-100, self.left_s_color, self.black)
        self.drive.run_until_line(-75, self.front_s_color, self.black)
        self.drive.run_until_line(50, self.front_s_color, 75)
        self.drive.gyro_turn(75, 125, 40)
        self.drive.run_until_line(300, self.right_s_color, 10 )
        self.drive.run_straight(1000, 900)


    def launch_five(self):
        self.drive.pid_run_straight(500, 100, 300)
        
    
    # Retornando o valor de voltagem e corrente da bateria
    def get_battery(self) -> list:
        return [self.brick.battery.voltage(), self.brick.battery.current()]

    

robot = Robot() # Instanciando objeto de robô
robot.start() # Startando a programação
