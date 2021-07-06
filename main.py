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

tiny_font = Font(size = 14, bold = True) # Fonte pequena

class Robot:
    def __init__(self):
        self.name = "KAL" # Propriedade de nome

        self.brick = EV3Brick() # Bloco EV3
        self.brick.screen.clear() # Limpa o visor do bloco

        # Tentando declarar as variáveis de sensor e robô
        self.wheel_diameter = 56 # Diâmetro da roda em mm
        self.distance_between_wheels = 121 # Distância entre as rodas em mm
        self.white = 90 # Valor de reflexão do preto
        self.black = 12 # Valor de reflexão do branco
        self.white_back_sensors = 62 # Valor de reflexão do branco nos sensores traseiros
        self.threshold = (self.black + self.white) / 2 # Valor de do meio da linha

        erros = [] # Lista de erros das portas na checkagem

        # Motores e sensores
        try:
            self.stopwatch = StopWatch() # Cronômetro
            self.right_t_motor = ClassMotor(Port.B) # Motor de tração direito
            self.left_t_motor = ClassMotor(Port.C) # Motor de tração esquerdo
            self.right_g_motor = ClassMotor(Port.A) # Motor de garra direito
            self.left_g_motor = ClassMotor(Port.D) # Motor de garra esquerdo
            self.front_s_color = ClassColorSensor(Port.S4) # Sensor de cor frontal
            self.right_s_color = ClassColorSensor(Port.S2) # Sensor de cor direito
            self.left_s_color = ClassColorSensor(Port.S3) # Sensor de cor esquerdo
            self.gyro_sensor = ClassGyroSensor(Port.S1) # Sensor giroscópio
            self.drive = ClassDriveBase(
                self.left_t_motor, self.right_t_motor, self.wheel_diameter, self.distance_between_wheels) # Classe Drive para movimento do robô
        
        except:
            '''nada'''

    # Método de inicialização do robô
    def start(self) -> None:
        self.brick.screen.clear() # Limpa o visor do bloco
        
        # State Machine
        while True:
            
            # Botões pressionados
            buttons_pressed = self.brick.buttons.pressed()

            self.brick.screen.set_font(tiny_font)#Define a fonte dos textos
            # Escreve a função de cada botão 
            self.brick.screen.draw_text(
                0,10,'Meio e Baixo: Calibração',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,25,'Meio e Cima: Check',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,40,'Meio e Cima: Check',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,55,'Cima: Saída 1',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,70,'Baixo: Saída 2',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,85,'Esquerda: Saída 3',text_color = Color.BLACK ,background_color = None)
            self.brick.screen.draw_text(
                0,100,'Direita: Saída 4',text_color = Color.BLACK ,background_color = None)

            # Switch de casos de botões pressionados
            if Button.CENTER in buttons_pressed:
                if Button.UP in buttons_pressed:
                    self.check() # Check
                elif Button.DOWN in buttons_pressed:
                    self.calibration() # Calibration
                elif Button.RIGHT in buttons_pressed:
                    self.launch_test() # Launch Test
            elif Button.UP in buttons_pressed:
                self.launch_one() # Launch one
            elif Button.DOWN in buttons_pressed:
                self.launch_two() # Launch two
            elif Button.LEFT in buttons_pressed:
                self.launch_three() # Launch three
            elif Button.RIGHT in buttons_pressed:
                self.launch_four() # Launch four
            

            
            wait(150) # Tempo de 150ms para a não ativação desproposital de alguma função

    # Calibra os sensores do robô e mede os valores de reflexão de preto e de branco
    def calibration(self) -> None:

        wait(1000) # Espera 1 segundo para não ativação desproposital de alguma função

        self.gyro_sensor.reset_angle() # Reseta o sensor giroscópio

        conditional = True # Variável  de condicional da calibração
        self.brick.screen.clear() # Limpa o visor do bloco

        # Executa a calibração  enquanto  a condicional é 'True'
        while conditional == True:

            # Botões pressionados
            buttons_pressed = self.brick.buttons.pressed() 

            # Mostra no visor a função de cada botão
            self.brick.screen.draw_text(0,10,'Cima: Preto', text_color = Color.BLACK, background_color = None)
            self.brick.screen.draw_text(0,25,'Baixo: Branco', text_color = Color.BLACK, background_color = None)
            self.brick.screen.draw_text(0,40,'Meio: Fim', text_color = Color.BLACK, background_color = None)
            
            # Mede o valor de reflexão de preto se o botão de cima for pressionado
            if Button.UP in buttons_pressed:
                self.brick.screen.clear()
                self.brick.screen.draw_text(0,50,'Coloque no Preto', text_color = Color.BLACK, background_color = None)
                wait(5000)
                self.brick.screen.clear()
                self.black = self.front_s_color.get_value('reflection')
                self.brick.screen.draw_text(0,50,'Preto: {}'.format(self.black))
                wait(5000)
                self.brick.screen.clear()
            
            # Mede o valor de reflexão de branco se o botão de  for pressionado
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
                conditional = False # Muda a condicional para 'False'
            
        wait(500) # Espera 500ms para não ativação desproposital de alguma função

    # Realiza a checkagem dos cabos e mostra no visor quais portas estão desconectadas
    def check(self):

        erros = [] # Lista de erros das portas na checkagem
        
        try:
            self.right_t_motor = ClassMotor(Port.B) # Motor de tração direito
        
        except:
            erros.append('B') # Adiciona a porta 'B' à lista 'erros' caso ela esteja desconectada

        try:
            self.left_t_motor = ClassMotor(Port.C) # Motor de tração esquerdo
        
        except:
            erros.append('C') # Adiciona a porta 'C' à lista 'erros' caso ela esteja desconectada

        try:
            self.right_g_motor = ClassMotor(Port.A) # Motor de garra direito
        
        except:
            erros.append('A') # Adiciona a porta 'A' à lista 'erros' caso ela esteja desconectada
        
        try:
            self.left_g_motor = ClassMotor(Port.D) # Motor de garra esquerdo
        
        except:
            erros.append('D') # Adiciona a porta 'D' à lista 'erros' caso ela esteja desconectada
        
        try:
            self.front_s_color = ClassColorSensor(Port.S4) # Sensor de luminosidade frontal
        
        except:
            erros.append('4') # Adiciona a porta '4' à lista 'erros' caso ela esteja desconectada
    
        try:
            self.right_s_color = ClassColorSensor(Port.S2) # Sensor de luminosidade direito
        
        except:
            erros.append('2') # Adiciona a porta '2' à lista 'erros' caso ela esteja desconectada

        try:
            self.left_s_color = ClassColorSensor(Port.S3) # Sensor de luminosidade esquerdo
        
        except:
            erros.append('3') # Adiciona a porta '3' à lista 'erros' caso ela esteja desconectada

        try:
            self.gyro_sensor = ClassGyroSensor(Port.S1) # Sensor giroscópio
        
        except:
            erros.append('1') # Adiciona a porta '1' à lista 'erros' caso ela esteja desconectada

        try:
            self.drive = ClassDriveBase(
                self.left_t_motor, self.right_t_motor, self.wheel_diameter, self.distance_between_wheels) # Drive Base
        except:
            """nothing"""

        finally:
            self.brick.screen.clear() # Limpa o visor 
            self.brick.screen.draw_text(
                0,50,'Erros:{}'.format(erros),text_color = Color.BLACK ,background_color = None) # Mostra no visor a lista de botões desconectados
            wait(5000) # Espera 5s 
            self.brick.screen.clear() # Limpa o visor novamente

    # Função para verificar bloqueios no giro do motor e realizar ações de resposta a fim de corrigir o problema. Missão: Basquete
    def move_grab_with_correction(self, speed, degree, motor):
        motor.reset_angle(0) # reseta o ângulo do motor antes executar a função 
        
        # Move o motor enquanto a sua velociade for menor que 900.
        # Isso evita problemas na correção, pois senão o robô identificaria um bloqueio e aplicaria uma correção desnecessária.
        while motor.get_speed() < 900:
                motor.dc(speed)

        # Move o motor até os graus definidos
        while motor.get_angle() < degree:

            # Move o motor verificando se sua velocidade está acima de um certo valor
            while motor.get_speed() > 1000  and motor.get_angle() < degree:
                motor.dc(speed)
            
            # Ações de respota

            # Se a velociade do motor estiver acima de 800
            if motor.get_speed() > 800:
                pass

            # Se a velocidade do motor estiver abaixo de 800   
            elif motor.get_speed() < 800:
                
                # Se os graus rotacionados até o momento forem menores que 1200
                if motor.get_angle() < 1200:
                    
                    # Faz com  que o motor gire na direção contrária 200 graus
                    angle = motor.get_angle()
                    while motor.get_angle() > angle - 200:
                        motor.dc(-speed)

                    # Move o robô para traz e para frente a fim de melhorar o encaixe do anexo na missão
                    self.drive.run_straight(-15, 50)
                    self.drive.run_straight(30, 50)

                    # Move o motor enquanto a velocidade for menor que 900 para evitar problemas na correção novamente
                    while motor.get_speed() < 900:
                        motor.dc(speed)

                # Se os graus rotacionados até o momento forem menores que 2725
                elif motor.get_angle() < 2725:
                    
                    # Faz com  que o motor gire na direção contrária 300 graus
                    angle = motor.get_angle()
                    while motor.get_angle() > angle - 300:
                        motor.dc(-speed)
                    
                    # Move o robô para frente a fim de melhorar o encaixe do anexo na missão
                    self.drive.run_straight(7.5, 50)

                    # Move o motor enquanto a velocidade for menor que 900 para evitar problemas na correção novamente
                    while motor.get_speed() < 900:
                        motor.dc(speed)

        motor.stop('hold') # Interrompe o movimento do motor


    # Threads para execução de ações simultâneas

    # Usadas no round 3

    # Move o robô até a linha branca enquanto ativa o motor médio direito
    def run_while_moving_grab_1(self):
        global thing
        thing = False

        # Função para ativar o motor e mover o anexo
        def moving_grab():
            while thing == False:
                self.right_g_motor.run(-1500)
            self.right_g_motor.stop('stop')

        threading.Thread(target = moving_grab).start()
        self.drive.run_until_line(200, self.left_s_color, 70, 'hold')
        thing = True


    # Realiza uma ré enquanto ativa o motor médio esquerdo para levantar o y do anexo
    def run_while_moving_grab_2(self):
        global thing
        thing = False

        # Função para ativar o motor e mover o anexo
        def moving_grab_2():
            while thing == False:
                self.left_g_motor.run(450)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_2).start()
        self.drive.run_straight(-150, 175)
        thing = True
        
    # Move-se para frente enquanto ativa o motor para acionar o y do anexo e poupar tempo no round
    def run_while_moving_grab_3(self):
        global thing
        thing = False

        # Função para ativar o motor e mover o anexo
        def moving_grab_3():
            while thing == False:
                self.left_g_motor.run(325)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_3).start()
        self.drive.run_straight(280, 700)
        thing = True
        
    # Anda pra frente enquanto aciona o y do anexo para baixo até o grau específico para executar o aparelho de ginástica
    def run_while_moving_grab_4(self):
        global thing
        thing = False

        # Função para ativar o motor e mover o anexo
        def moving_grab_4():
            while self.left_g_motor.get_angle() > 1675:
                self.left_g_motor.run(-1500)
            self.left_g_motor.stop('hold')

        threading.Thread(target = moving_grab_4).start()
        self.drive.run_until_line(125, self.left_s_color, 70)

    # Realiza uma ré enquanto aciona o y do anexo para cima afim de evitar que o anexo toque o boneco sobre o pneu e o derrube
    def run_while_moving_grab_5(self):
        global thing
        thing = False

        # Função para ativar o motor e mover o anexo
        def moving_grab_5():
            while thing == False:
                self.left_g_motor.run(1500)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_5).start()
        self.drive.run_until_line(-150, self.left_s_color, 10)
        thing = True

    # Realiza a correção sobre a linha enquanto aciona o y do anexo para cima afim de evitar que o anexo toque o boneco sobre o pneu e o derrube
    def run_while_moving_grab_6(self):
        global thing
        thing = False

        # Função para ativar o motor e mover o anexo
        def moving_grab_6():
            while thing == False:
                self.left_g_motor.run(1500)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_6).start()
        self.drive.line_correction(125, 'black', 'right_motor', 'back', 'back')
        thing = True

    # Move-se pra frente enquanto aciona o y para baixo a fim de poupar tempo
    def run_while_moving_grab_7(self):
        global thing
        thing = False

        # Função para ativar o motor e mover o anexo
        def moving_grab_7():
            while thing == False:
                self.left_g_motor.run(-1500)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_7).start()
        self.drive.run_straight(345, 260)
        thing = True

    # Move-se pra frente enquanto aciona o y para baixo a fim de poupar tempo
    def run_while_moving_grab_8(self):
        global thing
        thing = False

        # Função para ativar o motor e mover o anexo
        def moving_grab_8():
            while thing == False:
                self.left_g_motor.run(-1500)
            self.left_g_motor.stop('stop')

        threading.Thread(target = moving_grab_8).start()
        self.drive.run_straight(300, 200)
        thing = True

    # Usadas no round 4

    # Ativa o motor direito para poupa tempo enquanto aciona o motor esquerdo para liberar os cubos
    def run_while_moving_grab_9(self):
        global thing
        thing = False

        # Função para ativar o motor e mover o anexo
        def moving_grab_9():
            while thing == False:
                self.right_g_motor.run(-1500)
            self.right_g_motor.stop('hold')
        
        threading.Thread(target = moving_grab_9).start()
        self.left_g_motor.move_grab(600, 280)
        thing = True

    # Aciona o motor esquerdo para evitar que o anexo fique sobre a área dos cubos enquanto realiza a dança
    def run_while_moving_grab_10(self):
        global thing
        thing = False

        # Movimentos finais do robô(Dança)
        def final_movement():
            i = 0
            while i < 10:
                self.drive.run_straight(30, 100)
                self.drive.run_straight(-30, 100)
                i +=1

        # Função para ativar o motor e mover o anexo
        def moving_grab_10():
            while thing == False:
                self.right_g_motor.run(1500)
            self.right_g_motor.stop('stop')

        threading.Thread(target = moving_grab_10).start()
        final_movement()
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
        self.drive.run_straight(120, 250)
        self.drive.turn(225, 31.25)
        self.drive.run_straight(740, 1000)
        self.left_g_motor.move_grab(1500, 360)
        wait(250)
        self.drive.run_straight(-480, 350)
        self.drive.turn(225, -31.5)
        self.drive.run_straight(-300, 1000)

    # Função de lançamento Três
    def launch_three(self):
        # Robô sai da base e se move até a linha preta e começa seguir linha e se locomove para entrar na esteira
        self.gyro_sensor.reset_angle()
        self.drive.run_straight(1200,1000)
        self.drive.run_until_line(150, self.left_s_color, 60, 'stop')
        self.drive.run_until_line(125, self.left_s_color, 10, 'stop')
        self.drive.line_follow(135, 100, 'stop', 1.4)
        self.drive.line_follow(20, 15, 'stop', 1.4)
        self.drive.line_follow(35, 10, 'stop', 1.4)
        self.run_while_moving_grab_1() # Thread que aciona o motor enquanto o robô se move ,facilitando a entrada na esteira
        self.drive.run_until_line(100, self.left_s_color, 62)

        # Aciona o motor médio direito para realizar a missão da esteira 
        self.right_g_motor.reset_angle() 
        while self.right_g_motor.get_angle() > -6600:
            self.right_g_motor.dc(-100)
        self.right_g_motor.stop('stop')

        # Robô sai da esteira e se posiciona para realizar a máquina de remo
        self.run_while_moving_grab_2() # Thread para subir o y e para fazer a máquina de remo e poupar tempo no round
        self.drive.turn(125, 21.375)
        self.drive.run_straight(155, 300)
        self.left_g_motor.run_with_detection_stop_infinity(-1500, 90)
        wait(100)
        self.gyro_sensor.reset_angle()
        self.drive.turn(125, 32.125)
        self.drive.turn(50, -1.5)
        wait(100)

        # O robô sai da máquina de remo e realiza duas curvas para ficar paralelamente a linha
        self.left_g_motor.move_grab(1500, 525)
        wait(100)

        angle = self.gyro_sensor.get_gyro_angle()
        self.gyro_sensor.reset_angle()
        while self.gyro_sensor.get_gyro_angle() > -angle:
            self.right_t_motor.run(-100)
            self.left_t_motor.run(100)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.gyro_sensor.reset_angle()
        while self.gyro_sensor.get_gyro_angle() > -21.25:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(175)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        # Robô move-se até a linha preta, utilizando-a como referencial para a ré
        self.drive.run_until_line(-110, self.front_s_color, 70, 'hold')
        self.drive.run_until_line(-110, self.front_s_color, 10, 'hold')
        self.drive.run_straight(-110, 225)

        # Robô realiza uma curva até o sensor de cor esquerdo identificar a linha preta
        self.gyro_sensor.reset_angle()
        while self.left_s_color.get_value('reflection') > 10:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(-225)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
    
        self.drive.line_correction(175, 'black', 'right_motor', 'front', 'front') # Realiza uma correção do posicionamento do robô sobre a linha
        wait(100)

        # É feito um movimento para frente passando próximo aos pneus e soltando o bonequinho sobre o pneu grande
        self.run_while_moving_grab_3()
        motor_angle = self.left_g_motor.get_angle()
        degree = 1675 - motor_angle
        self.left_g_motor.move_grab(1500, degree)
        self.left_g_motor.move_grab(-1500, 100)
        self.run_while_moving_grab_4()
        self.drive.run_until_line(100, self.left_s_color, 10)
        
        # Movimentos sobre para posicionar o robô sobre a posição desejada
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

        self.drive.line_correction(150, 'black', 'right_motor', 'front', 'front') # Realiza uma correção do posicionamento do robô sobre a linha
        
        # Robô se posiciona para executar o aparelho de ginástica
        self.drive.run_straight(40, 100)
        self.gyro_sensor.reset_angle()
        while self.gyro_sensor.get_gyro_angle() > -60.5:  
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(175)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')
        self.drive.run_straight(45, 100)

        # Robô executa o aparelho de ginástica e se posiciona para pegar a unidade de saúde
        self.left_g_motor.move_grab(-1500, 1275)
        self.drive.run_straight(-15, 100)
        self.left_g_motor.move_grab(1500, 750)
        self.drive.run_straight(-45, 100)
        self.drive.turn(150, 58.25)
        self.run_while_moving_grab_5() # Thread para acionar o anexo enquanto o robô se move para o anexo não encostar no boneco 

        while self.right_s_color.get_value('reflection') > 70:
            self.left_t_motor.stop('hold')
            self.right_t_motor.run(200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.run_while_moving_grab_6() # Thread para acionar o anexo enquanto o robô se move para o anexo não encostar no boneco
        self.drive.run_straight(-35, 100)
        self.drive.turn(150, 90)
        self.drive.run_until_line(-150, self.right_s_color, 10)

        # Correção para que o posicionamento do robô seja o ideal para pegar a unidade de saúde
        if self.front_s_color.get_value('reflection') < 30:

            while self.right_s_color.get_value('reflection') < 70:
                self.left_t_motor.stop('hold')
                self.right_t_motor.run(-175)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

        else: pass

        # Robô realiza uma ré e duas curvas para pegar a unidade de saúde
        self.drive.run_straight(-100, 100)
        self.drive.turn(225, 25)
        self.drive.turn(50, -25)

        # Robô segue linha a fim de melhorar o seu posicionamento e em seguida move-se para frente usando as linhas como referencias de posicionamento
        self.drive.run_until_line(115, self.right_s_color, 70,'hold')
        self.drive.run_until_line(115, self.right_s_color, 10)
        self.drive.line_follow(100, 90, 'stop', -1.4)
        self.drive.line_follow(60, 20 ,'stop', -1.4)
        self.gyro_sensor.reset_angle()
        self.drive.run_until_line(110, self.right_s_color, 10, 'hold')
        self.run_while_moving_grab_7()
        self.drive.run_until_line(115, self.right_s_color, 70)
        self.drive.run_until_line(115, self.right_s_color, 10)
        self.drive.run_straight(205.5, 145)
        angle = self.gyro_sensor.get_gyro_angle()
        self.gyro_sensor.reset_angle()

        # Correção no caso do robô ter variado na trajetória
        if self.gyro_sensor.get_gyro_angle() < -0.875:

            while self.gyro_sensor.get_gyro_angle() < abs(angle):
                self.right_t_motor.stop('hold')
                self.left_t_motor.run(-125)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

            self.drive.run_straight(30, 60)
        
        else: pass

        self.gyro_sensor.reset_angle()

        # Curva para que o robô vá em direção ao basquete
        while self.gyro_sensor.get_gyro_angle() > -40:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        # Robô vai em direção ao basquete enquanto aciona o y para baixo
        self.run_while_moving_grab_8()
        self.drive.run_straight(25, 125)
        
        self.left_g_motor.reset_angle()
        self.move_grab_with_correction(1500, 2800, self.left_g_motor) # Execução da missão basquete
        self.left_g_motor.move_grab(-1500, 250) # Pequeno movimento do y para baixo com intuito de facilitar a saída do robô da missão
        self.drive.run_until_line(-175, self.left_s_color, 10, 'hold') # Robô sai da missão aplicando uma ré até a linha

        # É feita uma curva até sensor de luminosidade direitar identificar o valor de preto
        while self.right_s_color.get_value('reflection') < 70:
            self.left_t_motor.stop('hold')
            self.right_t_motor.run(200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.drive.line_correction(150, 'black', 'left_motor', 'front', 'back') # Realiza uma correção do posicionamento do robô sobre a linha

        # Robô se posiciona para pegar uma unidade de saúde e em seguida retorna para a área de lançamento
        self.drive.run_straight(-75, 100)
        self.drive.turn(175, 52.5)
        self.drive.run_straight(-60 ,100)
        self.drive.turn(150, -30)
        self.drive.turn(50, 30)
        self.drive.run_straight(1000, 1000)

    # Função de Lançamento Quatro
    def launch_four(self):
        # Robô se move para frente por uma distância definida para realizar o contador de passos
        self.gyro_sensor.reset_angle()
        self.drive.newton(15, 90, 1150, 0.10, 0.001)
        angle = self.gyro_sensor.get_gyro_angle()

        # Verifica-se os graus que o robô variou após a execução da missão e então corrige-se seu posicionamento

        # Se o grau for menor que 37.5 graus, será feita uma correção para posicioná-lo no grau desejado(37.5), elevando a curva da variação
        if abs(angle) < 37.5:
            new_angle = (37.5 - abs(angle)) * -1
            self.gyro_sensor.reset_angle()
            while self.gyro_sensor.get_gyro_angle() > new_angle:
                self.left_t_motor.stop('hold')
                self.right_t_motor.run(-225)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

        # Se o grau for maior que 37.5 graus, será feita uma correção para posicioná-lo no grau desejado(37.5), diminuindo a curva da variação
        elif abs(angle) > 37.5:
            new_angle = ((abs(angle) - 37.5)) * -1
            self.gyro_sensor.reset_angle()
            while self.gyro_sensor.get_gyro_angle() < new_angle:
                self.right_t_motor.stop('hold')
                self.left_t_motor.run(-225)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

        # Se o grau for igual a 37.5, nada será feito
        elif abs(angle) == 37.5:
            pass
        self.gyro_sensor.reset_angle()

        # Robô realiza uma ré utilizando as linhas como referenciais e depois curva para voltar para a 'posição inicial'
        self.drive.run_until_line(-175, self.front_s_color, 70, 'hold')
        self.drive.run_until_line(-150, self.front_s_color, 10)
        self.drive.run_until_line(-125, self.front_s_color, 70)
        self.drive.run_until_line(-100, self.front_s_color, 10)
        self.drive.turn(200, 37.5)

        self.drive.line_follow(70, 50) # Aplica-se um seguir linha para melhorar o posicionamento do robô

        # Move-se para frente até identificar a linha preta com o sensor de cor direito e mais 0.5cm para ficar na posição desejada
        self.drive.run_until_line(125, self.right_s_color, 70, 'hold') 
        self.drive.run_straight(5, 50)

        # Realiza uma curva de 90 graus e posiciona-se sobre a linha
        while self.gyro_sensor.get_gyro_angle() < 90:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(-200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.drive.run_until_line(125, self.left_s_color, 10)
        self.drive.line_correction(150, 'black', 'right_motor','back', 'front') # Realiza uma correção do posicionamento do robô sobre a linha

        # Vai para frente e libera as unidades de saúde sobre a logo Replay
        self.drive.run_straight(60, 100)
        self.right_g_motor.move_grab(-1500, 550)
        self.drive.run_straight(30, 100)
        self.right_g_motor.move_grab(1500, 550)

        # Move-se para a frente, libera os blocos dentro da área e posiciona-se para empurrar o cubinho amarelo para dentro da área
        self.drive.run_straight(490, 1000)
        self.left_g_motor.move_grab(-600, 280)
        self.drive.run_straight(-32.5, 100)
        self.gyro_sensor.reset_angle()

        # É feito uma curva para o robô ativar o motor e empurrar a bocha
        while self.gyro_sensor.get_gyro_angle() > -11.5:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.run_while_moving_grab_9() # Thread para retornar o compartimento dos blocos ao mesmo tempo que empurra bocha

        angle = 1475 - abs(self.right_g_motor.get_angle()) # Verifica os graus rotacionados pelo motor e calcula a quantidade restante

        self.right_g_motor.move_grab(-1500, angle) # Rotaciona os graus restantes
        self.gyro_sensor.reset_angle()

        # Curva para a 'posição inicial' e empurra o cubinho para dentro da área
        while self.gyro_sensor.get_gyro_angle() < 11.5:
            self.right_t_motor.stop('hold')
            self.left_t_motor.run(-200)
        self.left_t_motor.stop('hold')
        self.right_t_motor.stop('hold')

        self.right_g_motor.move_grab(1500, 420)
        self.drive.run_straight(100, 100)

        # Robô realiza uma ré e uma curva para em ir em direção à dança
        self.drive.run_straight(-190, 150)
        self.drive.turn(200, 45)
        self.drive.run_straight(380, 350)
            
        self.run_while_moving_grab_10() # Robô realiza a dança

    # Função para testes   
    def launch_test(self):
        self.move_grab_with_correction(1500, 2800, self.left_g_motor)
        self.left_g_motor.move_grab(-1500, 250)

    # Retornando o valor de voltagem e corrente da bateria
    def get_battery(self) -> list:
        return [self.brick.battery.voltage(), self.brick.battery.current()]

robot = Robot() # Instanciando objeto de robô
robot.start() # Startando a programação

