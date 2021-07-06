#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from sensor import ClassColorSensor,ClassGyroSensor
from motor import ClassMotor

class ClassDriveBase:
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        # Os dois motores (Da classe criada: ClassMotor)
        self.left_motor = left_motor
        self.right_motor = right_motor

        # Instanciando a classe de DriveBase
        self.drive = DriveBase(left_motor.motor, right_motor.motor, wheel_diameter, axle_track)

        # Objetos auxiliares da classe
        self.front_s_color = ClassColorSensor(Port.S4)
        self.left_s_color = ClassColorSensor(Port.S3)
        self.right_s_color = ClassColorSensor(Port.S2)
        self.gyro_sensor = ClassGyroSensor(Port.S1)

        # Valores de configuração
        self.straight_rate = 1000
        self.straight_acceleration = 600
        self.turn_rate = 450
        self.turn_acceleration = 250
        self.wheel_diameter = 56
        self.axle_track = 121
        self.black = 0
        self.white = 0
        self.white_back_sensors = 0
        self.error_correction = 0

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
    def set_acceleration(self, straight_acceleration):
        self.straight_acceleration = straight_acceleration
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
    def line_follow(self, distance, speed, stop_type = 'hold', kp = 2.1, ki = 0.001, kd = 1.5):
        self.reset() # Resetando os motores
        self.drive.distance()
        self.set_speed(speed) # Configuarando velocidade do movimento

        # Define os valores de preto e de branco caso eles não tenham sido definidos na calibração
        if self.black == 0:
            self.black = 7
            self.white = 80
        threshold = (self.black + self.white) / 2 # Calcula a méida dos valores de preto e branco

        deviation,integral,derivate,last_error = 0,0,0,0 # Variáveis de cálculo PID
        pi = 3.14 # Define o valor de pi

        # Se o valor de distância for positivo, o movimento será executado em uma direção
        if distance > 0:

            # Executará o código enquanto o robô não tiver percorrido a distância definida
            while distance > self.drive.distance():
                # Mede o desvio,calcula a correção a ser feita e então a executa
                deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                integral = integral + deviation # Determina o valor de integral na correção do robô
                last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                self.drive.drive(speed,self.error_correction) # Executando a correção

        # Se o valor de distância for negativo, o movimento será executado na direção contrária
        elif distance < 0:

            # Executará o código enquanto o robô não tiver percorrido a distância definida
            while distance < self.drive.distance():
                # Mede o desvio,calcula a correção a ser feita e então a executa
                deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                integral = integral + deviation # Determina o valor de integral na correção do robô
                last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                self.drive.drive(-speed,self.error_correction) # Executando a correção

        # Tipos de parada
        if stop_type == 'hold': # Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop': # Deixa os motores livres após a execução do movimento
            self.drive.stop()


    # Move-se de forma retilínea até identificar uma cor definida
    def run_until_line(self, speed, sensor, line, stop_type = 'hold'):

        # Se o valor de reflexão objetivado for menor que 55, ou seja, próximo do valor de reflexão do preto
        if line < 30:
            while sensor.get_value('reflection') > line:
                self.drive.drive(speed,0)# O robô se locomove até identificar o valor de reflexão da linha

        # Se o valor de reflexão objetivado for maior que 55, ou seja, próximo do valor de reflexão do branco   
        elif line > 55:
            while sensor.get_value('reflection') < line:
                self.drive.drive(speed,0)# O robô se locomove até identificar o valor de reflexão da linha
        
        # Tipos de parada
        if stop_type == 'hold': # Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop': # Deixa os motores livres após a execução do movimento
            self.drive.stop()

    # Realiza uma curva com os dois motores até identificar o valor de reflexão definido
    def turn_until_line(self, speed, sensor, line, direction, stop_type = 'hold'):
        
        # Definindo a direção do movimento
        if direction == 'positive': # Será feito um movimento para a esquerda 
            l_speed = -speed
            r_speed = speed

        elif direction == 'negative': # Será feito um movimento para a direita
            l_speed = speed
            r_speed = -speed
        
        # Se o valor de reflexão objetivado for menor que 35, ou seja, próximo do valor de reflexão do preto
        if line < 35:
            while sensor.get_value('reflection') > line:# O robô realiza uma curva até identificar o valor de reflexão da linha(Dois motores)
                self.left_motor.run(l_speed)
                self.right_motor.run(r_speed)

        # Se o valor de reflexão objetivado for maior que 35, ou seja, próximo do valor de reflexão do branco
        elif line > 35:
            while sensor.get_value('reflection') < line:# O robô realiza uma curva até identificar o valor de reflexão da linha(Dois motores)
                self.left_motor.run(l_speed)
                self.right_motor.run(r_speed)
            
        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()

    # Realiza uma curva com um motor até identificar o valor de reflexão definido
    def motor_line_turn(self, speed, motor, sensor, line, stop_type = 'hold'):

        if motor == 'left_motor': # Motor que será usado para fazer a curva

            # Se o valor de reflexão objetivado for menor que 35, ou seja, próximo do valor de reflexão do preto
            if line < 35:
                while sensor.get_value('reflection') > line:# O robô realiza uma curva até identificar o valor de reflexão da linha(Um motor)
                    self.right_motor.stop('hold')
                    self.left_motor.run(speed)
            
            # Se o valor de reflexão objetivado for maior que 35, ou seja, próximo do valor de reflexão do branco   
            elif line > 30:
                while sensor.get_value('reflection') < line:# O robô realiza uma curva até identificar o valor de reflexão da linha(Um motor)
                    self.right_motor.stop('hold')
                    self.left_motor.run(speed)
           
        elif motor == 'right_motor': # Motor que será usado para fazer a curva

            # Se o valor de reflexão objetivado for maior que 35, ou seja, próximo do valor de reflexão do branco 
            if line < 35:
                while sensor.get_value('reflection') > line:# O robô realiza uma curva até identificar o valor de reflexão da linha(Um motor)
                    self.left_motor.stop('hold')
                    self.right_motor.run(speed)
            
            # Se o valor de reflexão objetivado for maior que 35, ou seja, próximo do valor de reflexão do branco
            elif line > 35:
                while sensor.get_value('reflection') < line:# O robô realiza uma curva até identificar o valor de reflexão da linha(Um motor)
                    self.left_motor.stop('hold')
                    self.right_motor.run(speed)

        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()
            
    # Movimento de MRUV com PID
    def newton(self, force_min, force_max, dist, stop_type = 'hold',w = 0.15,w2 = 0.2, kp = 2.1, ki = 0.001, kd = 1.5 ):
        self.reset() # Resetando os dois motores
        self.gyro_sensor.reset_angle() # Resetando sensor giroscópio

        # Valores de configuração do movimento
        fmin = force_min # Força mínima
        fmax = force_max # Força máxima
        variation = fmax - fmin # Variação da velocidade
        way = dist * w # Trecho de aceleração
        way2 = dist * w2 # Trecho de desaceleração
        mid_way = dist - way2 # Trecho de velocidade constante
        factor = way / variation # Fator de mudança de velocidade(aceleração)
        factor_2 = way2 / variation # Fator de mudança de velocidade(desaleração)
        i = 0 # Variável auxiliar de mudança de velocidade

        # Variáveis do PID
        target,error_correction = 0,0 # Valor alvo a ser seguido pelo robô e correção do erro
        deviation,integral,derivate,last_error = 0,0,0,0 # Variáveis de cálculo do PID
        speed = 0 # Variável auxiliar de velocidade

        # Se a distância for maior que 0(Movimento para frente)
        if dist > 0:

            # Neste trecho da trajetória, é feito a aceleração do movimento enquanto aplica-se o PID(way)
            while self.drive.distance() < way:
                # Aumenta-se em 1 a velocidade conforme o robô percorre uma distância especìfica(factor)
                while self.drive.distance() <= factor * (i+1):
                    deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                    integral = integral + deviation # Determina o valor de integral na correção do robô
                    last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                    derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                    error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo robô

                    # Aplica-se a correção conforme a variação registrada pelo sensor giroscópio
                    if deviation >= 0: # Variação positiva - correção no motor esquerdo
                        self.left_motor.dc(fmin + abs(error_correction))
                        self.right_motor.dc(fmin)

                    elif deviation <= 0: # Variação negativa - correção no motor direito
                        self.right_motor.dc(fmin + abs(error_correction))
                        self.left_motor.dc(fmin)
                    pass   

                i = i + 1
                fmin += 1

            # Neste trecho, mantém-se a velocidade constante enquanto aplica-se o PID(mid_way)
            while self.drive.distance() < mid_way: 
                deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                integral = integral + deviation # Determina o valor de integral na correção do robô
                last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo robô

                # Aplica-se a correção conforme a variação registrada pelo sensor giroscópio
                if deviation >= 0: # Variação positiva - correção no motor esquerdo
                    self.left_motor.dc(fmax + abs(error_correction))
                    self.right_motor.dc(fmax)

                elif deviation <= 0: # Variação negativa - correção no motor direito
                    self.right_motor.dc(fmax + abs(error_correction))
                    self.left_motor.dc(fmax)

            i = 0 # Resetando a variável auxiliar

            # Neste trecho, é feito a desaceleração do movimento enquanto aplica-se o PID(way2)
            while self.drive.distance() < dist:
                # Diminui-se em 1 a velocidade conforme o robô percorre uma distância especìfica(factor_2)
                while self.drive.distance() <= factor_2 * (i+1) + mid_way:
                    deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                    integral = integral + deviation # Determina o valor de integral na correção do robô
                    last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                    derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                    error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo robô

                    # Aplica-se a correção conforme a variação registrada pelo sensor giroscópio
                    if deviation >= 0: # Variação positiva - correção no motor esquerdo
                        self.left_motor.dc(fmax + abs(error_correction))
                        self.right_motor.dc(fmax)

                    elif deviation <= 0: # Variação negativa - correção no motor direito
                        self.right_motor.dc(fmax + abs(error_correction))
                        self.left_motor.dc(fmax)
                    pass   

                i = i + 1
                fmax -= 1
        
        # Se a distância for menor que 0(movimento para trás)
        elif dist < 0:

            # Neste trecho da trajetória, é feito a aceleração do movimento enquanto aplica-se o PID(way)
            while self.drive.distance() > way:
                # Aumenta-se em 1 a velocidade conforme o robô percorre uma distância especìfica(factor)
                while self.drive.distance() <= factor * (i+1):
                    deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                    integral = integral + deviation # Determina o valor de integral na correção do robô
                    last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                    derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                    error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo robô

                    # Aplica-se a correção conforme a variação registrada pelo sensor giroscópio
                    if error_correction >= 0: # Variação positiva - correção no motor esquerdo
                        speed = fmax + error_correction
                        self.left_motor.dc(-speed)
                        self.right_motor.dc(-fmin)

                    elif error_correction <= 0: # Variação negativa - correção no motor direito 
                        speed = fmin + error_correction
                        self.right_motor.dc(-speed)
                        self.left_motor.dc(-fmin)
                    pass   

                i = i + 1
                fmin += 1

            # Neste trecho, mantém-se a velocidade constante enquanto aplica-se o PID(mid_way)
            while self.drive.distance() > mid_way:
                deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                integral = integral + deviation # Determina o valor de integral na correção do robô
                last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo robô

                # Aplica-se a correção conforme a variação registrada pelo sensor giroscópio
                if error_correction >= 0: # Variação positiva - correção no motor esquerdo
                    speed = fmax + error_correction
                    self.left_motor.dc(-speed)
                    self.right_motor.dc(-fmax)

                elif error_correction <= 0: # Variação negativa - correção no motor direito 
                    speed = fmax + error_correction 
                    self.right_motor.dc(-speed)
                    self.left_motor.dc(-fmax)

            i = 0 # Resetando a variável auxiliar

            # Neste trecho, é feito a desaleração do movimento enquanto aplica-se o PID(way2)
            while self.drive.distance() > dist:
                # Diminui-se em 1 a velocidade conforme o robô percorre uma distância especìfica(factor_2)
                while self.drive.distance() >= factor_2 * (i+1) + mid_way:
                    deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                    integral = integral + deviation # Determina o valor de integral na correção do robô
                    last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                    derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                    error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo robô

                    # Aplica-se a correção conforme a variação registrada pelo sensor giroscópio
                    if error_correction >= 0: # Variação positiva - correção no motor esquerdo
                        speed = fmax + error_correction
                        self.left_motor.dc(-speed)
                        self.right_motor.dc(-fmax)

                    elif error_correction <= 0: # Variação negativa - correção no motor direito 
                        speed = fmax + error_correction
                        self.right_motor.dc(-speed)
                        self.left_motor.dc(-fmax)
                    pass   

                i = i + 1
                fmax -= 1

        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()

    # Função para mover o robô enquanto o valor registrado pelo sensor for próximo ao solicitado
    def run_during_line(self, speed, sensor, line, stop_type = 'hold'):
        
        # Se o valor de reflexão objetivado for maior que 35, ou seja, próximo do valor de reflexão do branco
        if line >= 35:
            
            # Se o valor de reflexão objetivado for menor que 70, ou seja, menor que o valor de reflexão branco
            if line <= 70:
                while sensor.get_value('reflection') > line:
                    self.drive.drive(speed,0)

            # Se o valor de reflexão objetivado for maior que 70, ou seja, próximo do valor de reflexão do branco
            elif line >= 70:
                while sensor.get_value('reflection') > line:
                    self.drive.drive(speed,0)

        # Se o valor de reflexão objetivado for menor que 35, ou seja, próximo do valor de reflexão do preto
        elif line <= 35:
            while sensor.get_value('reflection') < line:
                self.drive.drive(speed,0)
            
        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()

    # Movimento de PID com velocidade constante
    def pid_run_straight(self, distance,speed,stop_type = 'hold', kp = 2.5, ki = 0.001, kd = 3):
        self.gyro_sensor.reset_angle(0)
        self.reset()
        self.drive.distance()
        self.set_speed(speed)

        target = 0 # Valor alvo a ser seguido pelo robô
        deviation,integral,derivate,last_error = 0,0,0,0 # Variáveis de cálculo do PID

        # Move-se para frente ou para trás a partir do parâmetro de distância

        # Distância positiva -> move-se para frente
        if distance > 0:
            while distance > self.drive.distance(): # Executa o movimento enquanto a distância determinada for maior que a distância percorrida pelos motores de tração

                deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                integral = integral + deviation # Determina o valor de integral na correção do robô
                last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                self.error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                self.drive.drive(speed,self.error_correction) # Move o robô aplicando a correção

        # Distãncia negativa -> move-se para trás
        elif distance < 0:
            while distance < self.drive.distance(): # Executa o movimento enquanto a distância determinada for menor que a distância percorrida pelos motores de tração

                deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                integral = integral + deviation # Determina o valor de integral na correção do robô
                last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                self.error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                self.drive.drive(-speed,self.error_correction) # Move o robô aplicando a correção

        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()

    # Move o robô para o angulo 0 no atual momento
    def move_robot_to_0(self, speed , stop_type = 'hold'):
        
        target = 0 # Ângulo alvo

        # Se o ângulo atual do robô for maior que 0 
        if self.gyro_sensor.get_gyro_angle() > 0:
            while self.gyro_sensor.get_gyro_angle() > target:
                self.left_motor.run(speed)
                self.right_motor.run(-speed)

        # Se o ângulo atual do robô for menor que 0
        elif self.gyro_sensor.get_gyro_angle() < 0:
            while self.gyro_sensor.get_gyro_angle() < target:
                self.left_motor.run(-speed)
                self.right_motor.run(speed)

        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()

    # Curva usando sensor giroscópio com MRUV
    def gyro_turn(self, force_min, force_max, degree, stop_type = 'hold', w = 0.2, w2 = 0.3):
        self.gyro_sensor.reset_angle()# Reseta o ângulo do sensor giroscópio

        # Variáveis de configuração da função
        fmin = force_min # Força mínima
        fmax = force_max # Força máxima
        variation = fmax - fmin # Variação da velocidade
        way = degree * w # Trecho de aceleração
        way2 = degree * w2 # Trecho de desaleração
        mid_way = degree - way2 # Trecho de velocidade constante
        factor = way/ variation # Fator de mudança de velocidade inicial(aceleração)
        factor_2 = way2/ variation # Fator de mudança de velocidade fina(desaleração)

        i = 0 # Variável auxiliar da mudança de velocidade

        # Se os graus a serem rotacionados forem maiores que 0(Curva para direita)
        if degree > 0:

            # Realiza a acelaração do movimento dentro deste trecho da curva total(way)
            while self.gyro_sensor.get_gyro_angle() < way:
                # Aumenta-se em 1 o valor da velocidade a cada vez que o robô rotaciona uma quantidade de graus específica(factor)
                while self.gyro_sensor.get_gyro_angle() < factor * (1+i):
                    self.left_motor.dc(-fmin)
                    self.right_motor.dc(fmin)
                    pass

                i += 1
                fmin += 1

            # Mantém-se a velocidade máxima nesse trecho da trajetória(mid_way)
            while self.gyro_sensor.get_gyro_angle() < mid_way:
                self.left_motor.dc(-fmax)
                self.right_motor.dc(fmax)

            i = 0 # Resetando variável auxiliar
            
            # Nesse trecho da curva, é feita a desaceleração do movimento(way2)
            while self.gyro_sensor.get_gyro_angle() < degree:
                # Diminui-se em 1 o valor da velocidade a cada vez que o robô rotaciona uma quantidade de graus específica(factor_2)
                while self.gyro_sensor.get_gyro_angle() < factor_2 * (1+i) + mid_way:
                    self.left_motor.dc(-fmax)
                    self.right_motor.dc(fmax)
                    pass

                i += 1
                fmax -= 1

        # Se os graus a serem rotacionados forem menores que 0(Curva para esquerda)
        elif degree < 0:

            # Realiza a acelaração do movimento dentro deste trecho da curva total(way)
            while self.gyro_sensor.get_gyro_angle() > way:
                # Aumenta-se em 1 o valor da velocidade a cada vez que o robô rotaciona uma quantidade de graus específica(factor)
                while self.gyro_sensor.get_gyro_angle() > (factor * (1+i)) * (-1):
                    self.left_motor.dc(fmin)
                    self.right_motor.dc(-fmin)
                    pass

                i += 1
                fmin += 1

            # Mantém-se a velocidade máxima nesse trecho da trajetória(mid_way)
            while self.gyro_sensor.get_gyro_angle() < mid_way:
                self.left_motor.dc(fmax)
                self.right_motor.dc(-fmax)

            i = 0 # Resetando variável auxiliar
            
            # Nesse trecho da curva, é feita a desaceleração do movimento(way2)
            while self.gyro_sensor.get_gyro_angle() > degree:
                # Diminui-se em 1 o valor da velocidade a cada vez que o robô rotaciona uma quantidade de graus específica(factor_2)
                while self.gyro_sensor.get_gyro_angle() > (factor_2 * (1+i) + mid_way) * (-1):
                    self.left_motor.dc(fmax)
                    self.right_motor.dc(-fmax)
                    pass

                i += 1
                fmax -= 1

        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()

    # Curva com velocidade constante usando sensor giroscópio
    def turn(self, speed, degree, stop_type = 'hold'):
        self.gyro_sensor.reset_angle() # Resetando sensor giroscópio

        # Se o grau for maior que 0 (Curva para esquerda)
        if degree > 0:

            while self.gyro_sensor.get_gyro_angle() < degree: # Aplica-se força nos motores enquanto o grau do sensor for menor que o solicitado
                self.left_motor.run(-speed)
                self.right_motor.run(speed)

        # Se o grau for menor que 0(Curva para direita)
        elif degree < 0:
            while self.gyro_sensor.get_gyro_angle() > degree: # Aplica-se força nos motores enquanto o grau do sensor for maior que o solicitado
                self.left_motor.run(speed)
                self.right_motor.run(-speed)

        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()

    # Correção do posicionamento do robô através dos sensores de luminosidade e giroscópio
    def flip_flop(self, speed, motor, correction,stop_type = 'hold'):
        self.left_motor.reset_angle(0) # Resetando o motor esquerdo
        self.right_motor.reset_angle() # Resetando o motor direito
        self.gyro_sensor.reset_angle() # Resetando o sensor giroscópio

        # Valores de configuração
        pi = 3.14
        axle_track = 121
        distance_between_sensors = 145 
        
        # Define os valores de preto e de branco caso eles não tenham sido definidos na calibração
        if self.black == 0:
            self.black = 7
            self.white = 80
        threshold = (self.black + self.white) / 2 # Calcula a méida dos valores de preto e branco

        # Define qual motor será usado para realizar a correção
        if motor == 'left_motor': # Motor direito
            motor1 = self.left_motor
            motor2 = self.right_motor
            sensor = self.left_s_color

        elif motor == 'right_motor': # Motor esquerdo
            motor1 = self.right_motor
            motor2 = self.left_motor
            sensor = self.right_s_color
        
        # Definindo o tipo de correção a ser feita pelo robô de acordo com sua posição inicial sobre a linha

        # A correção será feita movendo o motor para frente
        if correction == 'front':
            
            # Move o motor que está fora da posição deseja até a linha preta
            while self.sensor.get_value('reflection') > 10:
                motor2.stop('hold')
                motor1.run(speed)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

            dist = motor1.get_angle() # Registra-se a distância percorrida em graus

            # Move o motor para a posição inicial
            while motor1.get_angle() > 0:
                motor2.stop('hold')
                motor1.run(-speed)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

            # Calcula-se o arco tangente da curva
            way = (dist *(pi * self.wheel_diameter)/ 360) # Converte os graus em centímetros
            tan = abs(way) / distance_between_sensors # Calcula a tangente
            atan = tan * 180 / pi # Calcula o arco tangente

            self.gyro_sensor.reset_angle(0) # Resetando o sensor giroscópio

            # Curva até os graus do sensor serem maiores que o arco tangente, colocando o robô sobre a posição desejada
            while self.gyro_sensor.get_gyro_angle() < atan:
                motor2.stop('hold')
                motor1.run(speed)

        # A correção será feita movendo o motor para trás
        elif correction == 'back':
            
            # Move o motor que está fora da posição deseja até a linha preta
            while self.sensor.get_value('reflection') > 10:
                motor2.stop('hold')
                motor1.run(-speed)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

            dist = motor1.get_angle() # Registra-se a distância percorrida em graus

            # Move o motor para a posição inicial
            while motor1.get_angle() < 0:
                motor2.stop('hold')
                motor1.run(speed)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')
            
            # Calcula-se o arco tangente da curva
            way = (dist *(pi * self.wheel_diameter)/ 360) # Converte os graus em centímetros
            tan = abs(way) / distance_between_sensors # Calcula a tangente
            atan = tan * 180 / pi # Calcula o arco tangente

            self.gyro_sensor.reset_angle(0) # Resetando o sensor giroscópio

            # Curva até os graus do sensor serem maiores que o arco tangente, colocando o robô sobre a posição desejada
            while self.gyro_sensor.get_gyro_angle() > atan:
                motor2.stop('hold')
                motor1.run(-speed)
        
        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()

    # Correção sobre a linha por meio dos sensores de luminosidade
    def line_squaring(self, speed, motor,stop_type = 'hold'):
        
        # Valores de configuração
        if self.black == 0:
            self.black = 18
            self.white_back_sensors = 68
        self.threshold = (self.white_back_sensors + self.black) / 2

        # Define a configuração dos motores e sensores de acordo com a posição inicial do robô
        if motor == 'left_motor':
            motor1 = self.left_motor
            motor2 = self.right_motor
            sensor = self.left_s_color
            sensor2 = self.right_s_color

        elif motor == 'right_motor':
            motor1 = self.right_motor
            motor2 = self.left_motor
            sensor = self.right_s_color
            sensor2 = self.left_s_color

        # Move o motor que está fora da posição desejada até a linha preta
        while self.sensor.get_value('reflection') > 10:
            motor2.stop('hold')
            motor1.run(speed)
        self.drive.stop()
        motor1.stop('hold')
        motor2.stop('hold')

        # Se o valor do outro sensor for menor que 30, ou seja, se o outro sensor tiver permanecido sobre a linha preta após a curva
        if sensor2.get_value('reflection') < 30:
            
            # Robô se move até os dois sensores identificarem a linha branca
            while self.left_s_color.get_value('reflection') or self.right_s_color.get_value('reflection') < self.white_back_sensors:
                self.drive.drive(20, 0)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

            # Robô se move até os dois sensores identificarem o meio da linha
            while self.left_s_color.get_value('reflection') or self.right_s_color.get_value('reflection') < self.threshold:
                self.drive.drive(-20, 0)
            
        
        # Se o valor do outro sensor for maior que 30, ou seja, se o outro sensor não tiver permanecido sobre a linha preta após a curva
        elif sensor2.get_value('reflection') > self.black:
            
            # Move-se o outro motor para corrigir a posição do sensor
            while sensor2.get_value('reflection') > self.black:
                motor1.stop('hold')
                motor2.run(abs(speed))
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

            # Robô se move até os dois sensores identificarem a linha branca
            while self.left_s_color.get_value('reflection') and self.right_s_color.get_value('reflection') < self.white_back_sensors:
                self.drive.drive(20, 0)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

            # Robô se move até os dois sensores identificarem o meio da linha
            while self.left_s_color.get_value('reflection') and self.right_s_color.get_value('reflection') < self.threshold:
                self.drive.drive(-20, 0)

        # Tipos de parada
        if stop_type == 'hold':# Trava os motores após a execução do movimento
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':# Deixa os motores livres após a execução do movimento
            self.drive.stop()
            

    # Correção sobre a linha por meio dos sensores de luminosidade
    def line_correction(self, speed, inicial_line, inicial_motor_correction, inicial_correction, mid_final_correction, final_correction_speed = 35):
        
        # Se a linha inicial for a preta
        if inicial_line == 'black':

            # Define qual o motor está fora da linha
            if inicial_motor_correction == 'right_motor': # Motor direito
                
                # Define se a correção será para frente ou para trás
                if inicial_correction == 'front':# Correção para frente

                    if self.right_s_color.get_value('reflection') > 10:
                        while self.right_s_color.get_value('reflection') > 9: # Move o motor direito até o sensor direito identificar o preto
                            self.left_motor.stop('hold')
                            self.right_motor.run(speed)
                        self.left_motor.stop('hold')
                        self.right_motor.stop('hold')

                elif inicial_correction == 'back':# Correção para trás
                    
                    if self.right_s_color.get_value('reflection') > 10:
                        while self.right_s_color.get_value('reflection') > 9: # Move o motor direito até o sensor direito identificar o preto
                            self.left_motor.stop('hold')
                            self.right_motor.run(-speed)
                        self.left_motor.stop('hold')
                        self.right_motor.stop('hold')  

            elif inicial_motor_correction == 'left_motor': # Motor esquerdo

                # Define se a correção será para frente ou para trás
                if inicial_correction == 'front': # Correção para frente

                    if self.left_s_color.get_value('reflection') > 10:
                        while self.left_s_color.get_value('reflection') > 9: # Move o motor esquerdo até o sensor esquerdo identificar o preto
                            self.right_motor.stop('hold')
                            self.left_motor.run(speed)
                        self.left_motor.stop('hold')
                        self.right_motor.stop('hold')

                elif inicial_correction == 'back': # Correção para trás

                    if self.left_s_color.get_value('reflection') > 10:
                        while self.left_s_color.get_value('reflection') > 9: # Move o motor esquerdo até o sensor esquerdo identificar o preto
                            self.right_motor.stop('hold')
                            self.left_motor.run(-speed)
                        self.left_motor.stop('hold')
                        self.right_motor.stop('hold')

            elif inicial_motor_correction == 'no one': # Nenhum motor será usado
                if inicial_correction == 'no one': # Na ocorre a correção inicial
                    pass

            # Define se a correção será feita para frente ou para trás   
            if mid_final_correction == 'front': # Correção para frente

                while self.left_s_color.get_value('reflection') < 65: # Move o motor esquerdo até o sensor esquerdo identificar o branco
                    self.right_motor.stop('hold')
                    self.left_motor.run(speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.right_s_color.get_value('reflection') < 65: # Move o motor direito até o sensor direito identificar o branco
                    self.left_motor.stop('hold')
                    self.right_motor.run(speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.right_s_color.get_value('reflection') < 40: # Move o motor direito até o sensor direito identificar o meio da linha
                    self.left_motor.stop('hold')
                    self.right_motor.run(-final_correction_speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.left_s_color.get_value('reflection') < 40: # Move o motor esquerdo até o sensor esquerdo identificar o meio da linha
                    self.right_motor.stop('hold')
                    self.left_motor.run(-final_correction_speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

            elif mid_final_correction == 'back': # Correção para trás

                while self.left_s_color.get_value('reflection') < 65: # Move o motor esquerdo até o sensor esquerdo identificar o branco
                    self.right_motor.stop('hold')
                    self.left_motor.run(-speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.right_s_color.get_value('reflection') < 65: # Move o motor direito até o sensor direito identificar o branco
                    self.left_motor.stop('hold')
                    self.right_motor.run(-speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.right_s_color.get_value('reflection') < 40: # Move o motor direito até o sensor direito identificar o meio da linha
                    self.left_motor.stop('hold')
                    self.right_motor.run(final_correction_speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.left_s_color.get_value('reflection') < 40: # Move o motor esquerdo até o sensor esquerdo identificar o meio da linha
                    self.right_motor.stop('hold')
                    self.left_motor.run(final_correction_speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

        # Se a linha inicial for branca
        elif inicial_line == 'white': 
            # Define qual motor está fora da linha
            if inicial_motor_correction == 'right_motor': # Motor direito

                # Define se a correção inicial será para frente ou para trás
                if inicial_correction == 'front': # Correção para frente

                    if self.right_s_color.get_value('reflection') < 58:
                        while self.right_s_color.get_value('reflection') < 65: # Move o motor direito até identificar o sensor direito identificar o branco
                            self.left_motor.stop('hold')
                            self.right_motor.run(speed)
                        self.left_motor.stop('hold')
                        self.right_motor.stop('hold')

                elif inicial_correction == 'back': # Correção para trás
                    
                    if self.right_s_color.get_value('reflection') < 58:
                        while self.right_s_color.get_value('reflection') > 65: # Move o motor direito até identificar o sensor direito identificar o branco
                            self.left_motor.stop('hold')
                            self.right_motor.run(-speed)
                        self.left_motor.stop('hold')
                        self.right_motor.stop('hold')  

            elif inicial_motor_correction == 'left_motor': # Motor direito

                # Define se a correção inicial será feita para frente ou para trás
                if inicial_correction == 'front': # Correção para frente

                    if self.left_s_color.get_value('reflection') < 58:
                        while self.left_s_color.get_value('reflection') < 65: # Move o motor esquerdo até identificar o sensor esquerdo identificar o branco
                            self.right_motor.stop('hold')
                            self.left_motor.run(speed)
                        self.left_motor.stop('hold')
                        self.right_motor.stop('hold')

                elif inicial_correction == 'back': # Correção para trás

                    if self.left_s_color.get_value('reflection') < 58:
                        while self.left_s_color.get_value('reflection') < 65: # Move o motor esquerdo até identificar o sensor esquerdo identificar o branco
                            self.right_motor.stop('hold')
                            self.left_motor.run(-speed)
                        self.left_motor.stop('hold')
                        self.right_motor.stop('hold')

            # Define se a correção será feita para frente ou para trás
            if mid_final_correction == 'front': # Correção para frente

                while self.left_s_color.get_value('reflection') > 10: # Move o motor esquerdo até identificar o sensor esquerdo identificar o preto
                    self.right_motor.stop('hold')
                    self.left_motor.run(speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.right_s_color.get_value('reflection') > 10: # Move o motor direito até identificar o sensor direito identificar o preto
                    self.left_motor.stop('hold')
                    self.right_motor.run(speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.right_s_color.get_value('reflection') < 40: # Move o motor direito até identificar o sensor direito identificar o meio da linha
                    self.left_motor.stop('hold')
                    self.right_motor.run(-final_correction_speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.left_s_color.get_value('reflection') < 40: # Move o motor esquerdo até identificar o sensor esquerdo identificar o meio da linha
                    self.right_motor.stop('hold')
                    self.left_motor.run(-final_correction_speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

            elif mid_final_correction == 'back': # Correção para trás

                while self.left_s_color.get_value('reflection') > 10: # Move o motor esquerdo até identificar o sensor esquerdo identificar o preto
                    self.right_motor.stop('hold')
                    self.left_motor.run(-speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.right_s_color.get_value('reflection') > 10: # Move o motor direito até identificar o sensor direito identificar o preto
                    self.left_motor.stop('hold')
                    self.right_motor.run(-speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.right_s_color.get_value('reflection') < 40: # Move o motor direito até identificar o sensor direito identificar o meio da linha
                    self.left_motor.stop('hold')
                    self.right_motor.run(final_correction_speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

                while self.left_s_color.get_value('reflection') < 40: # Move o motor esquerdo até identificar o sensor esquerdo identificar o meio da linha
                    self.right_motor.stop('hold')
                    self.left_motor.run(final_correction_speed)
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')
