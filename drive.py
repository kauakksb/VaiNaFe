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

        self.front_s_color = ClassColorSensor(Port.S4)
        self.left_s_color = ClassColorSensor(Port.S3)
        self.right_s_color = ClassColorSensor(Port.S2)
        self.gyro_sensor = ClassGyroSensor(Port.S1)

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
    def line_follow(self, distance, speed, stop_type = 'hold', kp = 1.4, ki = 0.000000005, kd = 3):
        self.reset()
        self.drive.distance()
        self.set_speed(speed)

        # Define os valores de preto e de branco caso eles não tenham sido definidos na calibração
        if self.black == 0:
            self.black = 7
            self.white = 80
        
        threshold = (self.black + self.white) / 2 # Calcula a méida dos valores de preto e branco
        pi = 3.14 # Define o valor de pi
        deviation = 0
        integral = 0
        derivate = 0 
        last_error = 0

        if distance > 0:

            # Executará o código enquanto o robô não tiver percorrido a distância definida
            while distance > self.drive.distance():
                # Mede o desvio,calcula a correção a ser feita e então a executa
                reflection_line_follower = self.front_s_color.get_value('reflection') # Valor de reflexão do sensor
                deviation = reflection_line_follower - threshold # Calculando o desvio feito pelo robô
                integral = integral + deviation
                last_error = deviation
                derivate = deviation - last_error
                self.error_correction = kp *(deviation + ki * integral + kd * derivate)# Calculando a correção a ser feita pelo robô
                self.drive.drive(speed,self.error_correction) # Executando a correção

        elif distance < 0:

            # Executará o código enquanto o robô não tiver percorrido a distância definida
            while distance < self.drive.distance():
                # Mede o desvio,calcula a correção a ser feita e então a executa
                reflection_line_follower = self.front_s_color.get_value('reflection') # Valor de reflexão do sensor
                deviation = reflection_line_follower - threshold # Calculando o desvio feito pelo robô
                integral = integral + deviation
                last_error = deviation
                derivate = deviation - last_error
                self.error_correction = -kp *(deviation + ki * integral + kd * derivate)# Calculando a correção a ser feita pelo robô
                self.drive.drive(-speed,self.error_correction) # Executando a correção

        if stop_type == 'hold':
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':
            self.drive.stop()


    # Move-se de forma retilínea até identificar uma cor definida
    def run_until_line(self, speed, sensor, line, stop_type = 'hold'):

        if line < 30:
            while sensor.get_value('reflection') > line:
                self.drive.drive(speed,0)# O robô se locomove pra frente
            
        elif line > 55:
            while sensor.get_value('reflection') < line:
                self.drive.drive(speed,0)# O robô se locomove pra frente
        
        if stop_type == 'hold':
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':
            self.drive.stop()

    
    def turn_until_line(self, speed, sensor, line, direction, stop_type = 'hold'):

        if direction == 'positive':
            l_speed = -speed
            r_speed = speed

        elif direction == 'negative':
            l_speed = speed
            r_speed = -speed
        
        
        if line < 30:
            while sensor.get_value('reflection') > line:
                self.left_motor.run(l_speed)
                self.right_motor.run(r_speed)

        elif line > 30:
            while sensor.get_value('reflection') < line:
                self.left_motor.run(l_speed)
                self.right_motor.run(r_speed)
            
        if stop_type == 'hold':
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':
            self.drive.stop()


    def motor_line_turn(self, speed, motor, sensor, line, stop_type = 'hold'):

        if motor == 'left_motor':
            if line < 30:
                while sensor.get_value('reflection') > line:
                    self.right_motor.stop('hold')
                    self.left_motor.run(speed)
            
            elif line > 30:
                while sensor.get_value('reflection') < line:
                    self.right_motor.stop('hold')
                    self.left_motor.run(speed)
           
        elif motor == 'right_motor':
            if line < 30:
                while sensor.get_value('reflection') > line:
                    self.left_motor.stop('hold')
                    self.right_motor.run(speed)
            
            elif line > 30:
                while sensor.get_value('reflection') < line:
                    self.left_motor.stop('hold')
                    self.right_motor.run(speed)

        if stop_type == 'hold':
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':
            self.drive.stop()
            

    def newton(self, forca_min, forca_max, dist,w = 0.15,w2 = 0.2, kp = 2.5, ki = 0.001, kd = 3 ):
        self.reset()
        fmin = forca_min
        fmax = forca_max
        variation = fmax - fmin
        way = dist * w
        way2 = dist * w2
        mid_way = dist - way2
        factor = way / variation
        factor_2 = way2 / variation
        i = 0
        error_correction = 0
        target = 0 # Valor alvo a ser seguido pelo robô
        deviation,integral,derivate,last_error = 0,0,0,0 # Variáveis de cálculo do PID
        speed = 0

        if dist > 0:

            while self.drive.distance() < way:
                
                while self.drive.distance() <= factor * (i+1):
                    deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                    integral = integral + deviation # Determina o valor de integral na correção do robô
                    last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                    derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                    error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                    if error_correction <= 0:
                        self.left_motor.dc(fmin + error_correction)
                        self.right_motor.dc(fmin)

                    elif error_correction >= 0:  
                        self.right_motor.dc(fmin + error_correction)
                        self.left_motor.dc(fmin)
                    pass   

                i = i + 1
                fmin += 1

            while self.drive.distance() < mid_way: 

                deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                integral = integral + deviation # Determina o valor de integral na correção do robô
                last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                if error_correction <= 0:
                    self.left_motor.dc(fmax + error_correction)
                    self.right_motor.dc(fmax)

                elif error_correction >= 0:  
                    self.right_motor.dc(fmax + error_correction)
                    self.left_motor.dc(fmax)

            i = 0

            while self.drive.distance() < dist:
                
                while self.drive.distance() <= factor_2 * (i+1) + mid_way:
                    deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                    integral = integral + deviation # Determina o valor de integral na correção do robô
                    last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                    derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                    error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                    if error_correction <= 0:
                        self.left_motor.dc(fmax + error_correction)
                        self.right_motor.dc(fmax)

                    elif error_correction >= 0:
                        self.right_motor.dc(fmax + error_correction)
                        self.left_motor.dc(fmax)
                    pass   

                i = i + 1
                fmax -= 1

            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')
        
        elif dist < 0:

            while self.drive.distance() > way:
                
                while self.drive.distance() <= factor * (i+1):
                    deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                    integral = integral + deviation # Determina o valor de integral na correção do robô
                    last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                    derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                    error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                    if error_correction >= 0:
                        speed = fmax + error_correction
                        self.left_motor.dc(-speed)
                        self.right_motor.dc(-fmin)

                    elif error_correction <= 0:  
                        speed = fmin + error_correction
                        self.right_motor.dc(-speed)
                        self.left_motor.dc(-fmin)
                    pass   

                i = i + 1
                fmin += 1

            while self.drive.distance() > mid_way:

                deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                integral = integral + deviation # Determina o valor de integral na correção do robô
                last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                if error_correction >= 0:
                    speed = fmax + error_correction
                    self.left_motor.dc(-speed)
                    self.right_motor.dc(-fmax)

                elif error_correction <= 0: 
                    speed = fmax + error_correction 
                    self.right_motor.dc(-speed)
                    self.left_motor.dc(-fmax)

            i = 0

            while self.drive.distance() > dist:
                
                while self.drive.distance() >= factor_2 * (i+1) + mid_way:
                    deviation = self.gyro_sensor.get_gyro_angle() - target # Calcula a variação ,em graus ,sofrida pelo robô
                    integral = integral + deviation # Determina o valor de integral na correção do robô
                    last_error = deviation # Guarda o a última variação sofrieda pelo robô a fim de minimizar o próximo erro
                    derivate = deviation - last_error # Determina o valor de derivada na correção do robô
                    error_correction = kp * (deviation + ki * integral + kd * derivate) # Calcula correção a ser feita pelo
                    if error_correction >= 0:
                        speed = fmax + error_correction
                        self.left_motor.dc(-speed)
                        self.right_motor.dc(-fmax)

                    elif error_correction <= 0:
                        speed = fmax + error_correction
                        self.right_motor.dc(-speed)
                        self.left_motor.dc(-fmax)
                    pass   

                i = i + 1
                fmax -= 1

            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')


    def run_during_line(self, speed, sensor, line):
        
        if line > 30:
            while sensor.get_value('reflection') > 58:
                self.drive.drive(speed,0)
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        if line > 70:
            while sensor.get_value('reflection') > 82:
                self.drive.drive(speed,0)
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        if line < 30:
            while sensor.get_value('reflection') < 25:
                self.drive.drive(speed,0)
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

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

        # Determina o tipo de parada que o robô executará
        if stop_type == 'hold':
            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif stop_type == 'stop':
            self.drive.stop()


    def move_robot_to_0(self, speed ):

        
        target = 0 
        if self.gyro_sensor.get_gyro_angle() > 0:
            while self.gyro_sensor.get_gyro_angle() > target:
                self.left_motor.run(speed)
                self.right_motor.run(-speed)
            self.drive.stop()

        if self.gyro_sensor.get_gyro_angle() < 0:
            while self.gyro_sensor.get_gyro_angle() < target:
                self.left_motor.run(-speed)
                self.right_motor.run(speed)
            self.drive.stop()


    def gyro_turn(self, forca_min, forca_max, degree, w = 0.2, w2 = 0.2):
        self.gyro_sensor.reset_angle()

        fmin = forca_min
        fmax = forca_max
        variation = fmax - fmin
        way = degree * w
        way2 = degree * w2
        mid_way = degree - way2
        factor = way/ variation
        factor_2 = way2/ variation
        i = 0

        
        if degree > 0:

            while self.gyro_sensor.get_gyro_angle() < way:

                while self.gyro_sensor.get_gyro_angle() < factor * (1+i):
                    self.left_motor.dc(-fmin)
                    self.right_motor.dc(fmin)
                    pass

            i += 1
            fmin += 1

            while self.gyro_sensor.get_gyro_angle() < mid_way: pass

            i = 0 
            
            while self.gyro_sensor.get_gyro_angle() < degree:

                while self.gyro_sensor.get_gyro_angle() < factor_2 * (1+i) + mid_way:
                    self.left_motor.dc(-fmax)
                    self.right_motor.dc(fmax)
                    pass

                i += 1
                fmax -= 1

            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

        elif degree < 0:

            while self.gyro_sensor.get_gyro_angle() > way:

                while self.gyro_sensor.get_gyro_angle() > (factor * (1+i)) * (-1):
                    self.left_motor.dc(fmin)
                    self.right_motor.dc(-fmin)
                    pass

            i += 1
            fmin += 1

            while self.gyro_sensor.get_gyro_angle() > mid_way: pass

            i = 0 
            
            while self.gyro_sensor.get_gyro_angle() > degree:

                while self.gyro_sensor.get_gyro_angle() > (factor_2 * (1+i) + mid_way) * (-1):
                    self.left_motor.dc(fmax)
                    self.right_motor.dc(-fmax)
                    pass

                i += 1
                fmax -= 1

            self.drive.stop()
            self.left_motor.stop('hold')
            self.right_motor.stop('hold')

    def flip_flop(self, speed, motor, stop_type = 'hold'):
        self.gyro_sensor.reset_angle(0)
        self.left_motor.reset_angle(0)
        pi = 3.14
        axle_track = 121
        distance_between_sensors = 145 
        
        if self.black == 0:
            self.black = 7
            self.white_back_sensors = 68 

        self.threshold = (self.white_back_sensors + self.black) / 2

        if motor == 'left_motor':
            motor1 = self.left_motor
            motor2 = self.right_motor
            sensor = self.left_s_color

        elif motor == 'right_motor':
            motor1 = self.right_motor
            motor2 = self.left_motor
            sensor = self.right_s_color

        while self.sensor.get_value('reflection') > self.black:
            motor2.stop('hold')
            motor1.run(speed)
        self.drive.stop()
        motor1.stop('hold')
        motor2.stop('hold')

        motor_angle = self.motor1.get_angle()

        motor1.reset_angle(0)

        motor_dist = (motor_angle *(pi * self.wheel_diameter)/ 360)

        motor1.run_dist(75, abs(motor_dist*10))

        way = (motor1.get_angle() *(pi * self.wheel_diameter)/ 360)
        tan = abs(way) / distance_between_sensors
        atan = tan * 180 / pi

        self.gyro_sensor.reset_angle(0)
        motor1.reset_angle(0)

        while self.gyro_sensor.get_gyro_angle() < atan:
            motor2.stop('hold')
            motor1.run(speed)
        self.drive.stop()
        motor1.stop('hold')
        motor2.stop('hold')

    def line_squaring(self, speed, motor,stop_type = 'hold'):
        
        if self.black == 0:
            self.black = 18
            self.white_back_sensors = 68

        self.threshold = (self.white_back_sensors + self.black) / 2

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

        while self.sensor.get_value('reflection') > 10:
            motor2.stop('hold')
            motor1.run(speed)
        self.drive.stop()
        motor1.stop('hold')
        motor2.stop('hold')

        if sensor.get_value('reflection') < self.threshold:

            while self.left_s_color.get_value('reflection') and self.right_s_color.get_value('reflection') < self.white_back_sensors:
                self.drive.drive(20, 0)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

            while self.left_s_color.get_value('reflection') and self.right_s_color.get_value('reflection') < self.threshold:
                self.drive.drive(-20, 0)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

        elif sensor2.get_value('reflection') > self.black:

            while sensor2.get_value('reflection') > self.black:
                motor1.stop('hold')
                motor2.run(abs(speed))
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

            while self.left_s_color.get_value('reflection') and self.right_s_color.get_value('reflection') < self.white_back_sensors:
                self.drive.drive(20, 0)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

            while self.left_s_color.get_value('reflection') and self.right_s_color.get_value('reflection') < self.threshold:
                self.drive.drive(-20, 0)
            self.drive.stop()
            motor1.stop('hold')
            motor2.stop('hold')

    def desacceleration(self, dist, line, forca_min, forca_max, sensor):

        variation = forca_max - forca_min
        factor = dist / variation
        speed = forca_max
        final_force = forca_min
        i = 0

        if dist > 0:
        
            if line < 10:
                while sensor.get_value('reflection') > line:
                    self.left_motor.dc(speed)
                    self.right_motor.dc(speed)
                
                self.reset()

                while self.drive.distance() < dist:

                    while self.drive.distance() < factor * (1+i):
                        self.left_motor.dc(speed)
                        self.right_motor.dc(speed)

                    i+=1
                    speed -= 1

                self.drive.stop()
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

            elif line > 10:
                
                while sensor.get_value('reflection') < line:
                    self.left_motor.dc(speed)
                    self.right_motor.dc(speed)
            
                self.reset()
                while self.drive.distance() < dist:

                    while self.drive.distance() < factor * (1+i):
                        self.left_motor.dc(speed)
                        self.right_motor.dc(speed)

                    i+=1
                    speed -= 1

                self.drive.stop()
                self.left_motor.stop('hold')
                self.right_motor.stop('hold')

        elif dist < 0:

            if dist > 0:
        
                if line < 10:
                    while sensor.get_value('reflection') > line:
                        self.left_motor.dc(-speed)
                        self.right_motor.dc(-speed)
                    
                    self.reset()
                    while self.drive.distance() < dist:

                        while self.drive.distance() < factor * (1+i):
                            self.left_motor.dc(-speed)
                            self.right_motor.dc(-speed)
                        
                        i+=1
                        speed -= 1

                    self.drive.stop()
                    self.left_motor.stop('hold')
                    self.right_motor.stop('hold')

                elif line > 10:
                    
                    while sensor.get_value('reflection') < line:
                        self.left_motor.dc(-speed)
                        self.right_motor.dc(-speed)
                
                    self.reset()
                    while self.drive.distance() < dist:

                        while self.drive.distance() < factor * (1+i) *-1:
                            self.left_motor.dc(-speed)
                            self.right_motor.dc(-speed)

                        i+=1
                        speed -= 1

                    self.drive.stop()
                    self.left_motor.stop('hold')
                    self.right_motor.stop('hold')

        
    def line_correction(self, speed, inicial_line, sensor):

        if self.right_s_color.get_value('reflection') > 10:

            while self.right_s_color.get_value('reflection') > 9:
                self.left_t_motor.stop('hold')
                self.right_t_motor.run(50)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

            while self.right_s_color.get_value('reflection') < 65:
                self.left_t_motor.stop('hold')
                self.right_t_motor.run(50)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

            while self.left_s_color.get_value('reflection') < 65:
                self.right_t_motor.stop('hold')
                self.left_t_motor.run(50)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

            while self.right_s_color.get_value('reflection') < 40:
                self.left_t_motor.stop('hold')
                self.right_t_motor.run(-50)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

            while self.left_s_color.get_value('reflection') < 40:
                self.right_t_motor.stop('hold')
                self.left_t_motor.run(-50)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

        elif self.right_s_color.get_value('reflection') < 10:

            while self.right_s_color.get_value('reflection') < 65:
                self.left_t_motor.stop('hold')
                self.right_t_motor.run(50)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

            while self.left_s_color.get_value('reflection') < 65:
                self.right_t_motor.stop('hold')
                self.left_t_motor.run(50)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

            while self.right_s_color.get_value('reflection') < 40:
                self.left_t_motor.stop('hold')
                self.right_t_motor.run(-50)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')

            while self.left_s_color.get_value('reflection') < 40:
                self.right_t_motor.stop('hold')
                self.left_t_motor.run(-50)
            self.left_t_motor.stop('hold')
            self.right_t_motor.stop('hold')
