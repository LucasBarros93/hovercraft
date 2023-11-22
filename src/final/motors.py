#!/usr/bin/env python3

import time
import pigpio


#NOTA: Não estou setando pino nenhum como output, fica ligado nisso

class ESC:

    MIN_WIDTH = 1040 # Abaixo de 1040 a ESC não liga
    MAX_WIDTH = 2000

    def __init__(self, pin1:int, pin2:int = None)-> None:
        self.conn = pigpio.pi()
        self.pin1 = pin1
        self.pin2 = pin2
        
        self.conn.set_mode(self.pin1, pigpio.OUTPUT)  #pigpio.PWM(12)  
        
        self.conn.set_PWM_frequency(self.pin1, 50)
        
        if self.pin2 != None:
            self.conn.set_mode(self.pin2, pigpio.OUTPUT)
            self.conn.set_PWM_frequency(self.pin2, 50)


    def pwm(self, width: int)-> None:
        self.conn.set_servo_pulsewidth(self.pin1, width)
        if self.pin2 != None:
            self.conn.set_servo_pulsewidth(self.pin2, width)
        

    def calibrate(self)-> None:
        print('"Menor" velocidade')
        self.pwm(1000)
        time.sleep(3)
        
        
    def arm(self)-> None:
        #print("armando")
        self.pwm(self.MIN_WIDTH)
        time.sleep(2)
        
        print("Armado")
        
    def halt(self)-> None:
        print("Parando")
        self.pwm(1000)
        
        print("Ta safe")
        print("Desligando GPIO.")
        
        #self.conn.stop()
        
        print("Já era")
    
    def manual_control(self)-> None:
        char = input()
        veloc = self.conn.get_servo_pulsewidth(self.pin1)
        
        while char != 'x':
            if char == 'q':
                veloc += 10
                self.pwm(veloc)
            
            if char == 'a':
                veloc -= 10
                self.pwm(veloc)
            
            if char == 'e':
                veloc += 100
                self.pwm(veloc)
            
            if char == 'd':
                veloc -= 100
                self.pwm(veloc)
            
            print(veloc)
            char = input()
            
    def control(self, width:int)-> None:
        if width < self.MAX_WIDTH and width > self.MIN_WIDTH:
            self.pwm(width)
    
    def test(self) ->None:
        self.pwm(self.MIN_WIDTH)
        
        step = 100
        print("acelerando")
        for veloc in range(self.MIN_WIDTH, self.MAX_WIDTH, step):
            self.pwm(veloc)
            print(veloc)
            time.sleep(1)
        
        time.sleep(2)  
        print("parando")
        for veloc in range(self.MAX_WIDTH, self.MIN_WIDTH, -step):
            print(veloc)
            self.pwm(veloc)
            time.sleep(.5)  
                

class Servo:
    MIN_WIDTH = 1050 # Menor ângulo
    MAX_WIDTH = 1650 # Maior ângulo
     
    def __init__(self, pin:int)-> None:
        self.pin = pin
        self.conn = pigpio.pi()
        
    def test(self) -> None:
        self.conn.set_servo_pulsewidth(self.pin ,1500) # centro
        time.sleep(3)
        
        self.conn.set_servo_pulsewidth(self.pin ,1000) # um ppouco anti horario
        time.sleep(3)
        
        self.conn.set_servo_pulsewidth(self.pin ,2000) # um pouco horario
        time.sleep(3)
        
    def manual_control(self)-> None:
        width = input('Qual o "ângulo":')
        
        #tem q converter pra graus, mas fdc
        while width != "x":
            while int(width) < self.MIN_WIDTH or int(width) > self.MAX_WIDTH:
                width = input("ta fazendo merda: ")    
                
            self.conn.set_servo_pulsewidth(self.pin, int(width))
            width = input('Ângulo novo: ')
            
    def control(self, pos:int)-> None:
        if pos < self.MAX_WIDTH and pos > self.MIN_WIDTH:
            self.conn.set_servo_pulsewidth(self.pin, pos)
        
        
#NOTA: as escs estão sendo controlada separadamente, dps tem q ver isso

if __name__ == "__main__":

    servo = Servo(pin=18)
    
    #escbaixo = ESC(pin1=12)
    #escbunda = ESC(pin1=13)
    
    #escbaixo.calibrate()
    #escbunda.calibrate()
    
    servo.control(pos=1350)

    #escbaixo.manual_control()
    #escbunda.manual_control()
    
    servo.manual_control()
    
    #escbaixo.halt()
    #escbunda.halt()
    
        

# max 'esquerda' 1650
# max 'direita'  1050
# meio 1350