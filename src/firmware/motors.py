#!/usr/bin/env python3

import time
import pigpio


#NOTA: n to sentando pino nenhum como output, sla se da merda, mas fica ligado

class ESC:
    MIN_WIDTH = 650
    MAX_WIDTH = 2400

    def __init__(self, pin:int)-> None:
        self.conn = pigpio.pi()
        self.pin = pin
        
        self.conn.set_mode(self.pin, pigpio.OUTPUT)  #pigpio.PWM(12)  
        #pi.set_servo_pulsewidth(ESC1_GPIO, 0)
        self.conn.set_PWM_frequency(self.pin, 50)



    def calibrate(self)-> None:
        print('"menor" velocidade')
        self.conn.set_servo_pulsewidth(self.pin, self.MIN_WIDTH)
        time.sleep(5)
        
        print('"maior" velocidade')
        self.conn.set_servo_pulsewidth(self.pin, self.MAX_WIDTH)
        time.sleep(5)
        
        print("calibrado pai")
        self.conn.set_servo_pulsewidth(self.pin, 0)
        
    def arm(self)-> None:
        print("armando")
        self.conn.set_servo_pulsewidth(self.pin, self.MIN_WIDTH)
        time.sleep(2)
        
        print("armado")
        
    def halt(self)-> None:
        print("parando")
        self.conn.set_servo_pulsewidth(self.pin, self.MIN_WIDTH)
        
        print("ta safe")
        self.conn.set_servo_pulsewidth(self.pin, 0)
        
        print("desligando GPIO.")
        self.conn.stop()
        
        print("já era")
    
    def control(self)-> None:
        a = input()
        veloc = self.conn.get_servo_pulsewidth(self.pin)
        
        while a != 'x':
            if a == 'q':
                veloc += 10
                self.conn.set_servo_pulsewidth(self.pin, veloc)
            
            if a == 'a':
                veloc -= 10
                self.conn.set_servo_pulsewidth(self.pin, veloc)
            
            if a == 'e':
                veloc += 100
                self.conn.set_servo_pulsewidth(self.pin, veloc)
            
            if a == 'd':
                veloc -= 100
                self.conn.set_servo_pulsewidth(self.pin, veloc)
            
            a = input()
    
    def test(self) ->None:
        self.conn.set_servo_pulsewidth(self.pin, self.MIN_WIDTH)
        
        step = 100
        print("acelerando")
        for veloc in range(self.MIN_WIDTH, self.MAX_WIDTH, step):
            self.conn.set_servo_pulsewidth(self.pin, veloc)
            time.sleep(1)
        
        time.sleep(2)  
        print("parando")
        for veloc in range(self.MAX_WIDTH, self.MIN_WIDTH, -step):
            self.conn.set_servo_pulsewidth(self.pin, veloc)
                

class Servo:
    MIN_WIDTH = 600 #menor angulo em teoria
    MAX_WIDTH = 2400 #maior angulo em teoria, cuidado ppra n quebrar essa porra
     
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
        
    def control(self)-> None:
        width = int(input('qual o "angulo" puto:'))
        
        #tem q converter pra graus, mas fdc
        while width != "x":
            while width < self.MIN_WIDTH or width > self.MAX_WIDTH:
                width = input("ta fazendo merda: ")    
            self.conn.set_servo_pulsewidth(self.pin, width)
            width = int(input('angulo novo: '))
        
        
#NOTA: as escs estão sendo controlada separadamente, dps tem q ver isso

if __name__ == "__main__":

    servo = Servo(pin=18)
    
    esc1 = ESC(pin=12)
    esc2 = ESC(pin=13)
    while True:
        inp = input("vai mexer onde?")
    
        if inp == "esc":
            print("vou callibrar as paradas e testar")
            esc1.calibrate()
            esc2.calibrate()
            
            esc1.arm()
            esc2.arm()
            
            esc1.test()
            esc2.test()
            
            sOUn = input("quer brincar?")
            if sOUn == 's':
                esc1.control()
            
        elif inp == "servo":
            print("vou callibrar a parada e testar")
            servo.test()
            
            sOUn = input("quer brincar?")
            if sOUn == 's':
                servo.control()
            
        elif inp == "x":
            break
