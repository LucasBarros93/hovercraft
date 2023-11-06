#!/usr/bin/env python3

import time
import pigpio


#NOTA: n to sentando pino nenhum como output, sla se da merda, mas fica ligado

class ESC:
    MIN_WIDTH = 650
    MAX_WIDTH = 2400

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
        print('"menor" velocidade')
        self.pwm(self.MIN_WIDTH)
        time.sleep(5)
        
        print('"maior" velocidade')
        self.pwm(self.MAX_WIDTH)
        time.sleep(5)
        
        print("calibrado pai")
        self.pwm(self.MIN_WIDTH)
        
    def arm(self)-> None:
        print("armando")
        self.pwm(self.MIN_WIDTH)
        time.sleep(2)
        
        print("armado")
        
    def halt(self)-> None:
        print("parando")
        self.pwm(self.MIN_WIDTH)
        
        print("ta safe")
        self.pwm(0)
        
        print("desligando GPIO.")
        self.conn.stop()
        
        print("já era")
    
    def control(self)-> None:
        a = input()
        veloc = self.conn.get_servo_pulsewidth(self.pin1)
        
        while a != 'x':
            if a == 'q':
                veloc += 10
                self.pwm(veloc)
            
            if a == 'a':
                veloc -= 10
                self.pwm(veloc)
            
            if a == 'e':
                veloc += 100
                self.pwm(veloc)
            
            if a == 'd':
                veloc -= 100
                self.pwm(veloc)
            
            print(veloc)
            a = input()
    
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
    
    escs = ESC(pin1=12, pin2=13)
    while True:
        inp = input("vai mexer onde?")
    
        if inp == "esc":
            #print("vou callibrar as paradas e testar")
            #esc1.calibrate()
            
            escs.arm()
            
            #esc1.test()
            #esc2.test()
            
            sOUn = input("quer brincar?")
            if sOUn == 's':
                escs.control()
            
        elif inp == "servo":
            #print("vou callibrar a parada e testar")
            #servo.test()
            
            sOUn = input("quer brincar?")
            if sOUn == 's':
                servo.control()
            
        elif inp == "x":
            escs.halt()
            break
