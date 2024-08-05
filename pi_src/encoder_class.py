import RPi.GPIO as IO
import time


class Encoder:

    def __init__(self):
        
        # PINS
        self.left_A  = 23
        self.left_B  = 24
        self.right_A = 27
        self.right_B = 22
        
        # INIT GPIOs
        IO.setmode(IO.BCM)
        IO.setwarnings(False)
        IO.setup(self.left_A, IO.IN, pull_up_down=IO.PUD_UP)
        IO.setup(self.left_B, IO.IN, pull_up_down=IO.PUD_UP)
        IO.setup(self.right_A, IO.IN, pull_up_down=IO.PUD_UP)
        IO.setup(self.right_B, IO.IN, pull_up_down=IO.PUD_UP)


        global left_encoderPos
        global right_encoderPos

encoderPos = 0

def encoderA(channel):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos += 1
    else:
        encoderPos -= 1
        print('Pin_A : %d, encoder : %d' %(channel, encoderPos))

def encoderB(channel):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos -= 1
    else:
        encoderPos += 1
        print('Pin_B : %d, encoder : %d' %(channel, encoderPos))

IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)

while True:
    time.sleep(0.001)