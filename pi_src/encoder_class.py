import RPi.GPIO as IO
import time

class Encoder:
    def __init__(self, encPinA, encPinB, name):
        self.encPinA = encPinA
        self.encPinB = encPinB
        self.encoderPos = 0
        self.name = name
        
        IO.setup(self.encPinA, IO.IN, pull_up_down=IO.PUD_UP)
        IO.setup(self.encPinB, IO.IN, pull_up_down=IO.PUD_UP)
        
        IO.add_event_detect(self.encPinA, IO.BOTH, callback=self.encoderA)
        IO.add_event_detect(self.encPinB, IO.BOTH, callback=self.encoderB)

    def encoderA(self, channel):
        if IO.input(self.encPinA) == IO.input(self.encPinB):
            self.encoderPos += 1
        else:
            self.encoderPos -= 1
        print('Pin_A : %d, encoder %s : %d' % (channel, self.name, self.encoderPos))

    def encoderB(self, channel):
        if IO.input(self.encPinA) == IO.input(self.encPinB):
            self.encoderPos -= 1
        else:
            self.encoderPos += 1
        print('Pin_B : %d, encoder %s : %d' % (channel, self.name, self.encoderPos))

    

if __name__ == "__main__":
    encoder = Encoder()

    IO.setmode(IO.BCM)
    IO.setwarnings(False)

    motor1 = encoder(23, 24, "Motor1")
    motor2 = encoder(27, 22, "Motor2")

    try:
        while True:
            time.sleep(0.0001)
    except KeyboardInterrupt:
        print("Program stopped")
    finally:
        IO.cleanup()
