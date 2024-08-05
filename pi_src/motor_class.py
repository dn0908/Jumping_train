import RPi.GPIO as IO
import time
import threading
from encoder_class import Encoder


class MotorController:
    def __init__(self, ix1, ix2, encoder):
        self.ix1 = ix1
        self.ix2 = ix2
        self.encoder = encoder

        IO.setup(self.ix1, IO.OUT)
        IO.setup(self.ix2, IO.OUT)

        self.pwm1 = IO.PWM(self.ix1, 100)  # Set PWM frequency to 100 Hz
        self.pwm2 = IO.PWM(self.ix2, 100)  # Set PWM frequency to 100 Hz

        self.pwm1.start(0)
        self.pwm2.start(0)

    def set_speed(self, speed):
        # speed should be a value between -100 and 100
        if speed > 0:
            # Forward
            self.pwm1.ChangeDutyCycle(speed)
            IO.output(self.ix2, IO.LOW)  # Fast decay
        elif speed < 0:
            # Reverse
            IO.output(self.ix1, IO.LOW)  # Fast decay
            self.pwm2.ChangeDutyCycle(-speed)
        else:
            # Stop
            IO.output(self.ix1, IO.LOW)
            IO.output(self.ix2, IO.LOW)
            self.pwm1.ChangeDutyCycle(0)
            self.pwm2.ChangeDutyCycle(0)

    def move_to_position(self, target_position, kp=1.0):
        while True:
            current_position = self.encoder.get_position()
            error = target_position - current_position
            if abs(error) < 1:  # Position tolerance
                self.set_speed(0)
                break
            speed = kp * error
            speed = max(min(speed, 100), -100)  # Limit speed to -100 to 100
            self.set_speed(speed)
            time.sleep(0.01)  # Small delay to avoid excessive CPU usage

    def move_at_velocity(self, target_velocity, kp=1.0):
        self.encoder.reset_position()
        last_time = time.time()
        while True:
            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time
            
            current_position = self.encoder.get_position()
            current_velocity = current_position / delta_time
            self.encoder.reset_position()
            
            error = target_velocity - current_velocity
            speed = kp * error
            speed = max(min(speed, 100), -100)  # Limit speed to -100 to 100
            self.set_speed(speed)
            time.sleep(0.01)  # Small delay to avoid excessive CPU usage


if __name__ == "__main__":
    # GPIO setup
    IO.setmode(IO.BCM)
    IO.setwarnings(False)

    encoder1 = Encoder(23, 24, "Motor1")
    encoder2 = Encoder(27, 22, "Motor2")

    motor1 = MotorController(12, 18, encoder1)  # IA1=12, IA2=18
    motor2 = MotorController(13, 19, encoder2)  # IB1=13, IB2=19

    try:
        motor1_thread = threading.Thread(target=motor1.move_to_position, args=(500,))
        motor2_thread = threading.Thread(target=motor2.move_at_velocity, args=(100,))
        
        motor1_thread.start()
        motor2_thread.start()

        motor1_thread.join()
        motor2_thread.join()

    except KeyboardInterrupt:
        print("Program stopped")
    finally:
        IO.cleanup()
