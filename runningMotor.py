# Init motors
from motor import Motor
from pwm import PulseWidthModulation #is designed to control the PWM for motors using the I2C communication protocol. 
from pin import Pin
from speed import Speed
import time 


# pwm converts a string channel identifier (like "P13") to an integer and handles I2C communication setup.
# pwm class is used to control the speed and the direction of a motor. By adjusting the frequency and pulse width of the PWM signal

left_front_reverse = False
right_front_reverse= False
left_rear_reverse = False
right_rear_reverse = False

left_front_speed = Speed(25)
right_front_speed = Speed(25)
left_rear_speed = Speed(25)
right_rear_speed = Speed(25)  

left_front = Motor(PulseWidthModulation("P13"), Pin("D4"), is_reversed=left_front_reverse) # motor 1
right_front = Motor(PulseWidthModulation("P12"), Pin("D5"), is_reversed=right_front_reverse) # motor 2
left_rear = Motor(PulseWidthModulation("P8"), Pin("D11"), is_reversed=left_rear_reverse) # motor 3
right_rear = Motor(PulseWidthModulation("P9"), Pin("D15"), is_reversed=right_rear_reverse) # motor 4

def start_speed_thread():
    left_front_speed.start()
    right_front_speed.start()
    left_rear_speed.start()
    right_rear_speed.start()
    
def forward(power):
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(power)
    right_rear.set_power(power)


def backward(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(-power)
    right_rear.set_power(-power)

def turn_right(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(power)
    right_rear.set_power(power)

def turn_left(power):
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(-power)
    right_rear.set_power(-power)

def stop():
    left_front.set_power(0)
    left_rear.set_power(0)
    right_front.set_power(0)
    right_rear.set_power(0) 

def turn_forward(power, bias=0):
    # Apply a bias to the power levels for turning
    right_power = max(min(power + bias, 100), -100)  # Ensure power level stays within [-100, 100]
    left_power = max(min(power - bias, 100), -100)  # Ensure power level stays within [-100, 100]

    # print(f"Left Power: {left_power}, Right Power: {right_power}")

    left_front.set_power(left_power)
    left_rear.set_power(left_power)
    right_front.set_power(right_power)
    right_rear.set_power(right_power)


def set_motor_power(motor, power):
    if motor == 1:
        left_front.set_power(power)
    elif motor == 2:
        right_front.set_power(power)
    elif motor == 3:
        left_rear.set_power(power)
    elif motor == 4:
        right_rear.set_power(power)

    
def speed_val():
    return (left_front_speed() + right_front_speed() + left_rear_speed() + right_rear_speed()) / 4.0
    
