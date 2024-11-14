'''
    lego cable pinout:
        green   - 3v
        red     - GND
        blue    - encoderin1
        yellow  - encoderin2
        black/white - motor terminals  - connect to the ZK-5AD motor driver!        
          
'''
import motordriver
import machine
import utime

encoder1_pin = 14 
encoder2_pin = 12  
in1_pin = 4       
in2_pin = 0
wheel_size = 65    # wheel size (in mm)

robot = motordriver.motordriver(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size)


print('displacement = ', robot.gomm(100, 1000)) #distance, number of iterations
robot.gommp(200, 1000, 1, 2, 0) # distance, number of iterations, Kp, Ki, Kd
robot.motgo(0)

# try: godegrees(angle, times) and godegreesp(angle, times, kp, ki, kd)
