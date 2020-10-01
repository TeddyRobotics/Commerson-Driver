from commerson_driver import CommersonDriver
import time

if __name__ == '__main__':
    driver = CommersonDriver() #assign object to driver
    motor_id = 254 #define motor ID (254 is any motor)
    driver.raw_move(motor_id, 150, 1000) #pass position and time in ms
    #time.sleep(1)
    #print("Move motor")
    #angle = 150 #define angle to move to
    #speed = 0.5 #speed
    #driver.move_motor(motor_id, angle, speed) #move motor function
    print("Done")
