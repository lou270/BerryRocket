from machine import I2C,RTC,Timer,Pin,PWM
import time
import math
import sensors, icm20948
import time
import struct

#### Declaration
# Declaration du bus de communication I2C
i2c=I2C(1,freq=400000)  # default assignment: scl=Pin(7), sda=Pin(6)

# Declaration des capteurs
lps22 = sensors.LPS22HB(i2c)
# icm20948 = sensors.ICM20948(i2c)
imu = icm20948.ICM20948(i2c_bus=i2c)

# Declaration du timer
tim = Timer()

# Declaration de l'horloge temps réel (RTC)
rtc = RTC()

# Declaration d'un PWM pour le buzzer
buzzer = PWM(Pin(18))

#### Constantes
accThreshold = 1

#### Fonctions

# Initialisation de la carte
def InitBoard():
    # Initialisation de la date/heure
    rtc.datetime((2020,1,1,0,0,0,0,0))

    # Initialisation du timer
    tim.init(freq=2,mode=Timer.PERIODIC, callback=Sampling)

    # Initialisation du buzzer à 500 Hz
    buzzer.freq(500)

# Initialisation de la carte
def Sampling(timer):
    global isSampling
    isSampling=True

# Main fonction
if __name__ == '__main__':
    
    isSampling=False
    isLaunched=False
    sample=0

    InitBoard()
    fileOut = open("Data.txt","w")
    
    while True:
        #buzzer.duty_u16(1000)
        if isSampling==True:
            
            sample=sample+1
            acc = [0,3,0]
            #buzzer.duty_u16(0)

            # DEBUG : Mesure temps execution
            t = time.ticks_us()
            
            # Acquisitions des capteurs

            mx, my, mz = imu.read_magnetometer_data()
            ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
            # acc = icm20948.accelerometer
            # gyro = icm20948.gyrometer
            # mag = icm20948.magnetometer
            pressure = lps22.pressure
            temp = lps22.temperature

            # DEBUG : Mesure temps execution
            delta = time.ticks_diff(time.ticks_us(), t)
            print('\rAcq time = %.3f ms\r'%(delta/1000))
            

            if ay > accThreshold:
                # buzzer.freq(1000)
                isLaunched=True
                fileOut.write(str(rtc.datetime())+'\n')
                print('Lift off')


            if isLaunched==True:
                # DEBUG : Mesure temps execution
                t = time.ticks_us()
                
                data = "{:d} {:.1f} {:.1f} {:.2f} {:.2f} {:.2f}\n".format(sample, pressure, temp, ax, ay, az)
                fileOut.write(data)
                # file.close()

                # DEBUG : Mesure temps execution
                delta = time.ticks_diff(time.ticks_us(), t)
                print('\rWrite time = %.3f ms\r'%(delta/1000))
            isSampling=False   
    #         print("\r\n /-------------------------------------------------------------/ \r\n")
            print(rtc.datetime())
            print('\rAcceleration:  X = {:.2f} , Y = {:.2f} , Z = {:.2f}\r'.format(ax, ay, az))  
            print('\rGyroscope:     X = {:.2f} , Y = {:.2f} , Z = {:.2f}\r'.format(gx, gy, gz))
            print('\rMagnetic:      X = {:.2f} , Y = {:.2f} , Z = {:.2f}\r'.format(mx, my, mz))
            print('\rPressure:      P = {:.2f}\r'.format(pressure))
            print('\rTemperature:   T = {:.2f}\r'.format(temp))
          
      





