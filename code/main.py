from machine import I2C,RTC,Timer,Pin,PWM
import time
import _thread
import math
import lps22, icm20948
import time
import struct

#### Declaration
# Declaration du bus de communication I2C
i2c = I2C(1,freq=400000)  # default assignment: scl=Pin(7), sda=Pin(6)

# Declaration des capteurs
lps22 = lps22.LPS22HB(i2c)
imu = icm20948.ICM20948(i2c_bus=i2c)

# Declaration du timer
timerAcq = Timer()
timerBuzzer = Timer()

# Declaration de l'horloge temps réel (RTC)
rtc = RTC()

# Declaration d'un PWM pour le buzzer
buzzer = PWM(Pin(18))

# Declaration de la pin du parachute
# parachute = Pin(XXX, Pin.OUT)

#### Variables
accThreshold = 1
freqBuzzer  = 500
tempsBuzzer = 0

#### Fonctions

# Initialisation de la carte
def InitBoard():
    # Initialisation de la date/heure
    rtc.datetime((2020,1,1,0,0,0,0,0))

    # Initialisation du timer d'acquisition
    timerAcq.init(freq=10, mode=Timer.PERIODIC, callback=Sampling)

# Initialisation de la carte
def Sampling(timer):
    global isSampling
    isSampling=True

# Initialisation musique
def InitMusic(buzzer):
    notes = (146.83,164.81,174.61,164.81,130.81,146.83,164.81,174.61,164.81,196.00,155.56,174.61,196.00,155.56,146.83,138.59,164.81)
    notes = tuple(x*2 for x in notes) # Augmente d'une octave la musique
    tempsNotes = (1.5,0.5,0.5,0.5,1,1.5,0.5,0.5,0.5,1,1.5,0.5,1,1,1,2,2)
    bpm = 75
    buzzer.duty_u16(0) # Set to 0%
    buzzer.duty_u16(32768) # Set to 50%
    for iNote in range(0, len(notes)):
        buzzer.freq(round(notes[iNote]))
        time.sleep((60.0/bpm)*tempsNotes[iNote])

    buzzer.duty_u16(0) # Set to 0%

def MgtBuzzer(timer):
    global freqBuzzer
    global buzzer
    buzzer.freq(freqBuzzer)
    buzzer.duty_u16(32768) # Set to 50%
    time.sleep(0.1) # Ring the buzzer for this time
    buzzer.duty_u16(0) # Set to 0%

def SetBuzzer(enable=True, freq=500, tps=5):
    global freqBuzzer
    global timerBuzzer
    freqBuzzer = freq
    timerBuzzer.deinit()
    if enable == True:
        timerBuzzer.init(freq=1.0/tps, mode=Timer.PERIODIC, callback=MgtBuzzer)

# Main fonction
if __name__ == '__main__':
    
    # Initialisation des variables
    isSampling = False
    isLaunched = False
    isFalling = False
    tempsMsDebut = time.ticks_ms()

    # InitMusic(buzzer)
    InitBoard()

    # Ouvre un fichier pour l'écriture des données
    fileOut = open("Data.txt","w")
    
    # Configure le buzzer pour faire un son specifique avant décollage
    SetBuzzer(freq=1000, tps=2)

    while True:
        if isSampling==True:
            # Acquisition du temps actuel
            tempsAcq = time.ticks_diff(time.ticks_ms(), tempsMsDebut)/1000 + 1

            # Acquisitions des capteurs
            mx, my, mz = imu.read_magnetometer_data()
            ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
            pressure = lps22.read_pressure()
            temp = lps22.read_temperature()            

            # Si l'acceleration dépasse le seuil (et que le décollage n'est pas arrive encore), il y a eu décollage
            if (ay > accThreshold) and (isLaunched == False):
                # Changement de status de l'indicateur de decollage
                isLaunched = True
                # Changement du son du buzzer
                SetBuzzer(freq=1500, tps=1)
                # Acquisition du temps du composant RTC
                tempsRtc = rtc.datetime()
                # Ecriture du temps actuel du decollage dans le fichier
                fileOut.write("Decollage: {:d}h{:d}m{:d}s{:.0f}\n".format(tempsRtc[4], tempsRtc[5], tempsRtc[6], (tempsAcq % 1)*100))
                fileOut.write("Temps (s) / Pression (mBar) / temperature (°C) / acc X (g/s^2) / acc Y (g/s^2) / acc Z (g/s^2)\n")
                # Affichage sur la console
                print('Decollage !')
            
            # Si le decollage est passé et que la chute libre n'est pas encore arrive
            if (isLaunched == True) and (isFalling == False):
                # Si l'acceleration est quasi nulle ou négative alors qu'on a décollé c'est qu'on retombe
                if (ay <= 0.2):
                    # Ouverture du parachute en activant la pin XXX
                    # parachute.on()
                    # Changement de status de chute libre
                    isFalling = True
                    # Changement du son du buzzer
                    SetBuzzer(freq=2000, tps=0.5)
                    # Acquisition du temps du composant RTC
                    tempsRtc = rtc.datetime()
                    # Ecriture du temps actuel du debut de la chute libre dans le fichier
                    fileOut.write("Chute libre: {:d}h{:d}m{:d}s{:.0f}\n".format(tempsRtc[4], tempsRtc[5], tempsRtc[6], (tempsAcq % 1)*100))
                    # Affichage sur la console
                    print('Chute libre !')

            # Si la fusee est en chute libre
            # if isFalling == True:
                # isLaunched = False

            # Si le decollage est passé, on enregistre les données
            if isLaunched == True:
                # Mise en forme des données
                data = "{:.2f} {:.1f} {:.1f} {:.2f} {:.2f} {:.2f}\n".format(tempsAcq, pressure, temp, ax, ay, az)
                # Ecriture sur le fichier
                fileOut.write(data)

            # Reinitialisation de l'indicateur pour le timer d'acquisition
            isSampling = False  

            # Affichage des resultats sur la console 
            tempsRtc = rtc.datetime()
            print("\nTime:        {:d}h{:d}m{:d}s / {:.2f}".format(tempsRtc[4], tempsRtc[5], tempsRtc[6], tempsAcq))
            print('Acceleration:  X = {:.2f} , Y = {:.2f} , Z = {:.2f}'.format(ax, ay, az))  
            print('Gyroscope:     X = {:.2f} , Y = {:.2f} , Z = {:.2f}'.format(gx, gy, gz))
            print('Magnetic:      X = {:.2f} , Y = {:.2f} , Z = {:.2f}'.format(mx, my, mz))
            print('Pressure:      P = {:.2f} mBar'.format(pressure))
            print('Temperature:   T = {:.2f} °C'.format(temp))


