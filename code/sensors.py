from machine import I2C
import time
import math
#from micropython import const

#===========================================================================
#=                   Barometric sensor LPS22 Class                         =
#===========================================================================
#
# define LPS22 Register
#
LPS_ID                =  0xB1
#Register 
LPS_INT_CFG           =  0x0B        #Interrupt register
LPS_THS_P_L           =  0x0C        #Pressure threshold registers 
LPS_THS_P_H           =  0x0D        
LPS_WHO_AM_I          =  0x0F        #Who am I        
LPS_CTRL_REG1         =  0x10        #Control registers
LPS_CTRL_REG2         =  0x11
LPS_CTRL_REG3         =  0x12
LPS_FIFO_CTRL         =  0x14        #FIFO configuration register 
LPS_REF_P_XL          =  0x15        #Reference pressure registers
LPS_REF_P_L           =  0x16
LPS_REF_P_H           =  0x17
LPS_RPDS_L            =  0x18        #Pressure offset registers
LPS_RPDS_H            =  0x19        
LPS_RES_CONF          =  0x1A        #Resolution register
LPS_INT_SOURCE        =  0x25        #Interrupt register
LPS_FIFO_STATUS       =  0x26        #FIFO status register
LPS_STATUS            =  0x27        #Status register
LPS_PRESS_OUT_XL      =  0x28        #Pressure output registers
LPS_PRESS_OUT_L       =  0x29
LPS_PRESS_OUT_H       =  0x2A
LPS_TEMP_OUT_L        =  0x2B        #Temperature output registers
LPS_TEMP_OUT_H        =  0x2C
LPS_RES               =  0x33        #Filter reset register

LPS22HB_I2C_ADDRESS    = 0x5C

#
# define LPS22 Class
#
class LPS22Exception(Exception):
    pass

class InvalidMode(LPS22Exception):
    pass

class DeviceIsNotReady(LPS22Exception):
    pass

class LPS22HB(object):
    
    def __init__(self,i2c_bus,address=LPS22HB_I2C_ADDRESS):
        self._address = address
        self._bus = i2c_bus
        self._reset()                         #Wait for reset to complete
        self._write_byte(LPS_CTRL_REG1 ,0x02)        #Low-pass filter disabled , output registers not updated until MSB and LSB have been read , Enable Block Data Update , Set Output Data Rate to 0 
    
    def _reset(self):
        Buf=self._read_u16(LPS_CTRL_REG2)
        Buf|=0x04                                         
        self._write_byte(LPS_CTRL_REG2,Buf)          #Soft internal RESET, waiting for completion
        while Buf:
            Buf=self._read_u16(LPS_CTRL_REG2)
            Buf&=0x04
        
    def _read_byte(self,cmd):
        rec=self._bus.readfrom_mem(int(self._address),int(cmd),1)
        return rec[0]
    
    def _read_u16(self,cmd):
        LSB = self._bus.readfrom_mem(int(self._address),int(cmd),1)
        MSB = self._bus.readfrom_mem(int(self._address),int(cmd)+1,1)
        return (MSB[0] << 8) + LSB[0]
    
    def _write_byte(self,cmd,val):
        self._bus.writeto_mem(int(self._address),int(cmd),bytes([int(val)]))
      
    @property
    def pressure(self):
        """The current pressure measurement in hPa"""
        Buf=self._read_u16(LPS_CTRL_REG2)
        Buf|=0x01                                    #ONE_SHOT mode, Dataset is required
        self._write_byte(LPS_CTRL_REG2,Buf)
        
        while True:
            if (self._read_byte(LPS_STATUS)&0x01)==0x01:
                u8Buf=[0,0,0]
                u8Buf[0]=self._read_byte(LPS_PRESS_OUT_XL)
                u8Buf[1]=self._read_byte(LPS_PRESS_OUT_L)
                u8Buf[2]=self._read_byte(LPS_PRESS_OUT_H)
                return ((u8Buf[2]<<16)+(u8Buf[1]<<8)+u8Buf[0])/4096.0
                break
    
    @property
    def temperature(self):
        """The current temperature measurement in degrees Celsius"""
        Buf=self._read_u16(LPS_CTRL_REG2)
        Buf|=0x01                                    #ONE_SHOT mode, Dataset is required
        self._write_byte(LPS_CTRL_REG2,Buf)
        
        while True:
            if (self._read_byte(LPS_STATUS)&0x02)==0x02:
                u8Buf=[0,0]
                u8Buf[0]=self._read_byte(LPS_TEMP_OUT_L)
                u8Buf[1]=self._read_byte(LPS_TEMP_OUT_H)
                return ((u8Buf[1]<<8)+u8Buf[0])/100.0
                break
        
#===========================================================================
#=                       IMU 9DOF ICM-20948 Class                          =
#===========================================================================
#
# define Devices I2C address
I2C_ADD_ICM20948                     = 0x68
I2C_ADD_ICM20948_AK09916             = 0x0C
I2C_ADD_ICM20948_AK09916_READ        = 0x80
I2C_ADD_ICM20948_AK09916_WRITE       = 0x00
LPS22HB_I2C_ADDRESS                  = 0x5C

# define ICM-20948 Register
# user bank 0 register
REG_ADD_WIA                          = 0x00
REG_VAL_WIA                          = 0xEA
REG_ADD_USER_CTRL                    = 0x03
REG_VAL_BIT_DMP_EN                   = 0x80
REG_VAL_BIT_FIFO_EN                  = 0x40
REG_VAL_BIT_I2C_MST_EN               = 0x20
REG_VAL_BIT_I2C_IF_DIS               = 0x10
REG_VAL_BIT_DMP_RST                  = 0x08
REG_VAL_BIT_DIAMOND_DMP_RST          = 0x04
REG_VAL_ALL_RGE_RESET                = 0x80
REG_VAL_RUN_MODE                     = 0x01 # Non low-power mode
REG_ADD_LP_CONFIG                    = 0x05
REG_ADD_PWR_MGMT_1                   = 0x06
REG_ADD_PWR_MGMT_2                   = 0x07
REG_ADD_ACCEL_XOUT_H                 = 0x2D
REG_ADD_ACCEL_XOUT_L                 = 0x2E
REG_ADD_ACCEL_YOUT_H                 = 0x2F
REG_ADD_ACCEL_YOUT_L                 = 0x30
REG_ADD_ACCEL_ZOUT_H                 = 0x31
REG_ADD_ACCEL_ZOUT_L                 = 0x32
REG_ADD_GYRO_XOUT_H                  = 0x33
REG_ADD_GYRO_XOUT_L                  = 0x34
REG_ADD_GYRO_YOUT_H                  = 0x35
REG_ADD_GYRO_YOUT_L                  = 0x36
REG_ADD_GYRO_ZOUT_H                  = 0x37
REG_ADD_GYRO_ZOUT_L                  = 0x38
REG_ADD_EXT_SENS_DATA_00             = 0x3B
REG_ADD_REG_BANK_SEL                 = 0x7F
REG_VAL_REG_BANK_0                   = 0x00
REG_VAL_REG_BANK_1                   = 0x10
REG_VAL_REG_BANK_2                   = 0x20
REG_VAL_REG_BANK_3                   = 0x30

# user bank 1 register
# user bank 2 register
REG_ADD_GYRO_SMPLRT_DIV              = 0x00
REG_ADD_GYRO_CONFIG_1                = 0x01
REG_VAL_BIT_GYRO_DLPCFG_2            = 0x10  # bit[5:3]
REG_VAL_BIT_GYRO_DLPCFG_4            = 0x20  # bit[5:3]
REG_VAL_BIT_GYRO_DLPCFG_6            = 0x30  # bit[5:3]
REG_VAL_BIT_GYRO_FS_250DPS           = 0x00  # bit[2:1]
REG_VAL_BIT_GYRO_FS_500DPS           = 0x02  # bit[2:1]
REG_VAL_BIT_GYRO_FS_1000DPS          = 0x04  # bit[2:1]
REG_VAL_BIT_GYRO_FS_2000DPS          = 0x06  # bit[2:1]
REG_VAL_BIT_GYRO_DLPF                = 0x01  # bit[0]
REG_ADD_ACCEL_SMPLRT_DIV_2           = 0x11
REG_ADD_ACCEL_CONFIG                 = 0x14
REG_VAL_BIT_ACCEL_DLPCFG_2           = 0x10  # bit[5:3]
REG_VAL_BIT_ACCEL_DLPCFG_4           = 0x20  # bit[5:3]
REG_VAL_BIT_ACCEL_DLPCFG_6           = 0x30  # bit[5:3]
REG_VAL_BIT_ACCEL_FS_2g              = 0x00  # bit[2:1]
REG_VAL_BIT_ACCEL_FS_4g              = 0x02  # bit[2:1]
REG_VAL_BIT_ACCEL_FS_8g              = 0x04  # bit[2:1]
REG_VAL_BIT_ACCEL_FS_16g             = 0x06  # bit[2:1]
REG_VAL_BIT_ACCEL_DLPF               = 0x01  # bit[0]

# user bank 3 register
REG_ADD_I2C_SLV0_ADDR                = 0x03
REG_ADD_I2C_SLV0_REG                 = 0x04
REG_ADD_I2C_SLV0_CTRL                = 0x05
REG_VAL_BIT_SLV0_EN                  = 0x80
REG_VAL_BIT_MASK_LEN                 = 0x07
REG_ADD_I2C_SLV0_DO                  = 0x06
REG_ADD_I2C_SLV1_ADDR                = 0x07
REG_ADD_I2C_SLV1_REG                 = 0x08
REG_ADD_I2C_SLV1_CTRL                = 0x09
REG_ADD_I2C_SLV1_DO                  = 0x0A

# define ICM-20948 MAG Register
REG_ADD_MAG_WIA1                     = 0x00
REG_VAL_MAG_WIA1                     = 0x48
REG_ADD_MAG_WIA2                     = 0x01
REG_VAL_MAG_WIA2                     = 0x09
REG_ADD_MAG_ST2                      = 0x10
REG_ADD_MAG_DATA                     = 0x11
REG_ADD_MAG_CNTL2                    = 0x31
REG_VAL_MAG_MODE_PD                  = 0x00
REG_VAL_MAG_MODE_SM                  = 0x01
REG_VAL_MAG_MODE_10HZ                = 0x02
REG_VAL_MAG_MODE_20HZ                = 0x04
REG_VAL_MAG_MODE_50HZ                = 0x05
REG_VAL_MAG_MODE_100HZ               = 0x08
REG_VAL_MAG_MODE_ST                  = 0x10

MAG_DATA_LEN                         =6

VAL_ACCEL_FS_2g                      =16384
VAL_ACCEL_FS_4g                      =8192
VAL_ACCEL_FS_8g                      =4096
VAL_ACCEL_FS_16g                     =2048
VAL_GYRO_FS_250dps                   =131
VAL_GYRO_FS_500dps                   =65.5
VAL_GYRO_FS_1000dps                  =32.8
VAL_GYRO_FS_2000dps                  =16.4

#
# define ICM20948 Class
#
class ICM20948Exception(Exception):
    pass

class DeviceNotConnected(ICM20948Exception):
    pass

class ICM20948(object):
    
    def __init__(self,i2c_bus,acc_fs=16,gyro_fs=500):
        
        self._address = I2C_ADD_ICM20948
        self._bus = i2c_bus
        time.sleep(0.5)
        if REG_VAL_WIA != self._read_byte(REG_ADD_WIA):   # check IMU device
            raise DeviceNotConnected('IMU not detected')
        time.sleep(0.5)
        
        # Gyro and Accelero initialization
        
        if acc_fs not in (2, 4, 8, 16):
            raise InvalidMode('Accelerometer full scale (g unit) should be one of 2,4,8 or 16')
        else:
            if acc_fs==2:
                self.VAL_ACCEL_FS=VAL_ACCEL_FS_2g
                self.REG_VAL_BIT_ACCEL_FS=REG_VAL_BIT_ACCEL_FS_2g
            if acc_fs==4:
                self.VAL_ACCEL_FS=VAL_ACCEL_FS_4g
                self.REG_VAL_BIT_ACCEL_FS=REG_VAL_BIT_ACCEL_FS_4g
            if acc_fs==8:
                self.VAL_ACCEL_FS=VAL_ACCEL_FS_8g
                self.REG_VAL_BIT_ACCEL_FS=REG_VAL_BIT_ACCEL_FS_8g
            if acc_fs==16:
                self.VAL_ACCEL_FS=VAL_ACCEL_FS_16g
                self.REG_VAL_BIT_ACCEL_FS=REG_VAL_BIT_ACCEL_FS_16g
 
        if gyro_fs not in (250, 500, 1000, 2000):
            raise InvalidMode('Gyrometer full scale (dps unit) should be one of 250,500,1000 or 2000')
        else:
            if gyro_fs==250:
                self.VAL_GYRO_FS=VAL_GYRO_FS_250dps
                self.REG_VAL_BIT_GYRO_FS=REG_VAL_BIT_GYRO_FS_250DPS
            if gyro_fs==500:
                self.VAL_GYRO_FS=VAL_GYRO_FS_500dps
                self.REG_VAL_BIT_GYRO_FS=REG_VAL_BIT_GYRO_FS_500DPS
            if gyro_fs==1000:
                self.VAL_GYRO_FS=VAL_GYRO_FS_1000dps
                self.REG_VAL_BIT_GYRO_FS=REG_VAL_BIT_GYRO_FS_1000DPS
            if gyro_fs==2000:
                self.VAL_GYRO_FS=VAL_GYRO_FS_2000dps
                self.REG_VAL_BIT_GYRO_FS=REG_VAL_BIT_GYRO_FS_2000DPS

        self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_0)    # user bank 0 register 
        self._write_byte( REG_ADD_PWR_MGMT_1 , REG_VAL_ALL_RGE_RESET) # Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear.
        time.sleep(0.1)
        self._write_byte( REG_ADD_PWR_MGMT_1 , REG_VAL_RUN_MODE)  #
        self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_2) # bank 2
        self._write_byte( REG_ADD_GYRO_SMPLRT_DIV , 0x07) # Gyro acquisition sample rate
        self._write_byte( REG_ADD_GYRO_CONFIG_1 , REG_VAL_BIT_GYRO_DLPCFG_6 | self.REG_VAL_BIT_GYRO_FS | REG_VAL_BIT_GYRO_DLPF)
        self._write_byte( REG_ADD_ACCEL_SMPLRT_DIV_2 ,  0x07) # Accelero acquisition sample rate
        self._write_byte( REG_ADD_ACCEL_CONFIG , REG_VAL_BIT_ACCEL_DLPCFG_6 | self.REG_VAL_BIT_ACCEL_FS | REG_VAL_BIT_ACCEL_DLPF)
        self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_0)  #user bank 0 register
        time.sleep(0.1)
        self.icm20948WriteSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_20HZ)

    def icm20948ReadSecondary(self,u8I2CAddr,u8RegAddr,u8Len):
        pu8data=[0,0,0,0,0,0,0,0]
        u8Temp=0
        self._write_byte( REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3) #swtich bank3
        self._write_byte( REG_ADD_I2C_SLV0_ADDR, u8I2CAddr)
        self._write_byte( REG_ADD_I2C_SLV0_REG,  u8RegAddr)
        self._write_byte( REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len)

        self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0

        u8Temp = self._read_byte(REG_ADD_USER_CTRL)
        u8Temp |= REG_VAL_BIT_I2C_MST_EN
        self._write_byte( REG_ADD_USER_CTRL, u8Temp)
        #time.sleep(0.01)
        u8Temp &= ~REG_VAL_BIT_I2C_MST_EN
        self._write_byte( REG_ADD_USER_CTRL, u8Temp)

        for i in range(0,u8Len):
          pu8data[i]= self._read_byte( REG_ADD_EXT_SENS_DATA_00+i)

        self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3) #swtich bank3

        u8Temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
        u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN))
        self._write_byte( REG_ADD_I2C_SLV0_CTRL,  u8Temp)

        self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0
        return pu8data
    
    def icm20948WriteSecondary(self,u8I2CAddr,u8RegAddr,u8data):
        u8Temp=0
        self._write_byte( REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3) #swtich bank3
        self._write_byte( REG_ADD_I2C_SLV1_ADDR, u8I2CAddr)
        self._write_byte( REG_ADD_I2C_SLV1_REG,  u8RegAddr)
        self._write_byte( REG_ADD_I2C_SLV1_DO,   u8data)
        self._write_byte( REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1)

        self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0

        u8Temp = self._read_byte(REG_ADD_USER_CTRL)
        u8Temp |= REG_VAL_BIT_I2C_MST_EN
        self._write_byte( REG_ADD_USER_CTRL, u8Temp)
        #time.sleep(0.01)
        u8Temp &= ~REG_VAL_BIT_I2C_MST_EN
        self._write_byte( REG_ADD_USER_CTRL, u8Temp)

        self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3) #swtich bank3

        u8Temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
        u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN))
        self._write_byte( REG_ADD_I2C_SLV0_CTRL,  u8Temp)

        self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0
        
    def _read_byte(self,cmd):
        rec=self._bus.readfrom_mem(int(self._address),int(cmd),1)
        return rec[0]

    def _read_block(self, reg, length=1):
        rec=self._bus.readfrom_mem(int(self._address),int(reg),length)
        return rec

    def _read_u16(self,cmd):
        LSB = self._bus.readfrom_mem(int(self._address),int(cmd),1)
        MSB = self._bus.readfrom_mem(int(self._address),int(cmd)+1,1)
        return (MSB[0] << 8) + LSB[0]

    def _write_byte(self,cmd,val):
        self._bus.writeto_mem(int(self._address),int(cmd),bytes([int(val)]))
        #time.sleep(0.0001)
           
    @property
    def accelerometer(self):
        Accel=[0,0,0]
        self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_0)
        data =self._read_block(REG_ADD_ACCEL_XOUT_H, 6)
        self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_2)
        Accel[0] = ((data[0]<<8)|data[1])
        Accel[1] = ((data[2]<<8)|data[3])
        Accel[2] = ((data[4]<<8)|data[5])
        if Accel[0]>=32767:             #Solve the problem that Python shift will not overflow
          Accel[0]=Accel[0]-65535
        elif Accel[0]<=-32767:
          Accel[0]=Accel[0]+65535
        if Accel[1]>=32767:
          Accel[1]=Accel[1]-65535
        elif Accel[1]<=-32767:
          Accel[1]=Accel[1]+65535
        if Accel[2]>=32767:
          Accel[2]=Accel[2]-65535
        elif Accel[2]<=-32767:
          Accel[2]=Accel[2]+65535
        Accel[0] = Accel[0]/self.VAL_ACCEL_FS
        Accel[1] = Accel[1]/self.VAL_ACCEL_FS
        Accel[2] = Accel[2]/self.VAL_ACCEL_FS
        return Accel

    @property
    def gyrometer(self):
        Gyro=[0,0,0]
        self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_0)
        data =self._read_block(REG_ADD_GYRO_XOUT_H, 6)
        self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_2)
        Gyro[0] = ((data[0]<<8)|data[1])
        Gyro[1] = ((data[2]<<8)|data[3])
        Gyro[2] = ((data[4]<<8)|data[5])
        if Gyro[0]>=32767:
          Gyro[0]=Gyro[0]-65535
        elif Gyro[0]<=-32767:
          Gyro[0]=Gyro[0]+65535
        if Gyro[1]>=32767:
          Gyro[1]=Gyro[1]-65535
        elif Gyro[1]<=-32767:
          Gyro[1]=Gyro[1]+65535
        if Gyro[2]>=32767:
          Gyro[2]=Gyro[2]-65535
        elif Gyro[2]<=-32767:
          Gyro[2]=Gyro[2]+65535
        Gyro[0] = Gyro[0]/VAL_GYRO_FS_1000dps
        Gyro[1] = Gyro[1]/VAL_GYRO_FS_1000dps
        Gyro[2] = Gyro[2]/VAL_GYRO_FS_1000dps
        return Gyro

    @property
    def magnetometer(self):
        nbMean = 8
        Mag=[0,0,0]
        # U8tempX=[0,0,0,0,0,0,0,0,0]
        # U8tempY=[0,0,0,0,0,0,0,0,0]
        # U8tempZ=[0,0,0,0,0,0,0,0,0]
        counter=20
        while(counter>0):
          #time.sleep(0.01)
          pu8data=self.icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ , REG_ADD_MAG_ST2, 1)
          if ((pu8data[0] & 0x01)!= 0):
            break
          counter-=1
        if counter!=0:
          for i in range(0,nbMean):
            pu8data = self.icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ , REG_ADD_MAG_DATA , MAG_DATA_LEN)
            # U8tempX[i] = (pu8data[1]<<8)|pu8data[0]
            # U8tempY[i] = (pu8data[3]<<8)|pu8data[2]
            # U8tempZ[i] = (pu8data[5]<<8)|pu8data[4]

            Mag[0] = Mag[0] + ((pu8data[1]<<8)|pu8data[0])
            Mag[1] = Mag[1] + ((pu8data[3]<<8)|pu8data[2])
            Mag[2] = Mag[2] + ((pu8data[5]<<8)|pu8data[4])

          Mag[0] = Mag[0]/nbMean
          Mag[1] = Mag[1]/nbMean
          Mag[2] = Mag[2]/nbMean

          # Mag[0]=(U8tempX[0]+U8tempX[1]+U8tempX[2]+U8tempX[3]+U8tempX[4]+U8tempX[5]+U8tempX[6]+U8tempX[7])/8
          # Mag[1]=-(U8tempY[0]+U8tempY[1]+U8tempY[2]+U8tempY[3]+U8tempY[4]+U8tempY[5]+U8tempY[6]+U8tempY[7])/8
          # Mag[2]=-(U8tempZ[0]+U8tempZ[1]+U8tempZ[2]+U8tempZ[3]+U8tempZ[4]+U8tempZ[5]+U8tempZ[6]+U8tempZ[7])/8
        
        if Mag[0]>=32767:            #Solve the problem that Python shift will not overflow
          Mag[0]=Mag[0]-65535
        elif Mag[0]<=-32767:
          Mag[0]=Mag[0]+65535
        if Mag[1]>=32767:
          Mag[1]=Mag[1]-65535
        elif Mag[1]<=-32767:
          Mag[1]=Mag[1]+65535
        if Mag[2]>=32767:
          Mag[2]=Mag[2]-65535
        elif Mag[2]<=-32767:
          Mag[2]=Mag[2]+65535

        Mag[0] = Mag[0]*0.15
        Mag[1] = Mag[1]*0.15
        Mag[2] = Mag[2]*0.15
        return Mag
