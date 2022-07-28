#coding:utf-8
import time
import math
import board
from threading import Thread
import adafruit_mpu6050
import logging
import data

class IMU(Thread):
    def __init__(self):
        super(IMU, self).__init__()
        self.Kp = 100
        self.Ki = 0.002
        self.halfT = 0.001

        self.q0 = 1
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

        self.exInt = 0
        self.eyInt = 0
        self.ezInt = 0
        self.pitch = 0
        self.roll =0
        self.yaw = 0

        self.i2c = board.I2C()
        self.isActive = True
        self.name = "MPU6050"
        self.sensor = adafruit_mpu6050.MPU6050(self.i2c)
        #self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)
        #self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)

        self.kalman_filter_AX =  Kalman_filter(0.001,0.1)
        self.kalman_filter_AY =  Kalman_filter(0.001,0.1)
        self.kalman_filter_AZ =  Kalman_filter(0.001,0.1)

        self.kalman_filter_GX =  Kalman_filter(0.001,0.1)
        self.kalman_filter_GY =  Kalman_filter(0.001,0.1)
        self.kalman_filter_GZ =  Kalman_filter(0.001,0.1)

        self.Error_value_accel_data,self.Error_value_gyro_data=self.average_filter()

    def average_filter(self):
        sum_accel_x=0
        sum_accel_y=0
        sum_accel_z=0

        sum_gyro_x=0
        sum_gyro_y=0
        sum_gyro_z=0
        for i in range(100):
            accel_data = list(self.sensor.acceleration)
            gyro_data = list(self.sensor.gyro)

            sum_accel_x+=accel_data[0]
            sum_accel_y+=accel_data[1]
            sum_accel_z+=accel_data[2]

            sum_gyro_x+=gyro_data[0]
            sum_gyro_y+=gyro_data[1]
            sum_gyro_z+=gyro_data[2]

        sum_accel_x/=100
        sum_accel_y/=100
        sum_accel_z/=100

        sum_gyro_x/=100
        sum_gyro_y/=100
        sum_gyro_z/=100

        accel_data[0]=sum_accel_x
        accel_data[1]=sum_accel_y
        accel_data[2]=sum_accel_z-9.8

        gyro_data[0]=sum_gyro_x
        gyro_data[1]=sum_gyro_y
        gyro_data[2]=sum_gyro_z

        return accel_data,gyro_data

    def imuUpdate(self):
        accel_data = list(self.sensor.acceleration)
        gyro_data = list(self.sensor.gyro)
        ax=self.kalman_filter_AX.kalman(accel_data[0]-self.Error_value_accel_data[0])
        ay=self.kalman_filter_AY.kalman(accel_data[1]-self.Error_value_accel_data[1])
        az=self.kalman_filter_AZ.kalman(accel_data[2]-self.Error_value_accel_data[2])
        gx=self.kalman_filter_GX.kalman(gyro_data[0]-self.Error_value_gyro_data[0])
        gy=self.kalman_filter_GY.kalman(gyro_data[1]-self.Error_value_gyro_data[1])
        gz=self.kalman_filter_GZ.kalman(gyro_data[2]-self.Error_value_gyro_data[2])

        norm = math.sqrt(ax*ax+ay*ay+az*az)

        ax = ax/norm
        ay = ay/norm
        az = az/norm

        vx = 2*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3

        ex = (ay*vz - az*vy)
        ey = (az*vx - ax*vz)
        ez = (ax*vy - ay*vx)

        self.exInt += ex*self.Ki
        self.eyInt += ey*self.Ki
        self.ezInt += ez*self.Ki

        gx += self.Kp*ex + self.exInt
        gy += self.Kp*ey + self.eyInt
        gz += self.Kp*ez + self.ezInt

        self.q0 += (-self.q1*gx - self.q2*gy - self.q3*gz)*self.halfT
        self.q1 += (self.q0*gx + self.q2*gz - self.q3*gy)*self.halfT
        self.q2 += (self.q0*gy - self.q1*gz + self.q3*gx)*self.halfT
        self.q3 += (self.q0*gz + self.q1*gy - self.q2*gx)*self.halfT

        norm = math.sqrt(self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3)
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm

        pitch = math.asin(-2*self.q1*self.q3+2*self.q0*self.q2)*57.3
        roll = math.atan2(2*self.q2*self.q3+2*self.q0*self.q1,-2*self.q1*self.q1-2*self.q2*self.q2+1)*57.3
        yaw = math.atan2(2*(self.q1*self.q2 + self.q0*self.q3),self.q0*self.q0+self.q1*self.q1-self.q2*self.q2-self.q3*self.q3)*57.3
        self.pitch = pitch
        self.roll =roll
        self.yaw = yaw
        return self.pitch,self.roll,self.yaw

    def terminate(self):
        self.isActive = False

    def run(self):
        time.sleep(2)
        self.Error_value_accel_data,self.Error_value_gyro_data=self.average_filter()
        time.sleep(1)
        while self.isActive:
            r,p,y=self.imuUpdate()
            data.imu["r"] = r
            data.imu["p"] = p
            data.imu["y"] = y
            #data.RT_data["m_x"] = m_x
            #data.RT_data["m_y"] = m_y

class Kalman_filter:
    def __init__(self,Q,R):
        self.Q = Q
        self.R = R
        self.P_k_k1 = 1
        self.Kg = 0
        self.P_k1_k1 = 1
        self.x_k_k1 = 0
        self.ADC_OLD_Value = 0
        self.Z_k = 0
        self.kalman_adc_old=0

    def kalman(self,ADC_Value):
        self.Z_k = ADC_Value
        if (abs(self.kalman_adc_old-ADC_Value)>=60):
            self.x_k1_k1= ADC_Value*0.400 + self.kalman_adc_old*0.600
        else:
            self.x_k1_k1 = self.kalman_adc_old;
        self.x_k_k1 = self.x_k1_k1
        self.P_k_k1 = self.P_k1_k1 + self.Q
        self.Kg = self.P_k_k1/(self.P_k_k1 + self.R)
        kalman_adc = self.x_k_k1 + self.Kg * (self.Z_k - self.kalman_adc_old)
        self.P_k1_k1 = (1 - self.Kg)*self.P_k_k1
        self.P_k_k1 = self.P_k1_k1
        self.kalman_adc_old = kalman_adc
        return kalman_adc

# Main program logic follows:
if __name__ == '__main__':
    imu = IMU()
    time.sleep(2)
    imu.Error_value_accel_data,imu.Error_value_gyro_data=imu.average_filter()
    time.sleep(1)
    while True:
        r,p,y=imu.imuUpdate()
        print("r:%.0f, p: %.0f, y: %.0f"%(r, p, y))
# import time
# import board
# import adafruit_mpu6050
# try:
#     from typing import Tuple
#     from busio import I2C
# except ImportError:
#     pass
#
#
# class IMU(adafruit_mpu6050.MPU6050):
#     @property
#     def quaternion(self) -> Tuple[float, float, float, float]:
#         """The quaternion value calculated from"""
#         raw_gyro_data = self._raw_gyro_data
#         raw_accel_data = self._raw_accel_data
#         return (1,1,1,1)
#
#     @property
#     def absOrientation(self) -> Tuple[float, float, float, float]:
#         """The quaternion value calculated from"""
#         raw_temperature = self._raw_temp_data
#         raw_data = self._raw_gyro_data
#         return ()
#
#
# if __name__ == '__main__':
#     i2c = board.I2C()
#     mpu = IMU(i2c)
#     #mpu = adafruit_mpu6050.MPU6050(i2c)
#     while True:
#         print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(mpu.acceleration))
#         print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s"%(mpu.gyro))
#         print("Temperature: %.2f C"%mpu.temperature)
#         print("quaternion: %.2f, %.2f, %.2f, %.2f "%mpu.quaternion)
#         print("")
#         time.sleep(1)
