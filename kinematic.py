import numpy as np
from math import *
from threading import Thread
from adafruit_servokit import ServoKit
from gpiozero import Button
import logging
from IMU import IMU
import time
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    # handlers=[
    #     logging.FileHandler("debug.log"),
    #     logging.StreamHandler()
    # ]
)


#leika walk(x,y,z,a)

class Movement(Thread):
    """docstring for Movement."""

    def __init__(self):
        super(Movement, self).__init__()
        self.kinematic = Kinematic()
        self.imu = IMU()
        self.servos = ServoKit(channels=16, frequency=50, reference_clock_speed=25000000)
        self.state = "walk"
        self.target_state = "walk" #idle, Stand || IMU, Walk
        self.isActive = True

        self.button = Button(4, hold_time=2)
        self.button.when_pressed = self.toogleMode
        #button.when_held = shutdown

        self.__min_servo = np.array([0 ,  0,  0,  0, 40, 10, 0,  0, 20, 90, 40,10])
        self.__max_servo = np.array([90,150,130,180,150,155,90,150,140,180,180,155])
        self.pose([0,120,90,180,60,90,0,120,90,180,60,90])

        # Private walk variables
        self.__step_incr = np.array([15,0,-7,7]) #np.array([0,0,0,0])SideHop  #np.array([0,0,0,0])HOP
        self.__step_dir = np.array([-1,-1,-1,-1]) #np.array([-1,5,-1,5])sideHop #np.array([-1,-1,5,5])HOP
        self.__xStepConst = 0
        self.__yStepConst = 0
        self.__current_x = 0
        self.__current_y = 0               # Walking y
        self.__step_theta = 0           # Walking angle
        self.__step_height = 0          # Ground to body height
        self.__step_arc_height = 50     # Step arc height
        self.__step_accel_factor = 10
        self.__step_res = 15

        time.sleep(2)
        self.imu.Error_value_accel_data,self.imu.Error_value_gyro_data=self.imu.average_filter()
        time.sleep(1)

        self.loop()

    def toogleMode(self):
        if self.state == "stand":
            self.state = "walk"
        elif self.state == "walk":
            self.state = "rest"
        elif self.state == "rest":
            self.state = "stand"

    def loop(self):
        while self.isActive:
            if self.state == "walk":
                self.walk_curve(50)
            elif (self.__current_x != 0 or self.__current_y != 0):
                self.walk_curve(0,0)
            elif self.state == "stand":
                self.stand()
            elif self.state == "rest":
                self.pose([0,120,90,180,60,90,0,120,90,180,60,90])

    def curve(self, x, y, type=2):
        return abs(x)/15*y-y
        #return -(y/100)*x**2+y

    def walk_curve(self, target_x = 50, target_y = 0, theta = 0, height = 0, arcHeight = 70):

        for i in range(4):
            if self.__step_incr[i] <= -self.__step_res: # Current step incr is less than -Step resolution
                self.__step_dir[i] = 3 # Set the step increment size to 5

                self.__current_x += self.__step_accel_factor if target_x > self.__current_x else -self.__step_accel_factor # Make the current step size bigger
                self.__current_y += self.__step_accel_factor if target_y > self.__current_y else -self.__step_accel_factor # Make the current step size bigger
                #self.__current_x = self.clamp(self.__current_x)#max(min(self.__current_x,abs(target_x)), -abs(target_x)) # Clamp step size
                self.__xStepConst = self.__current_x/self.__step_res
                #self.__current_y = #max(min(self.__current_y,abs(target_y)), -abs(target_y))
                self.__yStepConst = self.__current_y/self.__step_res

            elif self.__step_incr[i] >= self.__step_res: # Current step incr is bigger or = Step resolution
                self.__step_dir[i] = -1
            self.__step_incr[i] += self.__step_dir[i] # get next increment



        heightOffset = height-self.curve(self.__step_incr[0], arcHeight) if self.__step_dir[0]> 0 else 0
        IK = self.kinematic.legIK(-61+self.__step_incr[0]*self.__yStepConst,-130+heightOffset,-60+self.__step_incr[0]*self.__xStepConst)
        LF = (degrees(pi/2 - IK[0]), degrees(pi/3 - IK[1]), degrees(pi - IK[2]))

        heightOffset = height-self.curve(self.__step_incr[1], arcHeight) if self.__step_dir[1]> 0 else 0
        IK = self.kinematic.legIK(-61+self.__step_incr[1]*self.__yStepConst*-1,-130+heightOffset,-60+self.__step_incr[1]*self.__xStepConst)
        RF = (degrees(pi/2 + IK[0]), degrees(2 * pi/3 + IK[1]), degrees(IK[2]))

        heightOffset = height-self.curve(self.__step_incr[2], arcHeight) if self.__step_dir[2]> 0 else 0
        IK = self.kinematic.legIK(-61+self.__step_incr[2]*self.__yStepConst,-130+heightOffset,-60+self.__step_incr[2]*self.__xStepConst)
        LB = (degrees(pi/2 + (IK[0])), degrees(pi/3 - IK[1]), degrees(pi - IK[2]))

        heightOffset = height-self.curve(self.__step_incr[3], arcHeight) if self.__step_dir[3]> 0 else 0
        IK = self.kinematic.legIK(-61+self.__step_incr[3]*self.__yStepConst*-1,-130+heightOffset,-60+self.__step_incr[3]*self.__xStepConst)
        RB = (degrees(pi/2 - IK[0]), degrees(2 * pi/3  + IK[1]), degrees(IK[2]))

        self.pose([LF[2]-30,LF[1]-50,LF[0],RF[2]+30,RF[1]+50,RF[0],LB[2]-30,LB[1]-50,LB[0],RB[2]+30,RB[1]+50,RB[0]])
        time.sleep(0.015)
    # def walk(self):
    #     #start_time = time.time()#perf_counter()
    #     first_height = self.__step_height
    #     second_height = self.__step_height
    #     if self.__step_incr_size < 0:
    #         first_height  += (abs(self.__step_incr)-self.__step_width_x)/self.__step_width_x*self.__step_arc_height
    #     else:
    #         second_height += (abs(self.__step_incr)-self.__step_width_x)/self.__step_width_x*self.__step_arc_height
    #
    #     IK = self.kinematic.legIK(-61,-130-second_height,self.__step_incr)
    #     LF = (degrees(pi/2 - IK[0]), degrees(pi/3 - IK[1]), degrees(pi - IK[2]))
    #
    #     IK = self.kinematic.legIK(-61,-130-height,self.__step_incr*-1)
    #     RF = (degrees(pi/2 + IK[0]), degrees(2 * pi/3 + IK[1]), degrees(IK[2]))
    #
    #     IK = self.kinematic.legIK(-61,-130-height,self.__step_incr*-1)
    #     LB = (degrees(pi/2 + (IK[0])), degrees(pi/3 - IK[1]), degrees(pi - IK[2]))
    #
    #     IK = self.kinematic.legIK(-61,-130-second_height,self.__step_incr)
    #     RB = (degrees(pi/2 - IK[0]), degrees(2 * pi/3  + IK[1]), degrees(IK[2]))
    #
    #     #end_time = time.time()#perf_counter()
    #     #print("Processing took: {} s".format(end_time-start_time))
    #     self.pose([LF[2]-30,LF[1]-50,LF[0],RF[2]+30,RF[1]+50,RF[0],LB[2]-30,LB[1]-50,LB[0],RB[2]+30,RB[1]+50,RB[0]])
    #
    #     if self.__step_incr >= self.__step_width_x or self.__step_incr <= -self.__step_width_x:
    #         self.__step_incr_size *= -1
    #     self.__step_incr+=self.__step_incr_size
    #
    #     if self.__num_step > 2:
    #         self.__step_width_x += self.__step_accel_factor
    #         self.__step_width_x = max(min(self.__step_width_x,self.xStepTarget), 0.0001)
    #         mulplyer = -1 if self.__step_incr_size > 0 else 1
    #         self.__step_incr_size = self.__step_width_x/30*mulplyer
    #         self.__step_incr = self.__step_width_x*mulplyer
    #         self.__num_step = 0

    def stand(self): # IMU
        r,p,y=self.imu.imuUpdate()
        (LF, RF, LB, RB) = self.kinematic.calcFeet(p/40,0,r/40,0,0,0) #y/40
        self.pose([LF[2]-30,LF[1]-50,LF[0],RF[2]+30,RF[1]+50,RF[0],LB[2]-30,LB[1]-50,LB[0],RB[2]+30,RB[1]+50,RB[0]])
        time.sleep(0.001)

    def pose(self, angles):
        self.angles = angles
        for i in range(len(angles)):
            # map(0,270,0,180)
            # if i==0 or i==6:
            #     self.angles[i]*=self.bigServeConst
            # if i==3 or i==9:
            #     self.angles[i]/=self.bigServeConst
            self.servos.servo[i].angle = max(min(self.angles[i], self.__max_servo[i]), self.__min_servo[i])

class Kinematic(object):
    """docstring for Kinematic."""

    def __init__(self):
        super(Kinematic, self).__init__()
        self.L = 207.5
        self.W = 78
        self.l1=60.5
        self.l2=10
        self.l3=100.7
        self.l4=118.5
        self.Lp=np.array([[80,-130,100,1],[80,-130,-100,1],[-130,-130,100,1],[-130,-130,-100,1]])
        self.Ix=np.array([[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    def legIK(self,x,y,z):
        """
        x/y/z=Position of the Foot in Leg-Space

        F=Length of shoulder-point to target-point on x/y only
        G=length we need to reach to the point on x/y
        H=3-Dimensional length we need to reach
        """

        F=sqrt(x**2+y**2-self.l1**2)
        G=F-self.l2
        H=sqrt(G**2+z**2)
        D=(H**2-self.l3**2-self.l4**2)/(2*self.l3*self.l4)

        theta1=-atan2(y,x)-atan2(F,-self.l1)
        theta3=acos(D)
        theta2=atan2(z,G)-atan2(self.l4*sin(theta3),self.l3+self.l4*cos(theta3))

        return(theta1,theta2,theta3)

    def bodyIK(self, omega, phi, psi, xm, ym, zm):

        """
        Calculate the four Transformation-Matrices for our Legs
        Rx=X-Axis Rotation Matrix
        Ry=Y-Axis Rotation Matrix
        Rz=Z-Axis Rotation Matrix
        Rxyz=All Axis Rotation Matrix
        T=Translation Matrix
        Tm=Transformation Matrix
        Trb,Trf,Tlb,Tlf=final Matrix for RightBack,RightFront,LeftBack and LeftFront
        """

        Rx = np.array([
            [1, 0, 0, 0],
            [0, np.cos(omega), -np.sin(omega), 0],
            [0,np.sin(omega),np.cos(omega),0],
            [0,0,0,1]])

        Ry = np.array([
            [np.cos(phi),0, np.sin(phi), 0],
            [0, 1, 0, 0],
            [-np.sin(phi),0, np.cos(phi),0],
            [0,0,0,1]])

        Rz = np.array([
            [np.cos(psi),-np.sin(psi), 0,0],
            [np.sin(psi),np.cos(psi),0,0],
            [0,0,1,0],
            [0,0,0,1]])

        Rxyz=Rx.dot(Ry).dot(Rz)


        T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
        Tm = T+Rxyz

        Trb = Tm.dot(np.array([
            [np.cos(pi/2),0,np.sin(pi/2),-self.L/2],
            [0,1,0,0],
            [-np.sin(pi/2),0,np.cos(pi/2),-self.W/2],
            [0,0,0,1]]))

        Trf = Tm.dot(np.array([
            [np.cos(pi/2),0,np.sin(pi/2),self.L/2],
            [0,1,0,0],
            [-np.sin(pi/2),0,np.cos(pi/2),-self.W/2],
            [0,0,0,1]]))

        Tlf = Tm.dot(np.array([
            [np.cos(pi/2),0,np.sin(pi/2),self.L/2],
            [0,1,0,0],
            [-np.sin(pi/2),0,np.cos(pi/2),self.W/2],
            [0,0,0,1]]))

        Tlb = Tm.dot(np.array([
            [np.cos(pi/2),0,np.sin(pi/2),-self.L/2],
            [0,1,0,0],
            [-np.sin(pi/2),0,np.cos(pi/2),self.W/2],
            [0,0,0,1]]))

        return (Tlf,Trf,Tlb,Trb,Tm)

    def calcFeet(self, omega, phi, psi, xm, ym, zm):
        try:

            (Tlf,Trf,Tlb,Trb,Tm) = self.bodyIK(omega, phi, psi, xm, ym, zm)

            Q=np.linalg.inv(Tlf).dot(self.Lp[0])
            #p=[Tlf.dot(x) for x in calcLegPoints(legIK(Q[0],Q[1],Q[2]))]
            #drawLegPoints(p)

            IK = self.legIK(Q[0],Q[1],Q[2])
            LF = (degrees(pi/2 - IK[0]), degrees(pi/3 - IK[1]), degrees(pi - IK[2]))

            Q=self.Ix.dot(np.linalg.inv(Trf)).dot(self.Lp[1])

            IK = self.legIK(Q[0],Q[1],Q[2])
            RF = (degrees(pi/2 + IK[0]), degrees(2 * pi/3 + IK[1]), degrees(IK[2]))

            Q=np.linalg.inv(Tlb).dot(self.Lp[2])

            IK = self.legIK(Q[0],Q[1],Q[2])
            LB = (degrees(pi/2 + (IK[0])), degrees(pi/3 - IK[1]), degrees(pi - IK[2]))

            Q=self.Ix.dot(np.linalg.inv(Trb)).dot(self.Lp[3])

            IK = self.legIK(Q[0],Q[1],Q[2])
            RB = (degrees(pi/2 - IK[0]), degrees(2 * pi/3  + IK[1]), degrees(IK[2]))
            return (LF, RF, LB, RB)
        except Exception as e:
            logging.error(e)
            return [[90]*3]*4

if __name__ == '__main__':
    movement = Movement()
    movement.start()
