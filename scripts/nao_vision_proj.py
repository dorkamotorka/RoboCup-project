from controller import Robot, Motion
import math
import numpy as np
from math import cos, sin

PICKUP_DISTANCE = 0.4

class Nao (Robot):

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)
        self.cameraTop.recognitionEnable(4 * self.timeStep)
        self.cameraBottom.recognitionEnable(4 * self.timeStep)

    def __init__(self):
        Robot.__init__(self)

        # initialize stuff
        self.findAndEnableDevices()
        self.load_motions()

    def load_motions(self):
        self.turn_left_60 = Motion("motions/TurnLeft60.motion");
        self.turn_right_60 = Motion('motions/TurnRight60.motion')
        self.forward = Motion("motions/Forwards50.motion");
        self.backward = Motion("motions/Backwards.motion");
        self.side_step_left = Motion("motions/SideStepLeft.motion");
        self.side_step_right = Motion("motions/SideStepRight.motion");

    def transform_bottom_cam(self, x, y, z):
        '''red := x, green := y, blue := z'''
        # Got by inverse quaternion of the camera orientation such that coordinate frame had x forward and y to the left of the robot(z up) + converted to radians
        rot_x = -0.8779005
        rot_y = 0
        rot_z = 1.5707963

        a11 = cos(rot_z)*cos(rot_y)
        a12 = cos(rot_z)*sin(rot_y)*sin(rot_x) - sin(rot_z)*cos(rot_x)
        a13 = cos(rot_z)*sin(rot_y)*cos(rot_x) + sin(rot_z)*sin(rot_x)
        a21 = sin(rot_z)*sin(rot_y)
        a22 = sin(rot_z)*sin(rot_y)*sin(rot_x) + cos(rot_z)*cos(rot_x)
        a23 = sin(rot_z)*sin(rot_y)*cos(rot_x) - cos(rot_z)*cos(rot_x)
        a31 = -sin(rot_y)
        a32 = cos(rot_y)*sin(rot_x)
        a33 = cos(rot_y)*cos(rot_x)
        rot_matrix = np.matrix([[a11, a12, a13],
                                [a21, a22, a23],
                                [a31, a32, a33]])
        out = np.dot([x, y, z], rot_matrix)
        out = np.squeeze(np.asarray(out))

        return out[0], out[1], out[2]

    def transform_top_cam(self, x, y, z):
        '''red := x, green := y, blue := z'''
        # Got by inverse quaternion of the camera orientation such that coordinate frame had x forward and y to the left of the robot(z up) + converted to radians
        rot_x = -1.5498539
        rot_y = -0.000002
        rot_z = 1.5707983

        a11 = cos(rot_z)*cos(rot_y)
        a12 = cos(rot_z)*sin(rot_y)*sin(rot_x) - sin(rot_z)*cos(rot_x)
        a13 = cos(rot_z)*sin(rot_y)*cos(rot_x) + sin(rot_z)*sin(rot_x)
        a21 = sin(rot_z)*sin(rot_y)
        a22 = sin(rot_z)*sin(rot_y)*sin(rot_x) + cos(rot_z)*cos(rot_x)
        a23 = sin(rot_z)*sin(rot_y)*cos(rot_x) - cos(rot_z)*cos(rot_x)
        a31 = -sin(rot_y)
        a32 = cos(rot_y)*sin(rot_x)
        a33 = cos(rot_y)*cos(rot_x)
        rot_matrix = np.matrix([[a11, a12, a13],
                                [a21, a22, a23],
                                [a31, a32, a33]])
        out = np.dot([x, y, z], rot_matrix)
        out = np.squeeze(np.asarray(out))

        return out[0], out[1], out[2]

    def move(self, motion):
        # Start motion and execute it until done
        motion.play()
        while (not motion.isOver()):
            robot.step(self.timeStep)

    def run(self):
        while True:
            # Rotate until you detect an object
            self.move(self.turn_right_60)
            # Try to detect object with Top camera (more precise if the object is more distant)
            objects = self.cameraTop.getRecognitionObjects()
            if len(objects) == 0:
                # If Top camera doesnt detect anything try to detect with Bottom camera
                objects = self.cameraBottom.getRecognitionObjects()

            if objects:
                distance = 100 # value doesnt matter

                while distance > PICKUP_DISTANCE:
                    # Update objects all the time since we are moving
                    objects = self.cameraTop.getRecognitionObjects()
                    if len(objects) == 0:
                        # If Top camera doesnt detect anything try to detect with Bottom camera
                        objects = self.cameraBottom.getRecognitionObjects()
                        print(objects)
                    if objects:
                        first_object = objects[0]
                        x = first_object.get_position()[0]
                        y = first_object.get_position()[1]
                        z = first_object.get_position()[2]
                        x, y, z = self.transform_bottom_cam(x, y, z)
                        #print('x: ', x)
                        #print('y: ', y)
                        #print('z: ', z)

                        distance = math.sqrt(math.pow(x,2)+math.pow(y,2))
                        print('distance: ', distance)
                        angle = math.atan2(x,y)
                        print('angle: ', angle)

                        # First turn towards the box
                        # TODO: Need smaller angle turns!

                        # Go to the box
                        # TODO: Need smaller steps forward!
                        self.move(self.forward)
                    else:
                        print('Lost sight ob object. Back to searching...')
                        distance = PICKUP_DISTANCE + 1  # Break out of loop 
                print('READY TO PICKUP THE BOX')    
                break
            else:
                print('No Object detected')

# create the Robot instance and run main loop
robot = Nao()
robot.run()
