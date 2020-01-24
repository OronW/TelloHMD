from djitellopy.tello import Tello
import cv2
import pygame
import numpy as np
import time

import openvr
import sys
import socket




# Speed of the drone
S = 60
# Frames per second of the pygame window display
FPS = 25

yawFlag = False

class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and down.
    """




    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Oron - Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        openvr.init(openvr.VRApplication_Scene)
        global poses, posesArr, posesCurrX, posedPrevX, posesCurrY, posedPrevY, posesCurrZ, posedPrevZ, rotAngleCurr, rotAnglePrev
        poses = [] # will be populated with proper type after first call
        posesCurrX = [] # will be populated with proper type after first call
        posesPrevX = [] # will be populated with proper type after first call
        posesCurrY = []  # will be populated with proper type after first call
        posesPrevY = []  # will be populated with proper type after first call
        posesCurrZ = []  # will be populated with proper type after first call
        posesPrevZ = []  # will be populated with proper type after first call
        rotAngleCurr = []
        rotAnglePrev = []

        for i in range(1):
            poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
            hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
            posesPrevX = posesCurrX
            posesCurrX = np.transpose(hmd_pose.mDeviceToAbsoluteTracking[0][3])

            posesPrevY = posesCurrY
            posesCurrY = np.transpose(hmd_pose.mDeviceToAbsoluteTracking[2][3])

            posesPrevZ = posesCurrZ
            posesCurrZ = np.transpose(hmd_pose.mDeviceToAbsoluteTracking[1][3])

            rotAnglePrev = rotAngleCurr
            rotAngleCurr = ([hmd_pose.mDeviceToAbsoluteTracking[0][0], hmd_pose.mDeviceToAbsoluteTracking[0][1], hmd_pose.mDeviceToAbsoluteTracking[0][2]],
                            [hmd_pose.mDeviceToAbsoluteTracking[1][0], hmd_pose.mDeviceToAbsoluteTracking[1][1], hmd_pose.mDeviceToAbsoluteTracking[1][2]],
                            [hmd_pose.mDeviceToAbsoluteTracking[2][0], hmd_pose.mDeviceToAbsoluteTracking[2][1], hmd_pose.mDeviceToAbsoluteTracking[2][2]])

            print(posesCurrX)
            print(posesPrevX)

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 50)

    def run(self):


        def m2rotaxis(m):   # get the rotation out of the 3x3 matrix

            eps = 1e-5

            # if (
            #         abs(m[0, 1] - m[1, 0]) < eps
            #         and abs(m[0, 2] - m[2, 0]) < eps
            #         and abs(m[1, 2] - m[2, 1]) < eps
            # ):
            #     # Singularity encountered. Check if its 0 or 180 deg
            #     if (
            #             abs(m[0, 1] + m[1, 0]) < eps
            #             and abs(m[0, 2] + m[2, 0]) < eps
            #             and abs(m[1, 2] + m[2, 1]) < eps
            #             and abs(m[0, 0] + m[1, 1] + m[2, 2] - 3) < eps
            #     ):
            #         angle = 0
            #     else:
            #         angle = np.pi
            # else:
            # Angle always between 0 and pi
            # Sense of rotation is defined by axis orientation
            t = 0.5 * (np.trace(m) - 1)
            t = max(-1, t)
            t = min(1, t)
            angle = np.arccos(t)

            if angle < 1e-15:
                # Angle is 0
                return 0.0, np.array([1, 0, 0])
            elif angle < np.pi:
                # Angle is smaller than pi
                x = m[2, 1] - m[1, 2]
                y = m[0, 2] - m[2, 0]
                z = m[1, 0] - m[0, 1]
                axis = np.array(x, y, z)
                axis.normalize()
                return angle, axis
            else:
                # Angle is pi - special case!
                m00 = m[0, 0]
                m11 = m[1, 1]
                m22 = m[2, 2]
                if m00 > m11 and m00 > m22:
                    x = np.sqrt(m00 - m11 - m22 + 0.5)
                    y = m[0, 1] / (2 * x)
                    z = m[0, 2] / (2 * x)
                elif m11 > m00 and m11 > m22:
                    y = np.sqrt(m11 - m00 - m22 + 0.5)
                    x = m[0, 1] / (2 * y)
                    z = m[1, 2] / (2 * y)
                else:
                    z = np.sqrt(m22 - m00 - m11 + 0.5)
                    x = m[0, 2] / (2 * z)
                    y = m[1, 2] / (2 * z)
                axis = np.array(x, y, z)
                axis.normalize()
                return np.pi, axis


        # Checks if a matrix is a valid rotation matrix.
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6  # <- some epsilon

        # Calculates rotation matrix to euler angles
        def rotationMatrixToEulerAngles(R):

            assert (isRotationMatrix(R))

            sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

            singular = sy < 1e-6    #  check condition

            if not singular:
                x = np.arctan2(R[2, 1], R[2, 2])
                y = np.arctan2(-R[2, 0], sy)
                z = np.arctan2(R[1, 0], R[0, 0])
            else:
                x = np.arctan2(-R[1, 2], R[1, 1])
                y = np.arctan2(-R[2, 0], sy)
                z = 0

            return np.array([x, y, z])

        white = (255, 255, 255)
        green = (0, 255, 0)
        blue = (0, 0, 128)
        # create the display surface object
        # of specific dimension..e(X, Y).
        # display_surface = pygame.display.set_mode((X, Y)) <----
        #display_surface = pygame.display.set_mode((960, 720))
        # set the pygame window name
        # pygame.display.set_caption('Show Text')

        # create a font object.
        # 1st parameter is the font file
        # which is present in pygame.
        # 2nd parameter is size of the font
        font = pygame.font.Font('freesansbold.ttf', 32)

        # create a text suface object,
        # on which text is drawn on it.
        text = font.render('Test text', True, green, blue)

        # create a rectangular object for the
        # text surface object
        textRect = text.get_rect()

        # set the center of the rectangular object.
        textRect.center = (960 // 2, 720 // 2)

        # display_surface.fill(0, 0, 0)

        # copying the text surface object
        # to the display surface object
        # at the center coordinate.
        self.screen.blit(text, textRect)



        if not self.tello.connect():
            print("Tello not connected")
            return

        if not self.tello.set_speed(self.speed):
            print("Not set speed to lowest possible")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            return

        if not self.tello.streamon():
            print("Could not start video stream")
            return

        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                frame_read.stop()
                break

            self.screen.fill([0, 0, 0])
            frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)
            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1 / FPS)

            global poses, posesArr, posesCurrX, posedPrevX, posesCurrY, posedPrevY, posesCurrZ, posedPrevZ, rotAnglePrev, rotAngleCurr
            flag = True
            poses = []  # will be populated with proper type after first call
            #posesCurr = []  # will be populated with proper type after first call
            #posesPrev = []  # will be populated with proper type after first call

            for i in range(2):
                poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
                hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
                posesPrevX = posesCurrX
                posesCurrX = (hmd_pose.mDeviceToAbsoluteTracking[0][3])




                if (posesCurrX*100*1.01 < posesPrevX*100):
                    self.tello.send_rc_control(0, 60, 0, 0)     # == "forward"
                    flag = False

                elif (posesCurrX*100 > 1.01*posesPrevX*100):
                    self.tello.send_rc_control(0, -60, 0, 0)  # == "backward"
                    flag = False

                else:
                    self.tello.send_rc_control(0, 0, 0, 0)



            for i in range(2):
                poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
                hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]

                posesPrevZ = posesCurrZ
                posesCurrZ = (hmd_pose.mDeviceToAbsoluteTracking[1][3])

                if (posesCurrZ*100*1.0001 < posesPrevZ*100):
                    self.tello.send_rc_control(0, 0, -80, 0)     # == "up"
                    flag = True

                elif (posesCurrZ*100 > 1.0001*posesPrevZ*100):
                    self.tello.send_rc_control(0, 0, 80, 0)  # == "down"
                    flag = True
                else:
                    self.tello.send_rc_control(0, 0, 0, 0)
                    flag = True

            for i in range(2):
                poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
                hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]

                posesPrevY = posesCurrY
                posesCurrY = (hmd_pose.mDeviceToAbsoluteTracking[2][3])

                if (posesCurrY * 1000  * 1.0001 < posesPrevY * 1000):
                    self.tello.send_rc_control(60, 0, 0, 0)  # == "side"
                    flag = True

                elif (posesCurrY * 1000 > 1.0001 * posesPrevY * 1000):
                    self.tello.send_rc_control(-60, 0, 0, 0)  # == "side"
                    flag = True
                else:
                    self.tello.send_rc_control(0, 0, 0, 0)
                    flag = True

            for i in range(2):
                poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
                hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]

                rotAnglePrev = rotAngleCurr
                rotAngleCurr = ([hmd_pose.mDeviceToAbsoluteTracking[0][0], hmd_pose.mDeviceToAbsoluteTracking[0][1], hmd_pose.mDeviceToAbsoluteTracking[0][2]],
                                [hmd_pose.mDeviceToAbsoluteTracking[1][0], hmd_pose.mDeviceToAbsoluteTracking[1][1], hmd_pose.mDeviceToAbsoluteTracking[1][2]],
                               [hmd_pose.mDeviceToAbsoluteTracking[2][0], hmd_pose.mDeviceToAbsoluteTracking[2][1], hmd_pose.mDeviceToAbsoluteTracking[2][2]])

                prevAngle = rotationMatrixToEulerAngles(np.asanyarray(rotAnglePrev))
                currAngle = rotationMatrixToEulerAngles(np.asanyarray(rotAngleCurr))

                angle = currAngle[1] - prevAngle[1] # negative = right, positive = left

                def eulerToDegree(euler):
                    return ((euler) / (2 * np.pi)) * 360

                degAngle = int(eulerToDegree(angle))
                print("Angle: " + str(degAngle))
                turnAngle = degAngle/5 *5

                if (prevAngle[1]*1000 > currAngle[1]):
                    self.tello.send_rc_control(0, 0, 0, -60)
                elif (currAngle[1] < 0.001*prevAngle[1]):
                    self.tello.send_rc_control(0, 0, 0, 60)
                # else:
                #     self.tello.send_rc_control(0, 0, 0, 0)

                # if pygame.K_y:
                #     yawFlag = True
                #
                # if (yawFlag):
                #     degAngle = eulerToDegree(angle)
                #     print("Angle: " + str(degAngle))
                #
                #     if (degAngle >= 0.1):
                #         self.yaw_velocity = -100
                #     elif (degAngle < -0.1):
                #         self.yaw_velocity = 100
                #     else:
                #         self.yaw_velocity = 0



        # Call it always before finishing. To deallocate resources.
        self.tello.end()
        print("IT'S DONE")

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S
        elif key == pygame.K_y:
            yawFlag = True

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
            flag = True
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_control = False

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
