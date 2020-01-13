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

flag = False

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

########################

        openvr.init(openvr.VRApplication_Scene)
        global poses, posesArr, posesCurr, posedPrev
        poses = [] # will be populated with proper type after first call
        posesCurr = [] # will be populated with proper type after first call
        posesPrev = [] # will be populated with proper type after first call
        for i in range(1):
            poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
            hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
            posesPrev = posesCurr
            posesCurr = np.transpose(hmd_pose.mDeviceToAbsoluteTracking[0][3])
            print(posesCurr)
            print(posesPrev)
            #print(hmd_pose.mDeviceToAbsoluteTracking)
            # print(hmd_pose.mDeviceToAbsoluteTracking)
            # sys.stdout.flush()
            # time.sleep(0.2)
        # openvr.shutdown()

        ############################


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



        ############
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
        ############


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

            global poses, posesArr, posesCurr, posedPrev
            poses = []  # will be populated with proper type after first call
            #posesCurr = []  # will be populated with proper type after first call
            #posesPrev = []  # will be populated with proper type after first call

            for i in range(1):
                poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
                hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
                posesPrev = posesCurr
                posesCurr = np.transpose(hmd_pose.mDeviceToAbsoluteTracking[0][3])
                print('curr: ' + str(posesCurr))
                print('prev: ' + str(posesPrev))
                # print(hmd_pose.mDeviceToAbsoluteTracking)
                # print('index is' + str(i))
                # sys.stdout.flush()
                time.sleep(0.2)

                if (posesCurr < posesPrev):
                    self.tello.send_rc_control(0, 40, 0, 0)     # == "forward"

                elif (posesCurr > posesPrev):
                    self.tello.send_rc_control(0, -40, 0, 0)  # == "backward"

                else:
                    self.for_back_velocity = 0


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
