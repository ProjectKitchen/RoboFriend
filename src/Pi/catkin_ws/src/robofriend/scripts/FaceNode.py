#!/usr/bin/env python3
import rospy
import time
import pygame
import pygame.gfxdraw
import constants
import os

# import ros messages and services
from robofriend.srv import SrvFaceDrawData, SrvFaceDrawDataResponse
from robofriend.srv import SrvFaceStatusData, SrvFaceStatusDataResponse
from robofriend.srv import SrvFaceScreenshotTimestamp

class FaceDataHandler():

    def __init__(self, time_req):
        pygame.init()
        self._time_req = time_req
        self._screen = pygame.display.set_mode((654, 380)) # pygame.display.set_mode((654, 380), pygame.FULLSCREEN)
        self._screen.fill((0, 0, 0))

        self._eyes_x = 0
        self._eyes_y = 0
        self._smile_percent = 60
        self._eye_step = 10
        self._screenshot_filename = 'screenshot.jpg'

        self._face_meth = {'get_screen_fn':   self._get_screenshot_filename, \
                           'set_smile':       self._set_smile, \
                           'increase':        self._increase_smile, \
                           'decrease':        self._decrease_smile, \
                           'set_eyes':        self._set_eyes, \
                           'up':         self._eyes_up, \
                           'down':       self._eyes_down, \
                           'left':       self._eyes_left, \
                           'right':      self._eyes_right
        }

        self.FIRST_ELEM = 0
        self.SECOND_ELEM = 1

        self._meth = ""
        self._param = []

        self._draw_face()

    def _sound_service_handler(self, request):
        rospy.logdebug("{%s} - Request regarding status from sound node received!",
            rospy.get_caller_id())

        resp = False
        is_sad = False

        if request.is_sad:
            resp = True
            is_sad = self._is_sad()
            rospy.logdebug("{%s} - Service response message: %s",
                    rospy.get_caller_id(), is_sad)
        else:
            resp = False
            is_sad = False

        rospy.logdebug("{%s} - Service response message: %s, %s",
            rospy.get_caller_id(), resp, is_sad)
        return SrvFaceStatusDataResponse(is_sad)


    def _service_handler(self, request):
        rospy.logdebug("{%s} - Face request received: %s, %s",
            rospy.get_caller_id(), str(request.action), str(request.param))

        fn = ""
        resp = False

        self._meth = request.action
        self._param = request.param

        if self._meth in constants.SET_SMILE or \
           self._meth in constants.SET_EYES:
                resp, fn = self._face_meth.\
                        get(self._meth, self._errorhandler)(self._param)
                rospy.logdebug("{%s} - Service response message: %s, %s",
                        rospy.get_caller_id(), resp, fn)
        elif self._meth in self._face_meth:
                resp, fn = self._face_meth.\
                        get(self._meth, self._errorhandler)()
                rospy.logdebug("{%s} - Service response message: %s, %s",
                    rospy.get_caller_id(), resp, fn)

        self._draw_face()
        return SrvFaceDrawDataResponse(resp, fn)

    def _errorhandler(self):
        rospy.logwarn("{%s} - Wrong methode received!",
            rospy.get_caller_id())
        return False, None

    def _get_screenshot_filename(self):
        return True, self._screenshot_filename

    def _set_smile(self, param):
        retVal = False
        smile_perc = 0

        if len(param) is not 1:
            retVal = False
        else:
            self._smile_percent = \
            self._restrict_range(param[self.FIRST_ELEM], -100, 100)
            retVal = True
        return retVal, None

    def _increase_smile(self):
        self._smile_percent = \
            self._restrict_range(self._smile_percent + 10, -100, 100)
        return True, None

    def _decrease_smile(self):
        self._smile_percent = \
                self._restrict_range(self._smile_percent - 10, -100, 100)
        return True, None

    def _set_eyes(self, param):
        # x_percent = self._param[self.FIRST_ELEM]
        # y_percent = self._param[self.SECOND_ELEM]
        x_percent = param[self.FIRST_ELEM]
        y_percent = param[self.SECOND_ELEM]
        rospy.logdebug("x_percent: %s, y_percent: %s",
            str(x_percent), str(y_percent))

        if x_percent < -100:
            x_percent = -100
        elif x_percent > 100:
            x_percent = 100

        if y_percent < -100:
            y_percent = 100
        elif y_percent > 100:
            y_percent = 100
        self._eyes_x = int(x_percent * (40.0 / 100))
        self._eyes_y = int(y_percent * (40.0 / 100))
        return True, None

    def _eyes_up(self):
        if self._eyes_y > -40:
            self._eyes_y -= self._eye_step
        return True, None

    def _eyes_down(self):
        if self._eyes_y < 40:
            self._eyes_y += self._eye_step
        return True, None

    def _eyes_left(self):
        if self._eyes_x > -40:
            self._eyes_x -= self._eye_step
        return True, None

    def _eyes_right(self):
        if self._eyes_x < 40:
            self._eyes_x += self._eye_step
        return True, None

    def _is_sad(self):
        return self._smile_percent < 0

    def _restrict_range(self, value, minValue, maxValue):
        return value if minValue <= value <= maxValue else (minValue if value < minValue else maxValue)

    def _draw_face(self):
        self._screen.fill((0, 0, 0))
        pygame.draw.circle(self._screen, (100, 250, 250), (163,100), 60) #lefteye
        pygame.draw.circle(self._screen, (100, 250, 250), (491,100), 60) #righteye
        pygame.draw.circle(self._screen, (10, 10, 10), (163 + self._eyes_x,100 + self._eyes_y), 20) #leftpupil
        pygame.draw.circle(self._screen, (10, 10, 10), (491 + self._eyes_x,100 + self._eyes_y), 20) #rightpupil
        self._draw_mouth()
        pygame.display.flip()
        path = os.path.dirname(os.path.realpath(__file__))
        pygame.image.save(self._screen, os.path.join(path, self._screenshot_filename))
        response = self._time_req(True)
        self._service_response_check(response.resp)


    def _draw_mouth(self):
        if self._smile_percent < 0:
            radius = self._map_range(abs(self._smile_percent), 0, 100, 0.3, 0.9)
            pygame.draw.arc(self._screen, (100, 200, 200), (57, 300, 540, 400), 1.57 - radius, 1.57 + radius, 20)  # smile
        else:
            radius = self._map_range(abs(self._smile_percent), 0, 100, 0.3, 1.3)
            pygame.draw.arc(self._screen, (100, 200, 200), (57, -30, 540, 400), 4.7 - radius, 4.7 + radius, 20)

    def _map_range(self, value, left_min, left_max, right_min, right_max):
        # Figure out how 'wide' each range is
        left_span = left_max - left_min
        right_span = right_max - right_min

        # Convert the left range into a 0-1 range (float)
        value_scaled = float(value - left_min) / float(left_span)

        # Convert the 0-1 range into a value in the right range.
        return right_min + (value_scaled * right_span)

    def _service_response_check(self, response = False):
        if response:
            rospy.logdebug("{%s} - Successfull response!",
                    rospy.get_caller_id())
        else:
            rospy.logwarn("{%s} - Erroneous response!",
                    rospy.get_caller_id())

def shutdown():
    rospy.loginfo("{%s} - stopping face node!",
        rospy.get_caller_id())
    pygame.quit()
    rospy.signal_shutdown("Stopping face node")

def Face():
    rospy.init_node("robofriend_face", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting face node!",
        rospy.get_caller_id())

    # create service to send to make time request
    rospy.wait_for_service("/robofriend/face_screen_timestamp")
    time_req = rospy.ServiceProxy('/robofriend/face_screen_timestamp', SrvFaceScreenshotTimestamp)

    face = FaceDataHandler(time_req)

    # create Service
    rospy.Service('/robofriend/face', SrvFaceDrawData, face._service_handler)

    # create service to communicate with sound Node
    rospy.Service('/robofriend/face_sound_stat', SrvFaceStatusData,
        face._sound_service_handler)

    rospy.spin()

if __name__ == '__main__':
    try:
        Face()
    except rospy.ROSInterruptException:
        pass
