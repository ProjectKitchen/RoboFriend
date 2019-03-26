#!/usr/bin/env python3
import rospy
import os
import pygame
import random

from robofriend.srv import SrvSoundData, SrvSoundDataResponse
from robofriend.srv import SrvFaceStatusData

class SoundDataHandler():

    def __init__(self, face_req):
        pygame.init()
        if pygame.mixer and not pygame.mixer.get_init():
            rospy.logwarn("{%s} - No sound!",
                rospy.get_caller_id())
            pygame.mixer = None

        self._lastPlayFile = ""
        self._face_req = face_req
        self._path = os.path.dirname(os.path.abspath(__file__)) + "/"

    def _service_game_handler(self, request):
        rospy.logdebug("{%s} - Request from game node received!",
            rospy.get_caller_id())
        self._playsound(request.game_data)
        return SrvSoundGameDataResponse(True)

    def _service_handler(self, request):
        rospy.logdebug("{%s} - Request received!",
            rospy.get_caller_id())

        if request.get_random is True:
            resp_filedir = self._get_filenames('data/random/')
            return SrvSoundDataResponse(resp_filedir)
        elif request.action in "random":
            if request.command is not "":
                self._play_sound_file("data/random/" + str(request.command))
            else:
                self._play_random()
        elif request.action in "mood":
            if request.command is not "":
                self._play_sound_file("data/mood/" + str(request.command))
            else:
                mood_list = ["data/", "mood"]
                self._playsound(mood_list)
        elif request.action in "play":
            self._playsound(request.game_list)
        else:
            rospy.logwarn("{%s} - Wrong methode recieved!",
                rospy.get_caller_id())

    def _get_filenames(self, given_path):
        return (os.listdir(self._path + given_path))

    def _play_random(self):
        path = 'data/random/'
        filenames = self._get_filenames(path)
        playFile = path + random.choice(filenames)
        self._lastPlayFile = playFile
        self._play_sound_file(playFile)

    def _play_sound_file(self, path):
        sound = None
        try:
            rospy.logdebug("{%s} - Playing sound: %s",
                rospy.get_caller_id(), path)
            sound = pygame.mixer.Sound(self._path + path)
        except:
            rospy.logwarn("{%s} - Could not open %s",
                str(path))
        else:
            sound.play()

    def _playsound(self, dataArray):
        self._stop()
        soundname = dataArray[0]
        if len(dataArray) > 1:
            dataArray = dataArray[1:]
            info = dataArray[0]

            if len(dataArray) > 1:
                dataArray = dataArray[1:]
                return
            elif info == "random":
                self._play_random()
            elif info == "mood":
                mood_sounds = ["sad","happy"]
                response = self._face_req(True)
                if response.resp_is_sad:
                    soundpath = soundname + mood_sounds[0]+".wav"
                else:
                    soundpath = soundname + mood_sounds[1]+".wav"
        else:
            soundpath = soundname
        self._play_sound_file(soundpath)

    def _stop(self):
        pygame.mixer.stop()

def shutdown():
    rospy.loginfo("{%s} - stopping sound node!",
        rospy.get_caller_id())
    pygame.quit()
    rospy.signal_shutdown("Stopping sound node")

def Sound():
    rospy.init_node("robofriend_sound", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting sound node!",
        rospy.get_caller_id())

    # create Service
    rospy.wait_for_service('/robofriend/face_sound_stat')
    face_req = rospy.ServiceProxy('/robofriend/face_sound_stat', SrvFaceStatusData)

    sound = SoundDataHandler(face_req)

    # create Service
    rospy.Service('/robofriend/sound', SrvSoundData, sound._service_handler)
    rospy.spin()

if __name__ == '__main__':
    try:
        Sound()
    except rospy.ROSInterruptException:
        pass
