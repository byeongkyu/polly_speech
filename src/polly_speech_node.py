#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import boto3
import yaml
import pyaudio
import tempfile
import threading
import json
from ctypes import *
import contextlib
from std_msgs.msg import String, Bool
from polly_speech.msg import SpeechAction, SpeechResult, SpeechFeedback

VOWELS = ['@', 'a', 'e', 'E', 'i', 'o', 'O', 'u']

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextlib.contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)

class PollySpeechNode:
    def __init__(self):
        try:
            config_file = rospy.get_param('~config_file')
        except:
            rospy.logerr('please set param ~config_file...')
            exit(-1)

        with open(config_file) as f:
            data = yaml.load(f)

        with noalsaerr():
            self.p = pyaudio.PyAudio()

        self.client = boto3.client('polly',
            region_name=data['region'],
            aws_access_key_id=data['aws_access_key_id'],
            aws_secret_access_key=data['aws_secret_access_key'],
            endpoint_url="https://polly.us-east-1.amazonaws.com")

        try:
            self.lang = data['lang']
            self.voice = data['voice']
            self.default_pitch = data['default_pitch']
        except KeyError as e:
            self.lang = 'en-US'
            self.voice = 'Joanna'
            self.default_pitch = '+20%'

        rospy.loginfo('language [%s], voice [%s], default pitch [%s].'%(self.lang, self.voice, self.default_pitch))

        self.pub_status = rospy.Publisher('u_is_speaking', Bool, queue_size=10)
        self.pub_vowels = rospy.Publisher('lipsync_vowel', String, queue_size=10)
        self.speech_server = actionlib.SimpleActionServer('internal_speech', SpeechAction,
            execute_cb=self.execute_speech_callback, auto_start=False)
        self.speech_server.start()

        rospy.loginfo('%s initialized.'%rospy.get_name())
        rospy.spin()

    def execute_speech_callback(self, goal):
        result = SpeechResult()
        feedback = SpeechFeedback()

        msg_text = goal.text
        speech_text = ''
        if msg_text.startswith('$'):
            msg_text = msg_text.strip('$')
            lang_code, text_body = msg_text.split('|')

            speech_text = '<speak><prosody rate="medium" pitch="%s">'%self.default_pitch + '<lang xml:lang="%s">'%lang_code + text_body + '</lang></prosody></speak>'
        else:
            speech_text = '<speak><prosody rate="medium" pitch="%s">'%self.default_pitch + msg_text + '</prosody></speak>'

        resp = self.client.synthesize_speech(OutputFormat="json", Text=speech_text, SpeechMarkTypes=['viseme'], TextType="ssml", VoiceId=self.voice)
        with open(tempfile.gettempdir() + '/polly_wave.txt', 'w') as f:
            f.write(resp['AudioStream'].read())
            f.close()

        viseme_data = []
        with open(tempfile.gettempdir() + '/polly_wave.txt', 'r') as f:
            for line in f:
                viseme = json.loads(line)
                if viseme['value'] in VOWELS:
                    viseme_data.append([viseme['value'], viseme['time']])

        resp = self.client.synthesize_speech(OutputFormat="pcm", Text=speech_text, TextType="ssml", VoiceId=self.voice)
        with open(tempfile.gettempdir() + '/polly_wave.wav', 'w') as f:
            f.write(resp['AudioStream'].read())
            f.close()

        self.pub_status.publish(True)

        stream = self.p.open(
            format=self.p.get_format_from_width(2),
            channels=1,
            rate=16000,
            output=True)

        th = threading.Thread(target=self.handle_pub_viseme, args=(viseme_data,))
        th.start()
        with open(tempfile.gettempdir() + '/polly_wave.wav', 'rb') as pcm_file:
            data = pcm_file.read(100)

            while data != '':
                stream.write(data)
                data = pcm_file.read(100)
                feedback.is_speaking = True
                self.speech_server.publish_feedback(feedback)
        rospy.sleep(0.2)
        th.join()

        stream.stop_stream()
        stream.close()

        self.pub_vowels.publish('N')
        self.pub_status.publish(False)

        result.result = True
        self.speech_server.set_succeeded(result)

    def handle_pub_viseme(self, args):
        viseme_data = args

        index = 0
        rospy.sleep(viseme_data[index][1] / 1000.0)
        while not rospy.is_shutdown():
            self.pub_vowels.publish(viseme_data[index][0])
            index = index + 1
            if index >= len(viseme_data):
                break
            rospy.sleep((viseme_data[index][1] - viseme_data[index-1][1]) / 1000.0)

if __name__ == '__main__':
    rospy.init_node('polly_speech_node', anonymous=False)
    try:
        m = PollySpeechNode();
    except rospy.ROSInterruptException: pass
