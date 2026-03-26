#! /usr/bin/env python3

from enum import Enum
import os
import subprocess
import time
import rclpy

from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from dis_tutorial3.srv import Speech

import soundfile as sf
import os
from kittentts import KittenTTS
import simpleaudio as sa
import numpy as np

class VoiceCommander(Node):

    def __init__(self, node_name='voice_commander', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        self.get_logger().info("VoiceCommander node has been initialized.")
        self.model = KittenTTS("KittenML/kitten-tts-mini-0.8")
        self.audio_srv = self.create_service(Speech, 'speech', self.speech_callback)
        self.get_logger().info("Speech service has been created and is ready to receive requests.")

    def speech_callback(self, request, response):
        self.get_logger().info(f"Received speech request: '{request.text}'")
        try:
            self.talk(request.text)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error in speech_callback: {e}")
            response.success = False
        return response

    def talk(self, text):
        try:
            self.get_logger().info(f"Generiram: {text}")
            audio_data = self.model.generate(text, voice="Bruno", speed=1.0)

            # 1. Shrani v začasno datoteko (soundfile pravilno zapiše headerje)
            file_path = os.path.abspath("temp_speech.wav")
            sf.write(file_path, audio_data, 24000)

            # 2. Predvajaj z zunanjim programom (to ne more sesuti tvojega noda!)
            # Poskusi najprej pw-play (standard na Ubuntu 22.04+), če ne gre, pa aplay
            try:
                subprocess.run(["pw-play", file_path], check=True)
            except FileNotFoundError:
                subprocess.run(["aplay", file_path], check=True)

            # 3. Pobriši datoteko
            if os.path.exists(file_path):
                os.remove(file_path)

        except Exception as e:
            self.get_logger().error(f"Napaka: {e}")


def main():
    rclpy.init(args=None)
    node = VoiceCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()