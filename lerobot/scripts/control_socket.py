import asyncio
import math
import re
import threading
import json
import time
import cv2
import av
import numpy as np
import websockets
from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription, MediaStreamTrack, VideoStreamTrack
from lerobot.common.robot_devices.control_configs import TeleoperateControlConfig
from lerobot.common.robot_devices.control_utils import control_loop
from lerobot.common.robot_devices.robots.configs import AlohaRobotConfig
from lerobot.common.robot_devices.robots.utils import make_robot, make_robot_from_config
from control_robot import teleoperate

# WebSocket signaling and control server URLs
SIGNALING_SERVER_URL = "ws://192.168.1.146:8000/ws/webrtc/"

import asyncio
import json
import re
import threading
import time
import cv2
import av
import websockets
from aiortc import (
    RTCIceCandidate,
    RTCPeerConnection,
    RTCSessionDescription,
    MediaStreamTrack,
)

# Create a single WebRTC peer connection
pc = RTCPeerConnection()

local_track = None

class RobotVideoStreamTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, robot):
        super().__init__()
        self.robot = robot

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        # Send images back to the robot
        for key in self.robot.images.keys():
            key_np = self.robot.images[key].numpy()
            frame = cv2.cvtColor(key_np, cv2.COLOR_RGB2BGR)

            frame = av.VideoFrame.from_ndarray(frame, format="bgr24")

            frame.pts = pts
            frame.time_base = time_base

            print(f"Sending frame with shape {frame.shape}")

            return frame
        # send a black frame if no images are available
        return av.VideoFrame(width=640, height=480)

class FlagVideoStreamTrack(VideoStreamTrack):
    """
    A video track that returns frames captured from the webcam.
    """

    def __init__(self):
        super().__init__()  # don't forget this!
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Could not start video capture")

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Could not read frame from webcam")

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = av.VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def __del__(self):
        self.cap.release()

def configure_teleop():
    """Configures the robot and teleoperation settings."""
    teleopControl = TeleoperateControlConfig(
        fps=30,
        teleop_time_s=None,
        display_cameras=False,
    )

    alohaRobotConfig = AlohaRobotConfig()
    alohaRobotConfig.max_relative_target = None

    return teleopControl, alohaRobotConfig

def create_robot(teleopControl, alohaRobotConfig):
    """Creates a robot instance."""
    robot = make_robot_from_config(alohaRobotConfig)

    return robot

def launch_teleop(robot, teleopControl):
    """Starts teleoperation in a separate thread and returns the Robot instance."""
    
    thread = threading.Thread(
        target=control_loop,
        kwargs={
            "robot": robot,
            "control_time_s": teleopControl.teleop_time_s,
            "fps": teleopControl.fps,
            "teleoperate": True,
            "display_cameras": False
        },
        daemon=True
    )
    thread.start()

    return thread

async def main():
    teleopControl, alohaRobotConfig = configure_teleop()
    robot = create_robot(teleopControl, alohaRobotConfig)
    
    pc = RTCPeerConnection()
    @pc.on("iceconnectionstatechange")
    def on_iceconnectionstatechange():
        print("ICE connection state is %s" % pc.iceConnectionState)

    async with websockets.connect(SIGNALING_SERVER_URL) as ws:
        # Create a video stream track
        track = FlagVideoStreamTrack() #RobotVideoStreamTrack(robot)
        pc.addTrack(track)

        # Create offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        # Send offer to the signaling server
        message = json.dumps({"type": "offer", "sdp": pc.localDescription.sdp})
        await ws.send(message)

        #thread = launch_teleop(robot, teleopControl)
        #thread.join()

        while True:
            message = await ws.recv()
            print("Message Received: ", message)
            data = json.loads(message)

            match data["type"]:
                case "answer":
                    answer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                    await pc.setRemoteDescription(answer)
                    print("Answer reecived")
                case "candidate":
                    await pc.addIceCandidate(data["candidate"])
                    print("ICE candidate added")
                case _:
                    print(f"Passing message")

            state_snapshot = json.dumps({"message": "socket test!", "data": robot.logs})
            ws.send(state_snapshot)

            time.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
