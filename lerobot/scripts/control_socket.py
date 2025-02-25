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
CONTROL_SERVER_URL = "ws://192.168.1.146:8080/ws/webrtc/"


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

# class FlagVideoStreamTrack(VideoStreamTrack):
    # """
    # A video track that returns an animated flag.
    # """

    # def __init__(self):
    #     super().__init__()  # don't forget this!
    #     self.counter = 0
    #     height, width = 480, 640

    #     # generate flag
    #     data_bgr = np.hstack(
    #         [
    #             self._create_rectangle(
    #                 width=213, height=480, color=(255, 0, 0)
    #             ),  # blue
    #             self._create_rectangle(
    #                 width=214, height=480, color=(255, 255, 255)
    #             ),  # white
    #             self._create_rectangle(width=213, height=480, color=(0, 0, 255)),  # red
    #         ]
    #     )

    #     # shrink and center it
    #     M = np.float32([[0.5, 0, width / 4], [0, 0.5, height / 4]])
    #     data_bgr = cv2.warpAffine(data_bgr, M, (width, height))

    #     # compute animation
    #     omega = 2 * math.pi / height
    #     id_x = np.tile(np.array(range(width), dtype=np.float32), (height, 1))
    #     id_y = np.tile(
    #         np.array(range(height), dtype=np.float32), (width, 1)
    #     ).transpose()

    #     self.frames = []
    #     for k in range(30):
    #         phase = 2 * k * math.pi / 30
    #         map_x = id_x + 10 * np.cos(omega * id_x + phase)
    #         map_y = id_y + 10 * np.sin(omega * id_x + phase)
    #         self.frames.append(
    #             av.VideoFrame.from_ndarray(
    #                 cv2.remap(data_bgr, map_x, map_y, cv2.INTER_LINEAR), format="bgr24"
    #             )
    #         )

    # async def recv(self):
    #     pts, time_base = await self.next_timestamp()

    #     frame = self.frames[self.counter % 30]
    #     frame.pts = pts
    #     frame.time_base = time_base
    #     self.counter += 1
    #     return frame

    # def _create_rectangle(self, width, height, color):
    #     data_bgr = np.zeros((height, width, 3), np.uint8)
    #     data_bgr[:, :] = color
    #     return data_bgr


class RobotVideoStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, robot):
        """
        :param robot: The robot object that has .images
        :param camera_key: The key for the camera in robot.images dict.
        """
        super().__init__()
        self.robot = robot

    async def recv(self):
        """
        Invoked each time aiortc wants the next video frame to send.
        """
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
        

def parse_ice_candidate(candidate_dict: dict) -> RTCIceCandidate:
    """
    Parses a candidate dictionary of the form:
        {
            'candidate': 'candidate:0 1 UDP 2122252543 4a332e2e-ceb9-48b0-be2a-f263a1feb620.local 35695 typ host ...',
            'sdpMid': '0',
            'sdpMLineIndex': 0,
            'usernameFragment': '7ab0d46f'
        }
    into an aiortc RTCIceCandidate object.
    """
    candidate_line = candidate_dict.get("candidate", "")
    sdp_mid = candidate_dict.get("sdpMid")
    sdp_mline_index = candidate_dict.get("sdpMLineIndex")
    
    # Regex to parse ICE candidate fields from the candidate line
    pattern = (
        r"^candidate:(?P<foundation>\S+)\s+"
        r"(?P<component>\d+)\s+"
        r"(?P<protocol>\S+)\s+"
        r"(?P<priority>\d+)\s+"
        r"(?P<ip>\S+)\s+"
        r"(?P<port>\d+)\s+typ\s+(?P<type>\S+)"
        r"(?:\s+raddr\s+(?P<raddr>\S+)\s+rport\s+(?P<rport>\d+))?"
    )
    match = re.match(pattern, candidate_line)
    if not match:
        raise ValueError(f"Failed to parse ICE candidate line: {candidate_line}")
    
    parts = match.groupdict()

    # Construct the RTCIceCandidate instance
    return RTCIceCandidate(
        component=int(parts["component"]),
        foundation=parts["foundation"],
        priority=int(parts["priority"]),
        protocol=parts["protocol"].lower(),
        ip=parts["ip"],
        port=int(parts["port"]),
        type=parts["type"].lower(),
        relatedAddress=parts.get("raddr"),
        relatedPort=int(parts["rport"]) if parts.get("rport") else None,
        sdpMid=sdp_mid,
        sdpMLineIndex=sdp_mline_index,
    )

async def establish_webrtc_connection(ws, robot):
    """Handles WebRTC signaling over a single WebSocket connection."""
    async def send_offer():
        local_track = RobotVideoStreamTrack(robot)
        pc.addTrack(local_track)

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        
        await ws.send(json.dumps({"type": "offer", "sdp": pc.localDescription.sdp}))

        print("Sent offer to signaling server.")

    async def handle_messages():
        async for message in ws:
            data = json.loads(message)
            webrtcMessageType = data.get("type")

            match webrtcMessageType:
                case "answer":
                    await pc.setRemoteDescription(RTCSessionDescription(data["sdp"], "answer"))
                    print("Received and set remote answer.")
                case "ice-candidate":
                    candidate = data.get("candidate")
                    candidate_str = candidate.get("candidate")
                    
                    if not candidate_str:
                        print("Received empty ice candidate, ignoring.")
                        return

                    candidate = parse_ice_candidate(data["candidate"])
                    await pc.addIceCandidate(candidate)
                    print("Added ICE candidate.")
                case _:
                    print("Unhandled message type, ignoring:", data)
    
    @pc.on("iceconnectionstatechange")
    def on_iceconnectionstatechange():
        print("ICE connection state is %s" % pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            print("ICE failed: No path for streaming was found.")
        elif pc.iceConnectionState == "connected":
            print("ICE connected: Peer-to-peer path is established!")
        elif pc.iceConnectionState == "completed":
            print("ICE completed: Final state, all checks done.")

    @pc.on("track")
    def on_track(track):
        print("Received remote track:", track)
        print("Track kind:", track.kind)
        print("Track ID:", track.id)

    @pc.on("datachannel")
    def on_datachannel(channel):
        print("Received data channel")

    await asyncio.gather(send_offer(), handle_messages())

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

    # thread.join()
    return thread

def monitor_robot(robot, websocket):
    """Monitors robot state and sends WebSocket messages."""
    while True:
        # Send telemetry data back to the controller
        try:
            state_snapshot = json.dumps({"message": "socket test!", "data": robot.logs})
            #websocket.send(state_snapshot)
            print(f"[Main] Sent WebSocket message: {state_snapshot}")
        except Exception as e:
            print(f"[Main] Error sending WebSocket message: {e}")
            break

        time.sleep(0.1)

async def main():
    teleopControl, alohaRobotConfig = configure_teleop()
    robot = create_robot(teleopControl, alohaRobotConfig)
    
    try:
        async with websockets.connect(SIGNALING_SERVER_URL) as ws:
            await establish_webrtc_connection(ws, robot)

            thread = launch_teleop(robot, teleopControl)
            monitor_robot(robot, ws)
            thread.join()
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(5)

if __name__ == "__main__":
    asyncio.run(main())
