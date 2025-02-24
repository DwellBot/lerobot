import asyncio
import threading
import json
import time
import cv2
import av
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.media import MediaPlayer
from lerobot.common.robot_devices.robots.utils import make_robot
from lerobot.common.utils.utils import init_hydra_config, init_logging, log_say, none_or_int
from lerobot.common.robot_devices.robots.utils import Robot
from lerobot.common.robot_devices.utils import busy_wait, safe_disconnect
from control_robot import teleoperate

# WebSocket signaling server URL
SIGNALING_SERVER_URL = "ws://localhost:8080/ws/webrtc/"
CONTROL_SERVER_URL = "ws://192.168.1.146:8000/ws/control/"

# Create WebRTC peer connection
pc = RTCPeerConnection()

class VideoStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)

    async def recv(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return av.VideoFrame.from_ndarray(frame, format="rgb24")

async def handle_signaling():
    async with websockets.connect(SIGNALING_SERVER_URL) as ws:
        async def send_offer():
            # Capture video
            local_track = VideoStreamTrack()
            pc.addTrack(local_track)

            # Create WebRTC offer
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)

            # Send the offer through WebSocket
            await ws.send(json.dumps({"type": "offer", "sdp": offer.sdp}))
            print("Sent offer to signaling server.")

        async def handle_messages():
            async for message in ws:
                data = json.loads(message)
                print("Received message:", data)
                
                if data["type"] == "answer":
                    print("Received answer from signaling server.")
                    await pc.setRemoteDescription(RTCSessionDescription(data["sdp"], "answer"))
                elif data["type"] == "ice-candidate":
                    candidate = data["candidate"]
                    await pc.addIceCandidate(candidate)
                    print("Added ICE candidate.")

        # Handle ICE candidates
        @pc.on("icecandidate")
        async def on_ice_candidate(candidate):
            if candidate:
                await ws.send(json.dumps({"type": "ice-candidate", "candidate": candidate}))
                print("Sent ICE candidate.")

        # Run tasks concurrently
        await asyncio.gather(send_offer(), handle_messages())

async def websocket_client():
    try:
        async with websockets.connect(CONTROL_SERVER_URL) as websocket:
            # Receive initial message
            response = await websocket.recv()
            print(f"Server: {response}")

            # Launch teleoperation in a separate thread
            robot = launch_teleop()
            
            # Start monitoring loop in the main thread
            await monitor_robot(robot, websocket)
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(5)

def launch_teleop():
    """Starts teleoperation in a separate thread and returns the Robot instance."""
    args = {'mode': 'teleoperate', 'robot_path': 'lerobot/configs/robot/aloha.yaml', 'robot_overrides': ['~cameras', 'max_relative_target=null'], 'fps': None, 'display_cameras': 1}
    robot_cfg = init_hydra_config(args['robot_path'], args['robot_overrides'])

    # Create robot instance
    robot = make_robot(robot_cfg)

    # Start teleoperation in a new thread
    teleop_thread = threading.Thread(target=teleoperate, args=(robot,), kwargs={"fps": 30, "teleop_time_s": 30, "display_cameras": False})
    teleop_thread.start()

    return robot  # Return robot so main thread can monitor

async def monitor_robot(robot, websocket):
    """Monitors the robot's state and sends WebSocket messages."""
    while True:
        try:
            # Get robot state snapshot
            state_snapshot = json.dumps({"message": "socket test!", "data": robot.logs})
            
            # Send WebSocket message asynchronously
            await websocket.send(state_snapshot)

            print(f"[Main] Sent WebSocket message: {state_snapshot}")
            
            await asyncio.sleep(0.5)  # Adjust as needed
        except Exception as e:
            print(f"[Main] Error sending WebSocket message: {e}")
            break  # Exit loop on error

# Run both WebRTC and control WebSocket handlers
async def main():
    await asyncio.gather(handle_signaling(), websocket_client())

if __name__ == "__main__":
    asyncio.run(main())
