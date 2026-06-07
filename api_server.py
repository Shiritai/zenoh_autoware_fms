import asyncio
import json
import os

import cv2
import zenoh
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from zenoh_app.camera_autoware import MJPEG_server
from zenoh_app.list_autoware import list_autoware
from zenoh_app.pose_service import PoseServer
from zenoh_app.status_autoware import get_cpu_status, get_vehicle_status

MJPEG_HOST = '0.0.0.0'
MJPEG_PORT = 5000

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=['*'],
    allow_headers=['*'],
)

conf = zenoh.Config.from_file('config.json5')
session = zenoh.open(conf)
use_bridge_ros2dds = os.environ.get('USE_BRIDGE_ROS2DDS') == 'True'
pose_service = PoseServer(session, use_bridge_ros2dds)

class Scope:
    """Per-scope server channels: intent pub, telemetry sub + cache, camera."""
    def __init__(self, name, session, use_bridge_ros2dds):
        self.last_telemetry = {}
        self.intent_pub = session.declare_publisher(f'manual_control/{name}/intent')
        self.telemetry_sub = session.declare_subscriber(
            f'manual_control/{name}/telemetry', self._on_telemetry)
        self.mjpeg = MJPEG_server(session, name, use_bridge_ros2dds)

    def _on_telemetry(self, sample):
        try:
            self.last_telemetry = json.loads(sample.payload.to_bytes().decode('utf-8'))
        except Exception:
            pass


class ScopeRegistry:
    """Lazily create + cache one Scope per vehicle id."""
    def __init__(self, session, use_bridge_ros2dds):
        self._session = session
        self._use_bridge_ros2dds = use_bridge_ros2dds
        self._scopes = {}

    def get(self, name):
        if name not in self._scopes:
            self._scopes[name] = Scope(name, self._session, self._use_bridge_ros2dds)
        return self._scopes[name]


scopes = ScopeRegistry(session, use_bridge_ros2dds)


@app.get('/')
async def root():
    return {'message': 'Hello World'}


@app.get('/list')
async def manage_list_autoware():
    return list_autoware(session, use_bridge_ros2dds)


@app.get('/zenoh/has-subscriber')
async def zenoh_has_subscriber(key: str):
    # True if any subscriber currently matches `key`.
    pub = session.declare_publisher(key)
    try:
        return {'matching': bool(pub.matching_status.matching)}
    finally:
        pub.undeclare()


@app.get('/status')
async def manage_status_autoware(scope: str = 'v1'):
    return {'cpu': get_cpu_status(session, scope, use_bridge_ros2dds), 'vehicle': get_vehicle_status(session, scope, use_bridge_ros2dds)}


@app.websocket('/video')
async def handle_ws(websocket: WebSocket, scope: str = 'v1'):
    await websocket.accept()
    mjpeg = scopes.get(scope).mjpeg
    try:
        while True:
            if mjpeg.camera_image is None:
                await asyncio.sleep(2)
            else:
                _, buffer = cv2.imencode('.jpg', mjpeg.camera_image)
                await websocket.send_bytes(buffer.tobytes())
                await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass


@app.websocket('/telemetry/stream')
async def telemetry_stream(websocket: WebSocket, scope: str = 'v1'):
    await websocket.accept()
    s = scopes.get(scope)
    try:
        while True:
            await websocket.send_json(s.last_telemetry)
            await asyncio.sleep(0.1)  # 10Hz
    except WebSocketDisconnect:
        pass


@app.get('/teleop/startup')
async def manage_teleop_startup(scope: str = 'v1'):
    """Start the camera stream; zenoh_control owns control and engage."""
    scopes.get(scope)
    return {
        'text': f'Startup manual control on {scope}.',
        'mjpeg_host': 'localhost' if MJPEG_HOST == '0.0.0.0' else MJPEG_HOST,
        'mjpeg_port': MJPEG_PORT,
    }


@app.websocket('/teleop/intent/ws')
async def handle_intent_ws(websocket: WebSocket, scope: str = 'v1'):
    await websocket.accept()
    intent_pub = scopes.get(scope).intent_pub
    try:
        while True:
            data = await websocket.receive_text()
            intent_pub.put(data)
    except WebSocketDisconnect:
        pass


@app.get('/map/list')
async def get_vehilcle_list():
    global pose_service
    pose_service.findVehicles()
    return list(pose_service.vehicles.keys())


@app.get('/map/pose')
async def get_vehicle_pose():
    global pose_service
    if pose_service is not None:
        return pose_service.returnPose()
    else:
        return []


@app.get('/map/goalPose')
async def get_vehicle_goalpose():
    global pose_service
    if pose_service is not None:
        return pose_service.returnGoalPose()
    else:
        return []


@app.get('/map/setGoal')
async def set_goal_pose(scope, lat, lon):
    global pose_service
    if pose_service is not None:
        print(f'[API SERVER] Set Goal Pose of {scope} as (lat={lat}, lon={lon})')
        pose_service.setGoal(scope, lat, lon)
        return 'success'
    else:
        return 'fail'


@app.get('/map/engage')
async def set_engage(scope):
    global pose_service
    if pose_service is not None:
        pose_service.engage(scope)
        return 'success'
    else:
        return 'fail'
