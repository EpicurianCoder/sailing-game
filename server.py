import math
import random
import asyncio
import io
import base64
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image

from boat_physics import boat_step, wrap_angle
from feature_gen import FeatureGenerator

app = FastAPI()

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

WIDTH, HEIGHT = 256, 256
ENGINE_DT = 1.0

def calculate_awa(global_wind_dir, boat_heading):
    relative_angle = global_wind_dir - boat_heading
    return wrap_angle(relative_angle)

@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")

@app.get("/api/map")
def get_map():
    land_pil, num_islands, coverage = FeatureGenerator.gen_land_feature(WIDTH)
    
    rgba = Image.new("RGBA", land_pil.size, (0, 0, 0, 0))
    pixels = rgba.load()
    mask = land_pil.load()
    
    for x in range(WIDTH):
        for y in range(HEIGHT):
            if mask[x, y] == 1:
                pixels[x, y] = (210, 180, 140, 255)
                
    buf = io.BytesIO()
    rgba.save(buf, format="PNG")
    b64_string = base64.b64encode(buf.getvalue()).decode("utf-8")
    
    return {"map_url": f"data:image/png;base64,{b64_string}"}

@app.websocket("/ws/physics")
async def physics_engine(websocket: WebSocket):
    await websocket.accept()
    
    # Generate random initial global wind direction
    initial_wind_dir = random.uniform(-math.pi, math.pi)

    state = {
        'global_wind_dir': initial_wind_dir, 
        'awa': 0.0, 
        'aws': 4.0,
        'velocity_boat_rotation': 0.0, 
        'velocity_lateral_drift': 0.0, 
        'velocity_forward': 0.0,
        'sail_size': 1.0, 
        'sail_angle': 0.0, 
        'relative_sail_angle': math.radians(-115),
        'lockout_target_angle': 0.0, 
        'out_of_control': -1,
        'boat_heading': 0.0, 
        'boat_x': 128.0, 
        'boat_y': 128.0,
        'stress_value': 0.0, 
        'trajectory_history': [],
        'state': 'unassigned', 
        'did_snap': None
    }

    current_inputs = {'rudder': 0.0, 'rope_length': 0.0}

    async def listen_for_inputs():
        nonlocal current_inputs
        try:
            while True:
                data = await websocket.receive_json()
                current_inputs['rudder'] = data.get('rudder', 0.0)
                current_inputs['rope_length'] = data.get('rope_length', 0.0)
        except Exception:
            pass

    input_task = asyncio.create_task(listen_for_inputs())

    try:
        while True:
            # 1. Update AWA
            state['awa'] = calculate_awa(state['global_wind_dir'], state['boat_heading'])
            
            # 2. Physics inputs mapping
            physics_inputs = {
                'rudder': current_inputs['rudder'] * math.radians(35),
                'rope_length': current_inputs['rope_length'] * math.radians(90),
                'sail_size': 4
            }

            # 3. Step Physics
            state = boat_step(state, physics_inputs)

            # 4. Kinematics
            state['boat_heading'] = wrap_angle(state['boat_heading'] + state['velocity_boat_rotation'] * ENGINE_DT)
            
            vel_x = (math.sin(state['boat_heading']) * state['velocity_forward'] * ENGINE_DT) + \
                    (math.cos(state['boat_heading']) * state['velocity_lateral_drift'] * ENGINE_DT)
            vel_y = (math.cos(state['boat_heading']) * state['velocity_forward'] * ENGINE_DT) - \
                    (math.sin(state['boat_heading']) * state['velocity_lateral_drift'] * ENGINE_DT)

            state['boat_x'] = (state['boat_x'] + vel_x) % WIDTH
            state['boat_y'] = (state['boat_y'] + vel_y) % HEIGHT
            
            # 6. Broadcast updated telemetry
            await websocket.send_json({
                'x': state['boat_x'],
                'y': state['boat_y'],
                'heading': state['boat_heading'],
                'sail_angle': wrap_angle(state['boat_heading'] + state['relative_sail_angle']),
                'wind_dir': state['global_wind_dir'],
                'rudder': current_inputs['rudder'],
                'rope_length': current_inputs['rope_length'],
                'speed': round(state['velocity_forward'], 2)
            })

            await asyncio.sleep(0.111)

    except WebSocketDisconnect:
        pass
    finally:
        input_task.cancel()