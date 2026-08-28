# requirements: Pillow, numpy

import pygame
import math
import random
import asyncio
from boat_physics import boat_step
import sys

from PIL import Image, ImageDraw
import numpy as np

# Simulation Settings
MAP_SIZE = 256

# colors
WATER_COLOR = (173, 216, 230)
LAND_COLOR = (210, 180, 140)
BOAT_COLOR = (255, 255, 255)
SAIL_COLOR = (255, 0, 255)
WIND_COLOR = (255, 0, 0)

# Vector Annotations
SPEED_VEC_COLOR = (0, 0, 255)     # Blue
WIND_VEC_COLOR = (0, 255, 0)      # Green
OPT_SAIL_COLOR = (255, 255, 0)    # Yellow
RESULTANT_COLOR = (255, 165, 0)   # Orange
FWD_FORCE_COLOR = (0, 255, 255)   # Cyan

# --- 1. ENVIRONMENT SETUP ---
WIDTH, HEIGHT = 256, 256
SCALE_FACTOR = 2
TWO_PI = math.pi * 2
FPS = 5
WIND_SCALE = 15.0
WIND_SPEED = 4.0

# --- 2. ENGINE CONSTANTS & STATE ---
# Sourced from the physics engine limits
MAX_SAIL_ANGLE = 1.57 

ENGINE_DT = 1.0 / 1.0


class FeatureGenerator:
    @staticmethod
    def gen_land_feature(map_size=MAP_SIZE):
        """
        Generates the procedural landmasses and protects starting/finish zones.
        Returns: (PIL_Image, int_island_count, float_coverage_percentage)
        """
        attempts = 0
        while True:
            attempts += 1
            if attempts > 50:
                print("Too many attempts, settling for current map.")
                break
                
            feature_image = Image.new("L", (map_size, map_size), 0)
            draw = ImageDraw.Draw(feature_image)

            num_landmasses = random.randint(1, 4)
            for i in range(num_landmasses):
                base_radius = random.uniform(14.0, 35.0)
                amplitude = random.uniform(8.0, 15.0)
                center_x, center_y = random.uniform(0, map_size), random.uniform(0, map_size)
                phases = [random.uniform(0, math.pi * 2) for _ in range(4)]

                points = []
                num_steps = 100
                for step in range(num_steps):
                    theta = (step / num_steps) * (2 * math.pi)
                    noise_val = (
                        math.sin(theta * 1 + phases[0]) * 1.0 +
                        math.sin(theta * 2 + phases[1]) * 0.5 +
                        math.sin(theta * 4 + phases[2]) * 0.25 +
                        math.sin(theta * 8 + phases[3]) * 0.125
                    ) / 1.875
                    
                    r = base_radius + (noise_val * amplitude)
                    points.append((center_x + r * math.cos(theta), center_y + r * math.sin(theta)))

                draw.polygon(points, fill=1)

            # --- THE C-OPTIMIZED FIX ---
            # Convert to numpy array for instant math
            img_array = np.asarray(feature_image)
            
            # 1. Instantly sum all pixels
            land_pixels = img_array.sum()
            coverage = land_pixels / (map_size * map_size)
            
            if coverage > 0.25:
                continue

            # 2. Instantly check the starting box (Y: 108 to 148, X: 78 to 178)
            starting_box = img_array[108:148, 78:178]
            if starting_box.any():  # If any pixel is 1, it fails
                continue

            return feature_image, num_landmasses, coverage
            
        return feature_image, num_landmasses, coverage


def to_physics_dimensions(normalized_inputs_dict):
    new_rope_length = map_input_to_radians(normalized_inputs_dict['rope_length'], math.radians(90))  # unsigned
    new_rudder = map_input_to_radians(normalized_inputs_dict['rudder'], math.radians(45))  # signed
    new_sail_size = round(max(0.0, min(1.0, normalized_inputs_dict['sail_size'])) * 4)
    new_dict = {
            'rudder': new_rudder,
            'rope_length': new_rope_length,
            'sail_size': new_sail_size
        }
    return new_dict

def initialize_state():
    global_wind_dir = 0.0
    boat_heading =  0.0
    relative_sail_angle = math.radians(-115)
    
    boat_x = 128
    boat_y = 128
    
    print("Initialized to:")
    print(f"global_wind_dir to: {global_wind_dir}")
    print(f"boat_heading to: {boat_heading}")
    print(f"relative_sail_angle to: {relative_sail_angle}")
    return global_wind_dir, boat_heading, relative_sail_angle, boat_x, boat_y
    
def get_random_signed_radian():
    return random.uniform(-math.pi, math.pi)

def calculate_awa(global_wind_dir, boat_heading):
    relative_angle = global_wind_dir - boat_heading
    return wrap_angle(relative_angle)

def map_input_to_radians(input_val, max_rads):
    sign = -1 if input_val < 0 else 1
    value = max(0.0, min(1.0, abs(input_val)))
    
    return (value * max_rads) * sign

def wrap_angle(angle):
    return (angle + math.pi) % TWO_PI - math.pi

def add_angle_wrap(angle, angle_to_add):
    theta = angle + angle_to_add
    return wrap_angle(theta)

def calculate_relative_sail_angle(sail_angle, boat_heading):
    relative_angle = sail_angle - boat_heading
    return wrap_angle(relative_angle)

def draw_arrow(surface, color, start, end, width=3, head_size=12):
    pygame.draw.line(surface, color, start, end, width)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    p1 = (end[0] - head_size * math.cos(angle - math.pi / 6),
          end[1] - head_size * math.sin(angle - math.pi / 6))
    p2 = (end[0] - head_size * math.cos(angle + math.pi / 6),
          end[1] - head_size * math.sin(angle + math.pi / 6))
    pygame.draw.polygon(surface, color, [end, p1, p2])
    
def draw_boat(surface, state):
    screen_y = 256 - state['boat_y']
    
    front_x = state['boat_x'] + 5 * math.sin(state['boat_heading'])
    front_y = screen_y - 5 * math.cos(state['boat_heading'])
    back_x = state['boat_x'] - 5 * math.sin(state['boat_heading'])
    back_y = screen_y + 5 * math.cos(state['boat_heading'])
    
    draw_arrow(surface, "black", (back_x, back_y), (front_x, front_y), 3, 2)
    
    sail_angle = add_angle_wrap(state['boat_heading'], state['relative_sail_angle'])
    
    visual_length = 20
    sail_end_x = front_x + math.sin(sail_angle) * visual_length
    sail_end_y = front_y - math.cos(sail_angle) * visual_length

    draw_arrow(surface, (145, 89, 102), (front_x, front_y), (sail_end_x, sail_end_y), 3, 5)
    
def pil_to_land_channel(pil_image):
    mapped_image = pil_image.point(lambda p: 0 if p == 1 else 255).convert("RGB")
    
    data = mapped_image.tobytes()
    size = mapped_image.size
    land_surface = pygame.image.fromstring(data, size, "RGB")
    
    land_surface.set_colorkey((255, 255, 255))
    
    return land_surface

def print_inputs(game_inputs_raw, game_inputs_physics):
    print("** PHYSICS")
    print(f"rope_length is {game_inputs_physics['rope_length']}")
    print(f"rudder is {game_inputs_physics['rudder']}")
    print(f"sail_size: {game_inputs_physics['sail_size']}")
    
    print("** RAW")
    print(f"rope_length is {game_inputs_raw['rope_length']}")
    print(f"rudder is {game_inputs_raw['rudder']}")
    print(f"sail_size: {game_inputs_raw['sail_size']}")

def print_game_state(game_state, framecount):
    print(f"global_wind_dir is {game_state['global_wind_dir']}")
    print(f"aws is {game_state['aws']}")
    print(f"awa: {game_state['awa']}")

    print(f"framecount is {framecount}")
    
print("** Booting Pygame...")
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Sailing Engine")
clock = pygame.time.Clock()
    
async def main():
    try:
        
        # Initial physics state dictionary
        game_state = {
            'global_wind_dir': 0.0,
            'awa': 0.0,                       
            'aws': 0.0,                       
            'velocity_boat_rotation': 0.0,       
            'velocity_lateral_drift': 0.0,    
            'velocity_forward': 0.0,          
            'sail_size': 1.0,                 
            'sail_angle': 0.0,
            'relative_sail_angle': 0.0,         
            'lockout_target_angle': 0.0,      
            'out_of_control': -1,
            'boat_heading': 0.0,
            'boat_x': 0,
            'boat_y': 0,
            'trajectory_history': [],
            'stress_value': 0.0,
            'state': 'unassigned',
            'did_snap': None
        }
    
        # Continuous Inputs (all continuous, two with negative values)
        normalized_inputs = {
            'rudder': 0.0,       # Range: [-1.0, 1.0]
            'rope_length': 0.0,  # Range: [0.0, 1.0]
            'sail_size': 1.0     # Range: [0.0, 1.0]
        }
    
        # Physics Based inputs (2 radians floats and 1 int)
        physics_inputs = {
            'rudder': 0.0,
            'rope_length': 0.0,
            'sail_size': 1
        }
    
        # MAIN GAME LOOP
        running = True
        
        print("** Initializing boat state...")
        # GENERATE GLOBAL WIND AND BOAT VALUE FOR START
        global_wind_dir, boat_heading, relative_sail_angle, boat_x, boat_y = initialize_state()
        
        game_state['global_wind_dir'] = global_wind_dir
        game_state['aws'] = WIND_SPEED  # set windspeed
        
        game_state['boat_x'] = boat_x
        game_state['boat_y'] = boat_y
        game_state['boat_heading'] = boat_heading
        
        game_state['relative_sail_angle'] = relative_sail_angle
        framecount = 0
        
        print("*** Initialized Values:")
        print_game_state(game_state, framecount)
        
        print("** Generating Landmasses... \n")
        land_PIL, num_landmasses, coverage = FeatureGenerator.gen_land_feature()
        land_channel = pil_to_land_channel(land_PIL)
        print("Land generated successfully! Starting main loop...")
        
        print(f"\nCoverage: {coverage}\n")
        print(f"\nNo. of Landmasses: {num_landmasses}\n")
        
        pygame.init()
        # pygame.key.set_repeat(150, 50)
        
        running = True
        
        while running:
            # Processes EVERY button event during the frame (limit user input rates HERE!!!)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
        
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # Accumulates the inputs from a frame step
                        running = False
                        break
                    if event.key == pygame.K_UP:
                        # Accumulates the inputs from a frame step
                        new_length = normalized_inputs['rope_length'] + 0.05
                        normalized_inputs["rope_length"] = round(max(0.0, min(1.0, new_length)), 2)
        
                    elif event.key == pygame.K_DOWN:
                        new_length = normalized_inputs['rope_length'] - 0.05
                        normalized_inputs["rope_length"] = round(max(0.0, min(1.0, new_length)), 2)
                        
                    if event.key == pygame.K_LEFT:
                        # Accumulates the inputs from a frame step
                        new_rudder = normalized_inputs['rudder'] - 0.1
                        normalized_inputs["rudder"] = round(max(-1.0, min(1.0, new_rudder)), 2)
        
                    elif event.key == pygame.K_RIGHT:
                        new_rudder = normalized_inputs['rudder'] + 0.1
                        normalized_inputs["rudder"] = round(max(-1.0, min(1.0, new_rudder)), 2)
        
                    elif event.key == pygame.K_a:
                        game_state["global_wind_dir"] = add_angle_wrap(game_state['global_wind_dir'], - math.pi/20)
        
                    elif event.key == pygame.K_s:
                        game_state["global_wind_dir"] = add_angle_wrap(game_state['global_wind_dir'], + math.pi/20)
            
            if not running:
                break
        
            if running:
                pygame.time.delay(200)
                
            # Get USER INPUT and calcualte current AWA
            physics_inputs_dict = to_physics_dimensions(normalized_inputs)
            game_state['awa'] = calculate_awa(game_state['global_wind_dir'], game_state['boat_heading'])
            
            print("*** Step: ")
            print_game_state(game_state, framecount)
            framecount += 1
            print_inputs(normalized_inputs, physics_inputs_dict)
            
            # Step the Physics Engine
            next_state = boat_step(game_state, physics_inputs_dict)
            
            # 1. Update the Heading (Multiply angular velocity by DT)
            next_state['boat_heading'] = add_angle_wrap(
                next_state['boat_heading'], 
                next_state['velocity_boat_rotation'] * ENGINE_DT
            )
            
            # 2. Calculate the X and Y movement components (Multiply speeds by DT)
            vel_x = (math.sin(next_state['boat_heading']) * next_state['velocity_forward'] * ENGINE_DT) + \
                    (math.cos(next_state['boat_heading']) * next_state['velocity_lateral_drift'] * ENGINE_DT)
                    
            vel_y = (math.cos(next_state['boat_heading']) * next_state['velocity_forward'] * ENGINE_DT) - \
                    (math.sin(next_state['boat_heading']) * next_state['velocity_lateral_drift'] * ENGINE_DT)
                    
            # 3. Apply to coordinates (Using modulo % to wrap around the screen edges)
            next_state['boat_x'] = (next_state['boat_x'] + vel_x) % WIDTH
            next_state['boat_y'] = (next_state['boat_y'] + vel_y) % HEIGHT
        
            # Draw WATER
            screen.fill((30, 144, 255)) # Simple ocean blue background
            
            # Draw LAND
            screen.blit(land_channel, (0, 0))
            
            # Draw WIND (Global)
            global_wind_dir_end_x = 30 + math.sin(next_state['global_wind_dir']) * WIND_SCALE
            global_wind_dir_end_y = 30 - math.cos(next_state['global_wind_dir']) * WIND_SCALE
            draw_arrow(screen, (255, 255, 100), (30, 30), (global_wind_dir_end_x, global_wind_dir_end_y), 3, 10)
            
            # Draw BOAT
            # boat_feature = FeatureGenerator.get_boat_feature_layer(next_state['boat_y'], next_state['boat_x'], next_state['boat_heading'])
            draw_boat(screen, next_state)
            
            # stack = FeatureGenerator.build_7_channel_grid(next_state, land_PIL)
            
            if framecount % 4 == 0:
                if normalized_inputs["rudder"] > 0.8:
                    normalized_inputs["rudder"] = normalized_inputs["rudder"] - 0.2
                elif normalized_inputs["rudder"] > 0.5:
                    normalized_inputs["rudder"] = normalized_inputs["rudder"] - 0.15
                elif normalized_inputs["rudder"] > 0.1:
                    normalized_inputs["rudder"] = normalized_inputs["rudder"] - 0.1
                elif normalized_inputs["rudder"] > 0.0:
                    normalized_inputs["rudder"] = 0.0
                elif normalized_inputs["rudder"] < - 0.8:
                    normalized_inputs["rudder"] = normalized_inputs["rudder"] + 0.2
                elif normalized_inputs["rudder"] < - 0.5:
                    normalized_inputs["rudder"] = normalized_inputs["rudder"] + 0.15
                elif normalized_inputs["rudder"] < - 0.1:
                    normalized_inputs["rudder"] = normalized_inputs["rudder"] + 0.1
                elif normalized_inputs["rudder"] < 0.0:
                    normalized_inputs["rudder"] = 0.0
            
            # Update States
            # awa:          START of each frame, ENV dependant
            # rope_length:  START of each frame, USR dependant
            
            game_state = next_state
            
            pygame.display.flip()
            clock.tick(FPS)
            await asyncio.sleep(0)
        
        pygame.quit()
        sys.exit()
    except Exception as e:
        # 1. Grab the full Python crash report
        import traceback
        error_msg = traceback.format_exc()
        
        # 2. Force it into the Browser's F12 Console
        import sys
        if sys.platform == "emscripten":
            import js
            js.console.error("!!! PYTHON CRASHED !!!")
            js.console.error(error_msg)
        else:
            print(error_msg)
            
        # Keep the page alive so you can read the error
        while True:
            await asyncio.sleep(1)
    
if __name__ == "__main__":
    asyncio.run(main())