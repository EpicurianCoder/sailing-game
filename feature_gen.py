import math
import numpy as np
from PIL import Image, ImageDraw
import random
from noise import pnoise2
from scipy.ndimage import binary_dilation

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
            feature_image = Image.new("L", (map_size, map_size), 0)
            draw = ImageDraw.Draw(feature_image)

            # Use perlin noise and circles with amplitude value to draw islands
            num_landmasses = random.randint(1, 4)
            for i in range(num_landmasses):
                base_radius = random.uniform(14.0, 35.0)
                noise_scale = random.uniform(0.7, 1.5)
                amplitude = random.uniform(8.0, 15.0)
                center_x, center_y = random.uniform(0, map_size), random.uniform(0, map_size)

                points = []
                num_steps = 100
                for step in range(num_steps):
                    theta = (step / num_steps) * (2 * math.pi)
                    noise_val = pnoise2(math.cos(theta) * noise_scale + center_x,
                                        math.sin(theta) * noise_scale + center_y, octaves=4)
                    r = base_radius + (noise_val * amplitude)
                    points.append((center_x + r * math.cos(theta), center_y + r * math.sin(theta)))

                draw.polygon(points, fill=1)

            pixels = feature_image.load()
            land_pixels = sum(1 for x in range(map_size) for y in range(map_size) if pixels[x, y] == 1)
            coverage = land_pixels / (map_size * map_size)

            # Dont Cover more than 25%
            if coverage > 0.25:
                continue

            # Protect Mid-Map Boat Starting Box (Sim Y around 128 -> Screen Y: 108 to 148)
            if any(pixels[max(0, min(map_size-1, x)), max(0, min(map_size-1, y))] == 1 
                   for x in range(78, 178) for y in range(108, 148)):
                continue

            return feature_image, num_landmasses, coverage

    @staticmethod
    def get_boat_feature_layer(x, y, boat_angle_radians, map_size=MAP_SIZE):
        """
        Creates a temporary mask of the boat used purely for collision/distance detection.
        """
        feature_image = Image.new("L", (map_size, map_size), 0)
        draw = ImageDraw.Draw(feature_image)

        screen_y = map_size - y
        boat_rad = boat_angle_radians  # Already in radians
        front_x = x + 5 * math.sin(boat_rad)
        front_y = screen_y - 5 * math.cos(boat_rad)
        back_x = x - 5 * math.sin(boat_rad)
        back_y = screen_y + 5 * math.cos(boat_rad)
        draw.line([(back_x, back_y), (front_x, front_y)], fill=1, width=3)

        return feature_image

    @staticmethod
    def _draw_hud_block(angle_radians, map_size=MAP_SIZE):
        """
        Draws a 40x40 two-tone bisected square in the bottom-left corner.
        Front half of the angle is 1.0, back half is 0.5. 
        """
        img = Image.new("F", (map_size, map_size), 0.0)
        draw = ImageDraw.Draw(img)

        # Bottom-left 40x40 box coordinates
        box_x1, box_y1 = 0, map_size - 40
        box_x2, box_y2 = 40, map_size
        cx, cy = 20, map_size - 20

        # Draw the base "back" half (0.5 intensity)
        draw.rectangle([box_x1, box_y1, box_x2, box_y2], fill=0.5)

        # Calculate vector for the angle
        rad = angle_radians  # Already in radians
        dx = math.sin(rad)
        dy = -math.cos(rad) # Y is inverted on screen

        # Perpendicular vector for the bisection line
        px, py = -dy, dx

        # Create a massive polygon pointing in the direction of the angle
        poly_size = 60
        p1 = (cx + px * poly_size, cy + py * poly_size)
        p2 = (cx - px * poly_size, cy - py * poly_size)
        p3 = (cx - px * poly_size + dx * poly_size, cy - py * poly_size + dy * poly_size)
        p4 = (cx + px * poly_size + dx * poly_size, cy + py * poly_size + dy * poly_size)

        # Draw the "front" half (1.0 intensity)
        draw.polygon([p1, p2, p3, p4], fill=1.0)

        # Mask out everything outside the 40x40 box by overwriting it with 0.0
        draw.rectangle([0, 0, map_size, box_y1], fill=0.0)             # Top mask
        draw.rectangle([box_x2, 0, map_size, map_size], fill=0.0)      # Right mask

        return np.asarray(img, dtype=np.float32)
    
    @staticmethod
    def los_finish_line_bearing(boat_xy, boat_bearing, land_channel, map_size=MAP_SIZE):
        land_arr = np.asarray(land_channel)
        boat_x, boat_y = boat_xy
        bearing_rad = boat_bearing  # Already in radians
    
        dir_x = math.sin(bearing_rad)
        dir_y = math.cos(bearing_rad)
    
        if dir_y <= 0:
            return False
    
        for step in range(1, int(map_size * 2)):
            curr_x = boat_x + dir_x * step
            curr_y = boat_y + dir_y * step
    
            if curr_y >= map_size:
                return True
    
            if curr_x < 0 or curr_x >= map_size:
                return False
    
            grid_x = int(curr_x)
            grid_y = int(map_size - curr_y)
    
            grid_x = max(0, min(map_size - 1, grid_x))
            grid_y = max(0, min(map_size - 1, grid_y))
    
            if land_arr[grid_y, grid_x] == 1:
                return False

    @staticmethod
    def build_7_channel_grid(state, land_pil, map_size=MAP_SIZE):
        """
        Builds the 7-channel normalized float array (7, 256, 256) for the CNN.
        """
        channels = []

        # ==========================================
        # CHANNEL 0: Wind HUD Block
        # ==========================================
        wind_hud = FeatureGenerator._draw_hud_block(state['global_wind_dir'], map_size)
        channels.append(wind_hud)

        # ==========================================
        # CHANNEL 1: Sail HUD Block
        # ==========================================
        # Simply pass the global sail angle from the state
        sail_hud = FeatureGenerator._draw_hud_block(state['sail_angle'], map_size)
        channels.append(sail_hud)

        # ==========================================
        # CHANNEL 2: Line Travelled (Fading Wake)
        # ==========================================
        path_img = Image.new("F", (map_size, map_size), 0.0)
        draw_path = ImageDraw.Draw(path_img)

        # Draw historical points as connected segments. Fade out older points.
        history_len = len(state['trajectory_history'])
        if history_len > 1:
            for i in range(history_len - 1):
                p1_x, p1_y, _ = state['trajectory_history'][i]
                p2_x, p2_y, speed = state['trajectory_history'][i + 1]

                s1_y = map_size - p1_y
                s2_y = map_size - p2_y

                # Speed determines base intensity, index determines age fade
                speed_intensity = max(0.1, min(1.0, speed / 3.0))
                fade_multiplier = (i + 1) / history_len 

                draw_path.line([(p1_x, s1_y), (p2_x, s2_y)], fill=speed_intensity * fade_multiplier, width=3)
        channels.append(np.asarray(path_img, dtype=np.float32))

        # ==========================================
        # CHANNEL 3: Proximity Indicator (Land Border)
        # ==========================================
        land_np = np.asarray(land_pil, dtype=np.float32)

        # Dilate the land by a few pixels to create a danger border
        land_mask = (land_np == 1.0)
        dilated_mask = binary_dilation(land_mask, iterations=6)
        border_mask = dilated_mask & ~land_mask

        prox_channel = land_np.copy()
        prox_channel[border_mask] = 0.5  # Border is halfway dangerous
        channels.append(prox_channel)

        # ==========================================
        # CHANNEL 4: Line of Sight (Finish Line)
        # ==========================================
        los_img = Image.new("F", (map_size, map_size), 0.0)
        draw_los = ImageDraw.Draw(los_img)

        # Calculate LOS directly here using the telemetry function!
        has_los = FeatureGenerator.los_finish_line_bearing((state['boat_x'], state['boat_y']), state['boat_heading'], land_pil)

        # If line of sight is clear, illuminate the finish line heavily
        finish_intensity = 1.0 if has_los else 0.0
        draw_los.line([(0, 0), (map_size, 0)], fill=finish_intensity, width=4)
        channels.append(np.asarray(los_img, dtype=np.float32))

        # ==========================================
        # CHANNEL 5: Reward Gradient (Distance)
        # ==========================================
        # Y=0 in image is Y=256 in simulation (Finish Line). Top = 0.0, Bottom = 1.0.
        gradient_1d = np.linspace(1.0, 0.0, map_size, dtype=np.float32)
        dist_gradient = np.tile(gradient_1d[:, np.newaxis], (1, map_size))
        channels.append(dist_gradient)

        # ==========================================
        # CHANNEL 6: Boat & Stress Level
        # ==========================================
        boat_img = Image.new("F", (map_size, map_size), 0.0)
        draw_boat = ImageDraw.Draw(boat_img)

        screen_y = map_size - state['boat_y']
        boat_rad = state['boat_heading']  # Already in radians

        # Draw 10px long boat (5px front, 5px back)
        front_x = state['boat_x'] + 5 * math.sin(boat_rad)
        front_y = screen_y - 5 * math.cos(boat_rad)
        back_x = state['boat_x'] - 5 * math.sin(boat_rad)
        back_y = screen_y + 5 * math.cos(boat_rad)

        stress_intensity = max(0.1, min(1.0, state['stress_value'] / 100.0))
        draw_boat.line([(back_x, back_y), (front_x, front_y)], fill=stress_intensity, width=3)
        channels.append(np.asarray(boat_img, dtype=np.float32))

        # Stack into shape: (7, 256, 256)
        return np.stack(channels, axis=0)