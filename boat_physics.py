import math
# from boat_physics_helpers import lift_and_drag, get_sign
# import random

# TWEAK THESE
MASS_BOAT = 400
INERTIA_BOAT = 1000
OFFSET_CE_CLR = 0.08
DRAG_HULL = 3
DRAG_KEEL = 9
DRAG_ROTATION = 2200.0

LATERAL_LIMITER = 9

DRAG_HULL_WIND = 14

RUDDER_POWER = 2000.0
RUDDER_MAX = math.radians(45)
DELTA_TIME = 1.0 / 60.0
MAX_SAIL_ANGLE = 1.57
SAIL_SWING_STEP = math.radians(10)
THRESHOLD_15_DEG = math.radians(5)
DEGREE_TO_RAD = math.pi / 180
TWO_PI = math.pi * 2
FLAP_ZONE = math.radians(5)
SAIL_SWING_SPEED = 2.0

BASE_LIFT_MULTIPLIER = 20.0
BASE_DRAG_MULTIPLIER = 3.0

ALIGN_STEP_SIZE = math.radians(20)
WING_PARA_STEP_SIZE = math.radians(10)

# aws: apparent wind speed
# aoa: angle of attack
# awa: apparent wind angle - NB! This is the source angle of the wind

# USER CONTROLS
# Rudder - This is a -1.0 -> 1.0 value and user has 'control' (not absolute)
# rudder_continuous = state['rudder_continuous']

# Sail Rope Length... this is how far the sail is let out.
# 0.0 -> 1.0
# The user should have no control over what side this ends up
# rope_length = state['rope_length']

# 0 -> 4 (User has absolute control)
# sail_size = state['sail_size']

# EVERYTHING is radians
# Clockwise if angle_to_add is positive
# Counter-clockwise if negative


def get_sign(value):
    return 1 if value >= 0 else -1

def lift_and_drag(state):
    
    awa = state['awa']
    aws = state['aws']
    sail_angle = state['relative_sail_angle']
    sail_size = state['sail_size']
    sail_state = state['state']
    
    base_state = sail_state.split(" ")[0]
    
    if base_state == "align":
        return 0.0, 0.0, 0.0, 0.0
    
    if base_state == "dead":
        # Minimal friction drag, zero lift
        return 0.0, 0.0, 0.0, 0.1 * sail_size
    
    AOA_signed = wrap_angle(awa - state['relative_sail_angle'])
    AOA_abs = abs(AOA_signed)
    
    if AOA_abs > math.pi / 2.0:
        AOA_abs = math.pi - AOA_abs
        
    
    opt_aoa = math.radians(18)
    
    if AOA_abs <= opt_aoa:
        # Pre-stall: Smooth linear ramp up of lift
        lift_efficiency = AOA_abs / opt_aoa
        drag_efficiency = 0.15 + 0.35 * (AOA_abs / opt_aoa)
    elif AOA_abs <= math.radians(35):
        # Aerodynamic Stall Zone (18° to 35°): Lift drops rapidly, Drag builds
        stall_factor = (AOA_abs - opt_aoa) / (math.radians(35) - opt_aoa)
        lift_efficiency = 1.0 - (0.65 * stall_factor) # Drops to 35% lift
        drag_efficiency = 0.50 + (0.50 * stall_factor)
    else:
        # Post-stall / Parachute Zone (> 35°): Pure Drag regime
        post_stall_factor = (AOA_abs - math.radians(35)) / (math.pi / 2.0 - math.radians(35))
        lift_efficiency = max(0.05, 0.35 * (1.0 - post_stall_factor))
        drag_efficiency = 1.0 + (0.8 * post_stall_factor) # High blunt-body drag

    # 3. NO-GO / UPWIND PENALTY
    # If Apparent Wind Angle is less than 35 deg, punish efficiency smoothy
    abs_awa = abs(awa)
    if abs_awa < math.radians(35):
        upwind_penalty = abs_awa / math.radians(35) # Smoothly scales 0.0 -> 1.0
        lift_efficiency *= (upwind_penalty ** 2)    # Quadratic punishment for pinching
        drag_efficiency *= (1.5 - 0.5 * upwind_penalty)

    # 4. Calculate Final Raw Forces
    raw_lift = (aws ** 2) * BASE_LIFT_MULTIPLIER * lift_efficiency * sail_size
    raw_drag = (aws ** 2) * BASE_DRAG_MULTIPLIER * drag_efficiency * sail_size

    # 5. Project Vectors onto Hull Space
    drag_direction = awa + math.pi
    force_forward_from_drag = raw_drag * math.cos(drag_direction)
    force_lateral_from_drag = raw_drag * math.sin(drag_direction)

    lift_direction = awa - (get_sign(awa) * (math.pi / 2.0))
    force_forward_from_lift = raw_lift * math.cos(lift_direction)
    force_lateral_from_lift = raw_lift * math.sin(lift_direction)

    force_forward = force_forward_from_lift + force_forward_from_drag
    force_lateral = force_lateral_from_lift + force_lateral_from_drag

    return force_forward, force_lateral, raw_lift, raw_drag


def is_in_slice(target, bound1, bound2, bounds="[]"):
    arc_length = abs(wrap_angle(bound2 - bound1))
    midpoint = add_angle_wrap(bound1, wrap_angle(bound2 - bound1) / 2.0)
    diff = abs(wrap_angle(target - midpoint))
    half_arc = arc_length / 2.0
    epsilon = 0.001

    # 1. Strictly INSIDE the slice
    if diff < (half_arc - epsilon):
        return True

    # 2. Strictly OUTSIDE the slice
    if diff > (half_arc + epsilon):
        return False

    # 3. Sitting EXACTLY ON a boundary (within epsilon tolerance)
    dist_to_b1 = abs(wrap_angle(target - bound1))
    dist_to_b2 = abs(wrap_angle(target - bound2))

    # Check Left Boundary (bound1)
    if dist_to_b1 <= epsilon:
        return bounds[0] == '['

    # Check Right Boundary (bound2)
    if dist_to_b2 <= epsilon:
        return bounds[1] == ']'

    return False


def add_angle_wrap(angle, angle_to_add):
    theta = angle + angle_to_add
    return wrap_angle(theta)


def wrap_angle(angle):
    return (angle + math.pi) % TWO_PI - math.pi

def map_to_clamped_radians(value):
    clamped = max(-1.0, min(1.0, value));
    return clamped * math.pi;


def reflect_north_to_south(angle):
    theta = math.pi - angle
    return wrap_angle(theta)


def reflect_east_to_west(angle):
    return wrap_angle(- angle)


def reflect_both(angle):
    theta = math.pi + angle
    return wrap_angle(theta)

def calculate_global_sail_angle(relative_sail_angle, boat_heading):
    global_angle = relative_sail_angle + boat_heading
    return wrap_angle(global_angle)

# def reverse_relative_sail_angle(relative_sail_angle, boat_heading):
#     global_angle = relative_sail_angle + boat_heading
#     return wrap_angle(global_angle)


def boat_step(current_state: dict, physics_inputs_dict: dict):
    
    # clamped between from math.radians(-135) -> math.radians(135) with centre at -pi
    # rudder_input = physics_inputs_dict['rudder']
    # rope_length_input = physics_inputs_dict['rope_length']
    # rudder_input = physics_inputs_dict['sail_size']

    previous_sail_state = current_state['state']
    state = current_state
    # ENV
    awa = state['awa']
    
    # Wind relative to boat
    wind_side = -1 if awa < 0 else 1
    
    wind_side_text = "left" if wind_side == -1 else "right"
    if abs(awa) == 0:
        wind_side = 0
        wind_side_text = "Head on"
    elif abs(awa) > (math.pi - math.radians(5)):
        wind_side = 0
        wind_side_text = "Dead behind"
        
    # Sail Relative to boat
    sail_side = -1 if state['relative_sail_angle'] < 0 else 1
    sail_side_text = "left" if sail_side == -1 else "right"
    if state['relative_sail_angle'] == -math.pi:
        sail_side = 0
        sail_side_text = "Perfectly aligned with boat"
        
    snap_side = None
    if sail_side == -1:
        snap_side = 1
    elif sail_side == 1:
        snap_side = -1
    elif sail_side == 0:
        snap_side = wind_side

    boom_angle = math.pi - abs(state['relative_sail_angle'])
        
    top_quad_wind = True if abs(awa) <= math.pi / 2 else False
    
    # positive is above sail while sail on the left, negative if above sail, while sail on the right
    AOA_signed = wrap_angle(awa - state['relative_sail_angle'])
    
    # [0 when 0 degress, 1,57 when 90 and 0 again when 0 degress on other side]
    # optimal theta would be math.pi/4 (0.7853) when wing or sail.
    AOA_abs = abs(AOA_signed)
    
    if AOA_abs > math.pi / 2.0:
        AOA_abs = math.pi - AOA_abs
            
    temp_state = "unassigned"
    wind_side_of_sail = 0
    
    good_wind = None
    target_point = None
    
    # Centre Sail
    if abs(state['relative_sail_angle']) == math.pi:
        # Centre wind
        if AOA_signed == -math.pi:
            temp_state = "dead on nothing"
        elif AOA_signed == 0:
            temp_state = "dead behind, straight sail"
        # Wind to the RIGHT
        elif AOA_signed < 0:
            if abs(AOA_signed) <= math.pi/2:
                temp_state = "parachute right"
            else:
                temp_state = "wing right"
        # Wind to the LEFT
        else:
            if abs(AOA_signed) <= math.pi/2:
                temp_state = "parachute left"
            else:
                temp_state = "wing left"
        
    # SAIL LEFT
    # AOA NEGATIVE and Sail LEFT
    elif AOA_signed < 0 and sail_side == -1:
        good_wind = True
        wind_side_of_sail = 1
        if abs(AOA_signed) < math.pi/2:
            temp_state = "parachute left"
        else:
            temp_state = "wing left"
    # AOA POSITIVE and Sail LEFT
    elif (AOA_signed >= 0 or AOA_signed == -math.pi) and sail_side == -1:
        good_wind = False
        temp_state = "align"
    # AOA POSITIVE and Sail RIGHT
    elif (AOA_signed > 0 or AOA_signed == -math.pi) and sail_side == 1:
        good_wind = True
        wind_side_of_sail = -1
        if abs(AOA_signed) < math.pi/2:
            temp_state = "parachute right"
        else:
            temp_state = "wing right"
    # AOA NEGATIVE and Sail RIGHT
    elif AOA_signed <= 0 and sail_side == 1:
        good_wind = False
        temp_state = "align"
    else:
        good_wind = None
        
    state['state'] = temp_state
    
    # State transition
    if previous_sail_state.split(" ")[0] == "parachute" and temp_state.split(" ")[0] == "align":
        state['did_snap'] = True
    
    # ALIGN
    # if the rope length is 0, or the relative sail angle is awa+180, then NO ACTION
    elif temp_state.split(" ")[0] == "align":
        target_point = wrap_angle(awa + math.pi)
        if target_point > 0 and abs(target_point) < math.pi /2:
            target_point = math.pi /2
        if target_point < 0 and abs(target_point) < math.pi /2:
            target_point = -math.pi /2
        travel_dist = abs(wrap_angle(state['relative_sail_angle'] - target_point))
        if travel_dist < ALIGN_STEP_SIZE:
           state['relative_sail_angle'] = target_point
           print(f"Attempting to jump to target point {target_point}")
        else:
            state['relative_sail_angle'] = wrap_angle(state['relative_sail_angle'] + ALIGN_STEP_SIZE * sail_side)
            
    
    # Move the relatvie sail_angle 1 step towards awa
    
    # Parachute or sail
    # If the rope length, added to the centremark is greater that the current sail angle, let it move to max
    if temp_state.split(" ")[0] == "parachute" or temp_state.split(" ")[0] == "wing":
        item = temp_state.split(" ")[0]
        print(f"GOOD CATCH {item}")
        target_side = sail_side if sail_side != 0 else (snap_side * -1)
        target_point = wrap_angle(-math.pi - physics_inputs_dict['rope_length'] * target_side)
        travel_dist = abs(wrap_angle(state['relative_sail_angle'] - target_point))
        
        if state['relative_sail_angle'] == - math.pi:
            dir_swing = wind_side * -1
        elif abs(state['relative_sail_angle']) - abs(target_point) > 0:
            # needs to move UP
            dir_swing = sail_side
        else:
            # needs to move DOWN
            dir_swing = snap_side
        
        if travel_dist < WING_PARA_STEP_SIZE:
           state['relative_sail_angle'] = target_point
           print("Attempting to jump to full sail")
        else:
           state['relative_sail_angle'] = wrap_angle(state['relative_sail_angle'] - WING_PARA_STEP_SIZE * dir_swing)
           print(f"Attempting increment towards rope length max set at {target_point}")
    
    print(f"wind_side_of_sail is {wind_side_of_sail}")
    print(f"top_quad_wind is {top_quad_wind}")
    print(f"wind_side is {wind_side_text}")
    print(f"sail_side is {sail_side_text}")
    print(f"AOA_abs is {AOA_abs}")
    print(f"boom_angle is {boom_angle}")
    print(f"good_wind is {good_wind}")
    print(f"awa is {awa}")
    print(f"AOA_signed is {AOA_signed}")
    print(f"did_snap is {state['did_snap']}")
    
    condition = None
    
    print(f"FLAP_ZONE is {FLAP_ZONE}")
    print(f"temp_state is {temp_state}")
    print(f'CONDITION: {condition}')
    
    # Determine how to handle GOOD WIND
    force_forward, force_lateral, raw_lift, raw_drag = lift_and_drag(state)
    
    print(f"force_forward is {force_forward}")
    print(f"force_lateral is {force_lateral}")
    print(f'raw_lift: {raw_lift}')
    print(f'raw_drag: {raw_drag}')
    
    raw_hull_wind_drag = (state['aws'] ** 2) * DRAG_HULL_WIND
    wind_push_dir = wrap_angle(state['awa'] + math.pi)
    
    force_hull_wind_forward = raw_hull_wind_drag * math.cos(wind_push_dir)
    force_hull_wind_lateral = raw_hull_wind_drag * math.sin(wind_push_dir) * 0.1

    # 2. Water Resistance Forces
    if state['velocity_forward'] < 0:
        force_drag_hull = (state['velocity_forward'] * abs(state['velocity_forward'])) * (DRAG_HULL * 30)
    else:
        force_drag_hull = (state['velocity_forward'] * abs(state['velocity_forward'])) * DRAG_HULL
    
    # Keel drag becomes LINEAR (instantly resists small lateral slips)
    force_drag_keel = state['velocity_lateral_drift'] * DRAG_KEEL

    # 3. Combined Net Forces (Sail + Wind on Hull - Water Resistance)
    force_net_forward = (force_forward + force_hull_wind_forward) - force_drag_hull
    force_net_lateral = (force_lateral + force_hull_wind_lateral) - force_drag_keel

    acceleration_forward = force_net_forward / MASS_BOAT
    acceleration_lateral = force_net_lateral / ( MASS_BOAT * LATERAL_LIMITER)

    # 4. Rotational Dynamics (Weather Helm & Rudder)
    direction_weather_helm = get_sign(state['awa'])
    speed_dampening = 1.0 / (1.0 + (abs(state['velocity_forward']) * 0.15))
    effective_offset = OFFSET_CE_CLR * speed_dampening

    # Torque 1: The lateral wind force pushing sideways against the mast
    torque_spine = force_lateral * effective_offset
    torque_side = force_forward * -math.sin(state['relative_sail_angle']) * state['sail_size']
    torque_weather_helm = torque_spine + torque_side

    velocity_forward = state.get('velocity_forward', 0.0)
    velocity_lateral_drift = state.get('velocity_lateral_drift', 0.0)
    current_rot_speed = state.get('velocity_boat_rotation', 0.0)
    
    rudder_angle = physics_inputs_dict.get('rudder', 0.0)
    speed_factor = state['velocity_forward'] 
    
    if velocity_forward < 0:
        speed_factor = velocity_forward * 3.0
    else:
        speed_factor = velocity_forward
    
    torque_rudder = math.sin(rudder_angle) * RUDDER_POWER * speed_factor
    torque_drag_rotation = ((current_rot_speed * abs(current_rot_speed)) * DRAG_ROTATION) / 2
    
    # Calculate net turning force
    force_net_turning = torque_rudder - torque_drag_rotation + (torque_weather_helm * 1.2)
    acceleration_rotational = (force_net_turning / INERTIA_BOAT)
    
    # 5. Integrate Velocities
    velocity_boat_rotation = (current_rot_speed / 2) + (acceleration_rotational * DELTA_TIME)
    velocity_lateral_drift += acceleration_lateral * DELTA_TIME
    velocity_forward += acceleration_forward * DELTA_TIME

    # 6. Save State
    state['velocity_boat_rotation'] = velocity_boat_rotation
    state['velocity_lateral_drift'] = velocity_lateral_drift
    state['velocity_forward'] = velocity_forward

    return state
