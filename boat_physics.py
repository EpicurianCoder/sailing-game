import math
from boat_physics_helpers import lift_and_drag, get_sign
import random

DRAG_HULL = 2
DRAG_KEEL = 250.0
DRAG_ROTATION = 1800.0
SAIL_SWING_SPEED = 2.0
RUDDER_POWER = 5000.0
RUDDER_MAX = math.radians(45)
MASS_BOAT = 200
INERTIA_BOAT = 1400
OFFSET_CE_CLR = 0.01
DELTA_TIME = 1.0 / 60.0
MAX_SAIL_ANGLE = 1.57
SAIL_SWING_STEP = math.radians(10)
THRESHOLD_15_DEG = math.radians(5)
DEGREE_TO_RAD = math.pi / 180
TWO_PI = math.pi * 2
FLAP_ZONE = math.radians(5)
SAIL_MOVE_INCREMENT = math.radians(10)

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
    # aws - FIXED

    # BOAT SPEEDS
    # speed_boat_rotation = state['speed_boat_rotation']
    # velocity_lateral_drift = state['velocity_lateral_drift']
    # velocity_forward = state['velocity_forward']
    
    # boat_heading = state['boat_heading']
    # sail_angle = state['sail_angle']

    # # BOAT SETTINGS
    # sail_size = state['sail_size']

    # SAIL SNAP LOCKOUT VALUES
    # this is the length of the theoretical rope that holds the sail boom
    # lockout_target_angle = state['lockout_target_angle']

    # # This is the boolean that prevents sail control until lockout_target_angle is reached
    # out_of_control = state['out_of_control']
    
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
    AOA_abs = abs(abs(awa) - abs(state['relative_sail_angle']))
            
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
        if travel_dist < SAIL_MOVE_INCREMENT:
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
        
        if travel_dist < SAIL_MOVE_INCREMENT:
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
    
    # temp_state = "unassigned"
    
    # if good_wind == True:
    #     if AOA_abs >= math.pi / 2:
    #         temp_state = "wing"
    #     elif AOA_abs > math.pi:
    #         temp_state = "error"
    #         temp_state = "parachute"
    # elif good_wind == False:
    #     condition = (abs(awa) + boom_angle)
    #     if abs(AOA_signed) <= math.pi and condition < FLAP_ZONE:
    #         temp_state = "align"
    #     else:
    #         temp_state = "snap"
    
    print(f"FLAP_ZONE is {FLAP_ZONE}")
    print(f"temp_state is {temp_state}")
    print(f'CONDITION: {condition}')
    
    
    
    # print(f"rope_length_angle is {rope_length_angle}")
    
    # temp_state = "unassigned"

    
    # if abs(awa) < math.radians(5) and AOA_abs >= abs(relative_sail_angle):
    #     temp_state = "flappy"
    #     random_number = random.randint(1, 5)
    #     relative_sail_angle = wrap_angle(-math.pi + math.radians(random_number) * snap_side)
    # elif good_wind == True:
    #     if AOA_abs >= math.pi / 2:
    #         temp_state = "wing"
    #     elif AOA_abs > math.pi:
    #         temp_state = "error"
    #     elif AOA_abs <= math.pi / 2 and AOA_abs >= 0:
    #         temp_state = "parachute"
    # elif good_wind == False:
    #     temp_state = "align"
    #     out_of_control = 1
            
    # if out_of_control < 0:
    #     if abs(relative_sail_angle) != abs(rope_length_angle):
    #         relative_sail_angle = rope_length_angle
        
    
    # # sail_impact = False if (temp_state == "align" or temp_state == "snap") else True

    # current_zone = temp_state

    # # snap: 1, align: 2
    # if out_of_control > 0:
    #     rope_stop = (math.pi - rope_length_angle) * sail_side
    #     limit = add_angle_wrap(awa, math.pi)
    #     if abs(limit) > abs(rope_stop):
    #         relative_sail_angle = rope_stop
    #     else:
    #         relative_sail_angle = add_angle_wrap(awa, math.pi)
            
    #     # shortest angular distance to the limit
    #     diff = wrap_angle(limit - relative_sail_angle)
    #     swing_amount = SAIL_SWING_STEP * (DELTA_TIME * 5)

    # # General Sail Forces
    # force_forward, force_lateral, raw_lift, raw_drag = lift_and_drag(awa, aws, relative_sail_angle, sail_size)

    # # Calculate the turning forces of the wind
    # direction_weather_helm = get_sign(awa)  # either 1 or -1 (it is the sign of the AWA)

    # # Lower the effective lever arm offset as forward speed builds
    # speed_dampening = 1.0 / (1.0 + (abs(velocity_forward) * 0.15))
    # effective_offset = OFFSET_CE_CLR * speed_dampening

    # torque_spine = (force_lateral * effective_offset)
    # torque_side = force_forward * (sail_size * math.sin(relative_sail_angle))
    # torque_weather_helm = (torque_spine + torque_side) * direction_weather_helm * 0.2

    # speed_factor = max(0.0, abs(velocity_forward))
    # torque_rudder_ideal = rudder_continuous * RUDDER_POWER * speed_factor  # 0.5 * 0.8 = 0.4

    # rudder_adjustment = 0.0

    # if (abs(torque_weather_helm) > abs(torque_rudder_ideal)) and torque_weather_helm * torque_rudder_ideal < 0:
    #     excess_torque = abs(torque_weather_helm) - abs(torque_rudder_ideal)
    #     pushback_direction = get_sign(torque_weather_helm)
    #     rudder_adjustment = (excess_torque / RUDDER_POWER) * pushback_direction * DELTA_TIME

    # rudder_continuous = max(-1.0, min(rudder_continuous + rudder_adjustment, 1.0))

    # torque_rudder = rudder_continuous * RUDDER_POWER * speed_factor
    # force_net_turning = torque_weather_helm + torque_rudder  # 0.4 - 0.1 = 0.3

    # # NOT MINE
    # torque_drag_rotation = (speed_boat_rotation * abs(speed_boat_rotation)) * DRAG_ROTATION
    # force_net_turning -= torque_drag_rotation

    # acceleration_rotational = (force_net_turning / INERTIA_BOAT)
    # speed_boat_rotation = speed_boat_rotation + (acceleration_rotational * DELTA_TIME)

    # # Calculate Sideways Drift (Acceleration)
    # force_drag_keel = (velocity_lateral_drift * abs(velocity_lateral_drift)) * DRAG_KEEL
    # force_net_keel = force_lateral - force_drag_keel
    # acceleration_lateral = force_net_keel / MASS_BOAT

    # # Calculate Forward Movement (Acceleration)
    # force_drag_hull = (velocity_forward * abs(velocity_forward)) * DRAG_HULL
    # force_net_hull = force_forward - force_drag_hull
    # acceleration_forward = force_net_hull / MASS_BOAT

    # # Update the velocity_forward and velocity_lateral
    # velocity_lateral_drift += acceleration_lateral * DELTA_TIME
    # velocity_forward += acceleration_forward * DELTA_TIME


    # BOAT SPEEDS
    # state['speed_boat_rotation'] = speed_boat_rotation
    # state['velocity_lateral_drift'] = velocity_lateral_drift
    # state['velocity_forward'] = velocity_forward
    
    # # ensure sail is never in north quadrant
    # if relative_sail_angle < 0 and relative_sail_angle > -math.pi / 2:
    #     relative_sail_angle = -math.pi / 2
    # elif relative_sail_angle > 0 and relative_sail_angle < math.pi / 2:
    #     relative_sail_angle = math.pi / 2
    # elif relative_sail_angle == 0:
    #     relative_sail_angle = math.pi
        

    # # BOAT SETTINGS
    # state['relative_sail_angle'] = relative_sail_angle

    # # SAIL SNAP LOCKOUT VALUES
    # state['lockout_target_angle'] = lockout_target_angle
    # state['out_of_control'] = out_of_control
    # state['state'] = current_zone
    
    # DO NOT RETURN CONTROL STATES UNTIL RUDDER IS FORCED STRAIGHT

    return state
