import math

from typing import Optional, List

from rlbot.utils.structures.game_data_struct import PlayerInfo
from rlbot.agents.base_agent import SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from util.boost_pad_tracker import BoostPadTracker

from util.orientation import Orientation, relative_location
from util.sequence import Sequence, ControlStep
from util.vec import Vec3


def limit_to_safe_range(value: float) -> float:
    """
    Controls like throttle, steer, pitch, yaw, and roll need to be in the range of -1 to 1.
    This will ensure your number is in that range. Something like 0.45 will stay as it is,
    but a value of -5.6 would be changed to -1.
    """
    if value < -1:
        return -1
    if value > 1:
        return 1
    return value


def steer_toward_target(car: PlayerInfo, target: Vec3) -> float:
    angle = angle_toward_target(car, target)
    return limit_to_safe_range(angle * 5)

def angle_toward_target(car: PlayerInfo, target: Vec3) -> float:
    relative = relative_location(Vec3(car.physics.location), Orientation(car.physics.rotation), target)
    angle = math.atan2(relative.y, relative.x)
    return angle

def create_front_flip_sequence():
    return Sequence([
        ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
        ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
        ControlStep(duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)),
        ControlStep(duration=0.8, controls=SimpleControllerState()),
    ])

def land_on_wheels_from_air(car: PlayerInfo):
    roll = 0.0

    if car.has_wheel_contact is False:
        if car.physics.rotation.roll >= 0.01:
            roll = -1.0
        elif car.physics.rotation.roll <= 0.01:
            roll = 1.0

    return roll

def check_for_close_boost_pad(
        car: PlayerInfo,
        car_location: Vec3,
        allowable_distance: float,
        allowable_angle: float,
        boost_pad_tracker: BoostPadTracker) -> Optional[Vec3]:

    target_location = None

    for boost_pad in boost_pad_tracker.get_boost_pads():
        if boost_pad.is_active:
            relative_distance = car_location.dist(boost_pad.location)
            relative_angle = angle_toward_target(car, boost_pad.location)

            if boost_pad.is_full_boost:
                if relative_distance <= allowable_distance and abs(relative_angle) <= allowable_angle:
                    target_location = boost_pad.location
            else:
                if relative_distance <= allowable_distance and abs(relative_angle) <= allowable_angle:
                    target_location = boost_pad.location
    
    return target_location


def check_for_close_foe(
        car: PlayerInfo,
        car_location: Vec3,
        allowable_distance: float,
        allowable_angle: float,
        foes: List[PlayerInfo]) -> Optional[Vec3]:

    closest_distance = None

    target_location = None
    foe_name = None

    for foe in foes:
        if foe.is_demolished is False:
            foe_location = Vec3(foe.physics.location)

            relative_distance = car_location.dist(foe_location)
            relative_angle = angle_toward_target(car, foe_location)

            if relative_distance <= allowable_distance and abs(relative_angle) <= allowable_angle:
                if closest_distance is None or closest_distance > relative_distance:
                    closest_distance = relative_distance
                    target_location = foe_location
                    foe_name = foe.name
    
    return target_location, foe_name