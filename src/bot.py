from typing import Optional

from enum import Enum

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.drive import angle_toward_target, steer_toward_target, check_for_close_boost_pad, check_for_close_foe, \
        land_on_wheels_from_air, create_front_flip_sequence
from util.sequence import Sequence
from util.vec import Vec3


class TargetStrategyState(Enum):
    GoForBall = 1
    GoForBoost = 2
    GoForBump = 3
    GoAlignBallHit = 4
    GoBackToQuickGoalSide = 5
    GoBackToFarPost = 6
    GoBackToNearPost = 7
    GoBackShadowDefense = 8


class MyBot(BaseAgent):

    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()

        self.seconds_elapsed = 0.0

        self.front_flip_timeout = 1.5 #seconds
        self.front_flip_last_timer = 0.0

        self.back_to_goal_dist_hyst = 0.0

        self.target_strategy = None

        self.kickoff_mode_enabled = False


    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def begin_sequence(self, packet: GameTickPacket, sequence: Sequence) -> Optional[SimpleControllerState]:
        # Send some quickchat just for fun
        self.send_quick_chat(team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)

        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = sequence

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)


    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """

        dt = packet.game_info.seconds_elapsed - self.seconds_elapsed
        self.seconds_elapsed = packet.game_info.seconds_elapsed

        if packet.game_info.is_kickoff_pause:
            self.kickoff_mode_enabled = True

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        # This is good to keep at the beginning of get_output. It will allow you to continue
        # any sequences that you may have started during a previous call to get_output.
        if self.active_sequence is not None and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            if controls is not None:
                return controls

        # Otherwise, create a single controls input state...
        # but first...
        # Gather some information about our car and the ball
        my_car = packet.game_cars[self.index]
        my_team = my_car.team

        if my_team == 0:
            opp_team = 1
        else:
            opp_team = 0

        car_location = Vec3(my_car.physics.location)
        car_velocity = Vec3(my_car.physics.velocity)
        ball_location = Vec3(packet.game_ball.physics.location)

        field_info = self.get_field_info()


        # By default we will chase the ball, but target_location can be changed later
        target_location = ball_location
        self.target_strategy = TargetStrategyState.GoForBall

        # Alternate Target Selections:

        if self.kickoff_mode_enabled is False:

            # Check if available boost is nearby to grab
            if my_car.boost <= 50.0:
                target_boost_location = check_for_close_boost_pad(
                    my_car, car_location, 
                    1500.0, 0.3,
                    self.boost_pad_tracker)
                if target_boost_location is not None:
                    target_location = target_boost_location
                    self.target_strategy = TargetStrategyState.GoForBoost

                    
            target_bump_location, target_foe_name = check_for_close_foe(
                my_car, car_location, 
                1500, 0.3,
                [foe for foe in packet.game_cars if foe.team != my_team and foe.name != ''])
            if target_bump_location is not None:
                target_location = target_bump_location
                self.target_strategy = TargetStrategyState.GoForBump

            goal_attack_location = Vec3(field_info.goals[opp_team].location)
            goal_defend_location = Vec3(field_info.goals[my_team].location)

            goal_angle_attack = angle_toward_target(my_car, goal_attack_location)
            goal_angle_defend = angle_toward_target(my_car, goal_defend_location)

            if abs(car_location.y - goal_defend_location.y) - abs(ball_location.y - goal_defend_location.y) >= self.back_to_goal_dist_hyst:
                target_location = goal_defend_location
                self.target_strategy = TargetStrategyState.GoBackToQuickGoalSide
                self.back_to_goal_dist_hyst = 0.0

        elif ball_location.y != 0:
            self.kickoff_mode_enabled = False


        # Control to Target
        target_angle = angle_toward_target(my_car, target_location)
        target_distance = car_location.dist(target_location)

        # Creating a single controls input state:
        controls = SimpleControllerState()
        controls.steer = steer_toward_target(my_car, target_location)
        controls.throttle = 1.0

        controls.roll = land_on_wheels_from_air(my_car)


        if self.front_flip_last_timer > 0.0:
            self.front_flip_last_timer -= dt


        ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc
        ball_in_future = find_slice_at_time(ball_prediction, packet.game_info.seconds_elapsed + 2)


        if car_velocity.length() <= 1500.0 and target_angle > 0.4:
            controls.handbrake = True


        if car_velocity.length() <= 2500.0:
            if self.front_flip_last_timer <= 0.0 and abs(target_angle) <= 0.2:
                controls.boost = True
            
            if car_velocity.length() >= 1300.0 and target_distance >= 3000 and abs(target_angle) <= 0.2:
                if self.front_flip_last_timer <= 0.0:
                    self.front_flip_last_timer = self.front_flip_timeout
                    return self.begin_sequence(packet, create_front_flip_sequence())

            # ball_in_future might be None if we don't have an adequate ball prediction right now, like during
            # replays, so check it to avoid errors.
            if self.target_strategy is TargetStrategyState.GoForBall and ball_in_future is not None:
                target_location = Vec3(ball_in_future.physics.location)
                self.renderer.draw_line_3d(ball_location, target_location, self.renderer.cyan())
        


        # Draw some things to help understand what the bot is thinking
        self.renderer.draw_line_3d(car_location, target_location, self.renderer.white())
        self.renderer.draw_string_3d(car_location, 1, 1, f'Speed: {car_velocity.length():.1f}', self.renderer.white())
        self.renderer.draw_string_3d(Vec3(car_location.x, car_location.y, car_location.z - 100.0), 1, 1, f'Boost: {my_car.boost:.1f}', self.renderer.white())
        self.renderer.draw_string_3d(Vec3(car_location.x, car_location.y, car_location.z - 200.0), 1, 1, f'T-Dist: {target_distance:.1f}', self.renderer.white())
        self.renderer.draw_string_3d(Vec3(car_location.x, car_location.y, car_location.z - 300.0), 1, 1, f'T-Angle: {target_angle:.1f}', self.renderer.white())
        self.renderer.draw_string_3d(Vec3(car_location.x, car_location.y, car_location.z - 400.0), 1, 1, f'T_Strat: {self.target_strategy}', self.renderer.white())
        self.renderer.draw_string_3d(Vec3(car_location.x, car_location.y, car_location.z - 500.0), 1, 1, f'X: {car_location.x:.1f}', self.renderer.white())
        self.renderer.draw_string_3d(Vec3(car_location.x, car_location.y, car_location.z - 600.0), 1, 1, f'Y: {car_location.y:.1f}', self.renderer.white())
        self.renderer.draw_rect_3d(target_location, 8, 8, True, self.renderer.cyan(), centered=True)

        return controls