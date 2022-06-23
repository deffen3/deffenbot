from typing import Optional

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.drive import angle_toward_target, steer_toward_target
from util.sequence import Sequence, ControlStep
from util.vec import Vec3


class MyBot(BaseAgent):

    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()

        self.seconds_elapsed = 0.0

        self.front_flip_timeout = 1.5 #seconds
        self.front_flip_last_timer = 0.0

    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """

        dt = packet.game_info.seconds_elapsed - self.seconds_elapsed
        self.seconds_elapsed = packet.game_info.seconds_elapsed

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
        car_location = Vec3(my_car.physics.location)
        car_velocity = Vec3(my_car.physics.velocity)
        ball_location = Vec3(packet.game_ball.physics.location)

        field_info = self.get_field_info()


        # By default we will chase the ball, but target_location can be changed later
        target_location = ball_location

        # Check if available boost is nearby to grab
        if my_car.boost <= 70.0:
            target_boost_location = check_for_close_boost_pad(my_car, car_location)
            if target_boost_location is not None:
                target_location = target_boost_location

        target_angle = angle_toward_target(my_car, target_location)
        target_distance = car_location.dist(target_location)

        # Creating a single controls input state:
        controls = SimpleControllerState()
        controls.steer = steer_toward_target(my_car, target_location)
        controls.throttle = 1.0

        controls.roll = land_on_wheels_from_air(mycar)


        if self.front_flip_last_timer > 0.0:
            self.front_flip_last_timer -= dt

        if target_distance > 1000:
            # We're far away from the ball, let's try to lead it a little bit
            ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc
            ball_in_future = find_slice_at_time(ball_prediction, packet.game_info.seconds_elapsed + 2)

            if car_velocity.length() <= 1700:
                if abs(target_angle) <= 0.2:
                    # The ball is far and relatively straight ahead
                    if my_car.boost >= 10.0:
                        controls.boost = True

                    elif self.front_flip_last_timer <= 0.0:
                        self.front_flip_last_timer = self.front_flip_timeout
                        return self.begin_front_flip(packet)

            # ball_in_future might be None if we don't have an adequate ball prediction right now, like during
            # replays, so check it to avoid errors.
            if ball_in_future is not None:
                target_location = Vec3(ball_in_future.physics.location)
                self.renderer.draw_line_3d(ball_location, target_location, self.renderer.cyan())

        # Draw some things to help understand what the bot is thinking
        self.renderer.draw_line_3d(car_location, target_location, self.renderer.white())
        self.renderer.draw_string_3d(car_location, 1, 1, f'Speed: {car_velocity.length():.1f}', self.renderer.white())
        self.renderer.draw_rect_3d(target_location, 8, 8, True, self.renderer.cyan(), centered=True)

        return controls

    def begin_front_flip(self, packet):
        # Send some quickchat just for fun
        self.send_quick_chat(team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)

        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = Sequence([
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
            ControlStep(duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)),
            ControlStep(duration=0.8, controls=SimpleControllerState()),
        ])

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)

    def land_on_wheels_from_air(self, car: PlayerInfo):
        roll = 0.0

        if car.has_wheel_contact is False:
            if car.physics.rotation.roll >= 0.01:
                roll = -1.0
            elif car.physics.rotation.roll <= 0.01:
                roll = 1.0

        return roll

    def check_for_close_boost_pad(self, car: PlayerInfo, car_location: Vec3) -> Optional[Vec3]:
        target_location = None

        for boost_pad in self.boost_pad_tracker.boost_pads:
            if boost_pad.is_active:
                relative_distance = car_location.dist(boost_pad.location)
                relative_angle = angle_toward_target(my_car, boost_pad.location)

                if boost_pad.is_full_boost:
                    if relative_distance <= 500 and abs(relative_angle) <= 0.5:
                        target_location = boost_pad.location
                else:
                    if relative_distance <= 500 and abs(relative_angle) <= 0.5:
                        target_location = boost_pad.location
        
        return target_location