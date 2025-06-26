import os,sys, time, curses
from SpotStack import GraphRecorder, ImageFetcher, MotionController, ArmCore

from bosdyn.client.math_helpers import Quat, SE3Pose

class SentMapRecorder:
    ARM_IMAGE_POSE = SE3Pose(x=0.6, y=0.0, z=0.45, rot=Quat.from_pitch(1.2))
    KEYBOARD_LOOKUP = {
        'w': (0.5, 0.0, 0.0),   # forward in x
        's': (-0.5, 0.0, 0.0),  # backward in x
        'a': (0.0, 0.5, 0.0),   # left in y
        'd': (0.0, -0.5, 0.0),  # right in y
        'q': (0.0, 0.0, 0.5),   # rotate left (yaw)
        'e': (0.0, 0.0, -0.5)   # rotate right (yaw)
    }

    def __init__(self, robot, map_path, is_new_map=True, image_transform=None):

        self._map_path = map_path
        graph_path = os.path.join(map_path, 'spot_graph')
        if not os.path.exists(graph_path):
            os.mkdir(graph_path)

        self._graph_recorder = GraphRecorder(robot, graph_path, is_new_map)
        self._image_fetcher = ImageFetcher(robot, use_front_stitching=True)
        self._motion_controller = MotionController(robot)
        self._arm_core = ArmCore(robot)

        self._graph_recorder.start_recording()
        self._image_transform = image_transform

        self._stdscr = None

    def on_quit(self):
        self._motion_controller.on_quit()

    def create_waypoint(self, name):

        waypoint_dir = os.path.join(self._map_path, f'{name}')
        if not os.path.exists(waypoint_dir):
            os.mkdir(waypoint_dir)

        self._log(f'Start Recording Waypoint {name} ...')
        
        self._graph_recorder.record_waypoint(name)
        current_images = self._image_fetcher.get_images(self._image_transform)

        image_path = os.path.join(waypoint_dir, 'front_image.jpg')
        current_images[0].save(image_path)

        # Move arm
        self._arm_core.move_to_pose(self.ARM_IMAGE_POSE, 1.0)
        time.sleep(1)
        gripper_image = self._arm_core.get_image(self._image_transform)
        image_path = os.path.join(waypoint_dir, 'gripper_image.jpg')
        gripper_image.save(image_path)

        self._arm_core.rest_arm()
        self._log(f'Waypoint {name} Recorded !')
    
    def _flush_input(self):
        """Flush the input buffer by calling getch until empty."""
        self._stdscr.nodelay(True)  # Ensure non-blocking mode
        while self._stdscr.getch() != -1:
            pass

    def _log(self, msg):
        self._stdscr.clear()
        self._stdscr.addstr(0, 0, msg)
        self._stdscr.refresh()

    def drive(self, stdscr):

        self._stdscr = stdscr
        curses.curs_set(0)
        stdscr.nodelay(True)

        try:
            sys.stdout = open(os.devnull, 'w')

            waypoint_index = 0
            self._log(f"Go to Waypoint_{waypoint_index} using wasdqe to control. When finished, type k, to end type x:")
            self._flush_input()

            is_finished = False
            while not is_finished:
                key = stdscr.getch()

                if key == -1:
                    continue

                elif key == ord('x'):
                    is_finished = True
                    self._log('Mapping Complete !')

                elif key == ord('k'):
                    self._motion_controller.send_velocity_command(0, 0, 0, 1)
                    time.sleep(1)

                    self.create_waypoint(f'Waypoint_{waypoint_index}')
                    waypoint_index += 1
                
                else:
                    try:
                        action = self.KEYBOARD_LOOKUP.get(chr(key), (0, 0, 0))
                        self._motion_controller.send_velocity_command(action[0], action[1], action[2], 0.6)
                        self._flush_input()

                    except ValueError:
                        continue
                
                time.sleep(0.1)

            self._graph_recorder.stop_recording()
            self._graph_recorder.download_full_graph()
            self._log("Graph is downloaded !")

        finally:
            sys.stdout = sys.__stdout__

# Example Usage
if __name__ == '__main__':

    import argparse, bosdyn.client.util, sys
    from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
    
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('--map-path',
                        help='Full filepath for SentMap directory.',
                        default=os.getcwd())
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('SentMapRecorder')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    lease_client = robot.ensure_client(LeaseClient.default_service_name)

    if not os.path.exists(options.map_path):
        os.mkdir(options.map_path)
    
    try:
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            try:
                sentmap_recorder = SentMapRecorder(robot, options.map_path)
                curses.wrapper(sentmap_recorder.drive)

            except Exception as exc:  # pylint: disable=broad-except
                print("SentMapRecorder threw an error.")
                print(exc)
            finally:
                sentmap_recorder.on_quit()
                
    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )