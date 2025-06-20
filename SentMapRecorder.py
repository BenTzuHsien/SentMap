import os,sys, termios, tty, select, time
from SpotStack import GraphRecorder, ImageFetcher, MotionController, ArmBase

from bosdyn.api import geometry_pb2
from bosdyn.client.math_helpers import Quat

def key_pressed():
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        return dr != []
def get_key():
    return sys.stdin.read(1) if key_pressed() else None

class SentMapRecorder:
    ARM_IMAGE_POSE = geometry_pb2.SE3Pose(position=geometry_pb2.Vec3(x=0.6, y=0.0, z=0.65), rotation=Quat.from_pitch(0.5).to_proto())
    KEYBOARD_LOOKUP = {
        'w': (1.0, 0.0, 0.0),   # forward in x
        's': (-1.0, 0.0, 0.0),  # backward in x
        'a': (0.0, 1.0, 0.0),   # left in y
        'd': (0.0, -1.0, 0.0),  # right in y
        'q': (0.0, 0.0, 1.0),   # rotate left (yaw)
        'e': (0.0, 0.0, -1.0)   # rotate right (yaw)
    }

    def __init__(self, robot, map_path, is_new_map=True, image_transform=None):

        self._map_path = map_path
        graph_path = os.path.join(map_path, 'spot_graph')
        if not os.path.exists(graph_path):
            os.mkdir(graph_path)

        self._graph_recorder = GraphRecorder(robot, graph_path, is_new_map)
        self._image_fetcher = ImageFetcher(robot, use_front_stitching=True)
        self._motion_controller = MotionController(robot)
        self._arm_base = ArmBase(robot)

        self._graph_recorder.start_recording()
        self._image_transform = image_transform

    def create_waypoint(self, name):

        waypoint_dir = os.path.join(self._map_path, f'{name}')
        if not os.path.exists(waypoint_dir):
            os.mkdir(waypoint_dir)
        
        self._graph_recorder.record_waypoint(name)
        current_images = self._image_fetcher.get_images(self._image_transform)

        image_path = os.path.join(waypoint_dir, 'front_image.jpg')
        current_images[0].save(image_path)

        # Move arm
        self._arm_base.move_to_pose(self.ARM_IMAGE_POSE, 1.0)
        time.sleep(1)
        gripper_image = self._arm_base.get_image(self._image_transform)
        image_path = os.path.join(waypoint_dir, 'gripper_image.jpg')
        gripper_image.save(image_path)

        self._arm_base.rest_arm()

    def run(self):

        # Setup input mode
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        try:
            waypoint_index = 0
            while True:
                print(f"Go to Waypoint_{waypoint_index} using wasdqe to control. When finished, type k, to end type x:")
                key = get_key()

                if key == 'x':
                    break

                elif key == 'k':
                    self._motion_controller.send_velocity_command(0, 0, 0, 1)
                    time.sleep(1)

                    self.create_waypoint(f'Waypoint_{waypoint_index}')
                    waypoint_index += 1

                else:
                    action = self.KEYBOARD_LOOKUP.get(key, (0, 0, 0))
                    self._motion_controller.send_velocity_command(action[0], action[1], action[2], 0.5)
                    time.sleep(0.5)
            
            self._graph_recorder.stop_recording()
            self._graph_recorder.download_full_graph()
            print("Graph is downloaded !")
            self._motion_controller.on_quit()

        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, old_settings)

        return True
    
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
                sentmap_recorder.run()

            except Exception as exc:  # pylint: disable=broad-except
                print("SentMapRecorder threw an error.")
                print(exc)
    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )