import os, time, cv2, numpy
from SpotStack import GraphNavigator, ArmGrasper, MotionController

from bosdyn.client.math_helpers import Quat, SE3Pose

class SentMapNavigator:
    PRE_GRASP_POSE = SE3Pose(x=0.6, y=0.0, z=0.45, rot=Quat.from_pitch(1.2))
    KEYBOARD_LOOKUP = {
        'w': (0.5, 0.0, 0.0),   # forward in x
        's': (-0.5, 0.0, 0.0),  # backward in x
        'a': (0.0, 0.5, 0.0),   # left in y
        'd': (0.0, -0.5, 0.0),  # right in y
        'q': (0.0, 0.0, 0.5),   # rotate left (yaw)
        'e': (0.0, 0.0, -0.5)   # rotate right (yaw)
    }

    def __init__(self, robot, map_path):
        
        self._map_path = map_path
        graph_path = os.path.join(map_path, 'spot_graph')

        self._graph_navigator = GraphNavigator(robot, graph_path)
        self._motion_controller = MotionController(robot)
        self._arm_grasper = ArmGrasper(robot)

    def on_quit(self):
        self._graph_navigator.on_quit()

    def go_to(self, waypoint_name):

        print(f'Going to {waypoint_name}')
        self._graph_navigator.navigate_to(f'{waypoint_name}')

    @staticmethod
    def pil_to_cv(image_pil):
        image_np = numpy.array(image_pil)
        return cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
    
    @staticmethod
    def cv_mouse_callback(event, x, y, flags, param):
        clone = param['image'].copy()
        if event == cv2.EVENT_LBUTTONUP:
            param['click'] = (x, y)
        else:
            # Draw auxiliary lines on the image.
            color = (30, 30, 30)
            thickness = 2
            image_title = 'Click to grasp'
            height = clone.shape[0]
            width = clone.shape[1]
            cv2.line(clone, (0, y), (width, y), color, thickness)
            cv2.line(clone, (x, 0), (x, height), color, thickness)
            cv2.imshow(image_title, clone)
    
    def pick(self, constraint):

        # Pre grasp
        self._arm_grasper.move_to_pose(self.PRE_GRASP_POSE, 1)
        time.sleep(2)
                
        image, image_response = self._arm_grasper.get_image_and_response(data_transform=self.pil_to_cv)

        # Click to grasp
        image_title = 'Click to grasp'
        param = {'image': image, 'click': None}
        cv2.namedWindow(image_title)
        cv2.setMouseCallback(image_title, self.cv_mouse_callback, param)
        cv2.imshow(image_title, image)

        while param['click'] is None:
            image, image_response = self._arm_grasper.get_image_and_response(data_transform=self.pil_to_cv)
            param['image'] = image
            cv2.imshow(image_title, image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('x'):
                self._motion_controller.send_velocity_command(0, 0, 0, 0.6)
                print('"x" pressed, exiting.')
                exit(0)
            
            try:
                action = self.KEYBOARD_LOOKUP.get(chr(key), (0, 0, 0))
                self._motion_controller.send_velocity_command(action[0], action[1], action[2], 0.6)

            except ValueError:
                continue
                
        cv2.destroyWindow(image_title)

        # Grasping
        if not self._arm_grasper.grasp(param['click'], image_response, constraint):
            raise RuntimeError("Grasp failed")
        
        # Carrying
        carrying_pose = SE3Pose(x=0.2, y=0.0, z=0.45, rot=Quat.from_pitch(1.2))
        self._arm_grasper.move_to_pose(carrying_pose, 0)
        time.sleep(1)

    def place(self):
        
        high_pose = SE3Pose(x=0.6, y=0.0, z=0.6, rot=Quat.from_pitch(1.2))
        drop_pose = SE3Pose(x=0.9, y=0.0, z=0.6, rot=Quat.from_pitch(1.2))
        
        # Lift High
        self._arm_grasper.move_to_pose(high_pose)
        time.sleep(1)

        # Move forward
        self._arm_grasper.move_to_pose(drop_pose)
        time.sleep(1)

        # Drop
        self._arm_grasper.move_to_pose(drop_pose, 1)
        time.sleep(1)

        # Move backward
        self._arm_grasper.move_to_pose(high_pose)
        time.sleep(1)

        # Reset
        self._arm_grasper.rest_arm()
        time.sleep(1)

    def run(self):

        print(f'Number of Waypoints: {self._graph_navigator.get_waypoint_count()}')

        while True:
            input_str = input("Select an action: 0 - Go to waypoint, 1 - Pick, 2 - Place. To quit, type 'q':")
            while not input_str.isdigit() and input_str != 'q':
                print('wrong input')
                input_str = input("Select an action: 0 - Go to waypoint, 1 - Pick, 2 - Place. To quit, type 'q':")

            if input_str == 'q':
                break
            
            elif input_str == '0':
                waypoint_index = input('Which waypoint to go to? To end, type q:')
                while not waypoint_index.isdigit() and waypoint_index != 'q':
                    print('wrong input')
                    waypoint_index = input('Which waypoint to go to? To end, type q:')
                
                if waypoint_index == 'q':
                    break
                
                self.go_to(f'Waypoint_{waypoint_index}')
            
            elif input_str == '1':
                constraint = input('Which direction to grasp: 0 - Top-Down, 1 - Horizontal? To end, type q:')
                while not constraint.isdigit() and constraint != 'q':
                    print('wrong input')
                    constraint = input('Which direction to grasp: 0 - Top-Down, 1 - Horizontal? To end, type q:')

                if constraint == 'q':
                    break

                elif constraint == '0':
                    constraint = 'top_down'

                elif constraint == '1':
                    constraint = 'horizontal'

                self.pick(constraint)

            elif input_str == '2':
                self.place()

        self._graph_navigator.on_quit()

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
    sdk = bosdyn.client.create_standard_sdk('SentMapNavigator')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    lease_client = robot.ensure_client(LeaseClient.default_service_name)

    try:
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            try:
                sentmap_navigator = SentMapNavigator(robot, options.map_path)
                sentmap_navigator.run()

            except Exception as exc:  # pylint: disable=broad-except
                print("SentMapNavigator threw an error.")
                print(exc)
            finally:
                sentmap_navigator.on_quit()

    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )