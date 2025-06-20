import os
from SpotStack import GraphNavigator
from Object_Centric_Local_Navigation.object_centric_local_navigation import ObjectCentricLocalNavigation

class SentMapNavigator:

    def __init__(self, robot, map_path, architecture, weight_name):
        
        self._map_path = map_path
        graph_path = os.path.join(map_path, 'spot_graph')

        self._graph_navigator = GraphNavigator(robot, graph_path)
        self._oc_local_navigator = ObjectCentricLocalNavigation(architecture, weight_name, robot)

    def run(self):

        print(f'Nuber of Waypoint: {self._graph_navigator.get_waypoint_count()}')

        while True:
            input_str = input(f"Which waypoint to go to? To end, type q:")
            while not input_str.isdigit() and input_str != 'q':
                print('wrong input')
                input_str = input(f"Which waypoint to go to? To end, type q:")
            
            if input_str == 'q':
                break
        
            print(f'Going to Waypoint_{input_str}')
            self._graph_navigator.navigate_to(f'Waypoint_{input_str}')
            goal_image_path = os.path.join(self._map_path, f'Waypoint_{input_str}', 'front_image.jpg')
            self._oc_local_navigator.run(goal_image_path)

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
                sentmap_navigator = SentMapNavigator(robot, options.map_path, 'DinoMlp5', 'DinoMLP5_discretized.pth')
                sentmap_navigator.run()

            except Exception as exc:  # pylint: disable=broad-except
                print("SentMapNavigator threw an error.")
                print(exc)

    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )