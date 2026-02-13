import sys
import yaml

from isaacsim_msgs.msg import Person
from isaacsim_msgs.srv import Pedestrian

import rclpy
from rclpy.node import Node


class SpawnPedestrians(Node):

    def __init__(self):
        super().__init__('Spawn_peds')
        self.cli = self.create_client(Pedestrian, '/isaac/spawn_pedestrian')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Pedestrian.Request()

    def send_request(self, people):
        self.req.people = people
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    pedestrian_client = SpawnPedestrians()

    agents_gen_data_path = "/home/brainfucker/Downloads/arena-isaac/src/ros2isaacsim/isaac_utils/config/agent_data_gen.yaml"
    with open(agents_gen_data_path) as f:
        agent_data = yaml.load(f, Loader=yaml.SafeLoader)
    

    people = []
    for agent in agent_data.values():
        person = Person()
        person.stage_prefix = agent.get("stage_prefix")
        person.character_name = agent.get("character_name")
        person.initial_pose = agent.get("initial_pose")
        person.goal_pose = agent.get("goal_pose")
        person.orientation = agent.get("orientation")
        person.controller_stats = agent.get("controller_stats")
        person.velocity = agent.get("velocity")
        people.append(person)

    response = pedestrian_client.send_request(people)
    pedestrian_client.get_logger().info(f'Spawn success: {response.ret}')

    pedestrian_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()