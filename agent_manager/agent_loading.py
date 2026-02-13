import yaml

def load_yaml_file(agents_gen_data: str):
    
    with open(agents_gen_data) as f1:
        agent_data = yaml.load(f1, Loader=yaml.SafeLoader)

    agent_list = ()

    for agent in agent_data.values():
        stage_prefix = agent.get("stage_prefix")
        character_name = agent.get("character_name")
        initial_pose = [round(agent.get("initial_pose")[0], 2), round(agent.get("initial_pose")[1], 2), round(agent.get("initial_pose")[2], 2)]
        goal_pose = agent.get("goal_pose")
        orientation = agent.get("orientation")
        controller_stats = agent.get("controller_stats")
        velocity = agent.get("velocity")

        print(type(velocity))

def main():
    base_dir = Path(__file__).resolve().parent  # ros2isaacsim/agent_manager

    agent_path = (base_dir / ".." / "isaac_utils" / "config" / "agent_data_gen.yaml").resolve()
    load_yaml_file(agent_path)

if __name__ == "__main__":
    main()
