import rclpy
import subprocess
from typing import AnyStr


def main(args=None):
    scenario_path: str = 'scenario.sdf'
    file_header_path: str = 'src/benchmark/resource/world_file_header'
    target_file_path: str = 'install/share/aquabot_gz/worlds/aquabot_benchmark.sdf'

    rclpy.init(args=args)
    node = rclpy.create_node('benchmark')
    generate_scenario()
    replace_content(scenario_path, file_header_path, target_file_path)
    node.destroy_node()
    rclpy.shutdown()


def generate_scenario() -> None:
    command = "ros2 run aquabot_python generate_scenario.py"

    try:
        # Run the command and wait for it to finish
        subprocess.run(command, shell=True, check=True)
        print("Command execution completed successfully.")

    except subprocess.CalledProcessError as e:
        # If the command returns a non-zero exit code, an exception will be raised
        print(f"Command execution failed with exit code {e.returncode}.")
        print(e)


def replace_content(scenario_file_path: str, file_header_path: str, target_file_path: str) -> None:

    # Open the target file in write mode
    with open(target_file_path, 'w') as target_file:
        # Write the content from the header file to the target file
        with open(file_header_path, 'r') as header_file:
            target_file.write(header_file.read())

        # Write the content from the scenario file to the target file
        with open(scenario_file_path, 'r') as scenario_file:
            # Skip the first two line of the scenario file
            for _ in range(2):
                next(scenario_file)
            target_file.write(scenario_file.read())


if __name__ == '__main__':
    main()
