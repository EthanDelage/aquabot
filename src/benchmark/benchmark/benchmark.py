import rclpy
import subprocess


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('benchmark')
    generate_scenario()
    replace_content('scenario.sdf', 'src/aquabot/aquabot_gz/worlds/aquabot_benchmark.sdf')
    node.destroy_node()
    rclpy.shutdown()


def generate_scenario():
    command = "ros2 run aquabot_python generate_scenario.py"

    try:
        # Run the command and wait for it to finish
        subprocess.run(command, shell=True, check=True)
        print("Command execution completed successfully.")

    except subprocess.CalledProcessError as e:
        # If the command returns a non-zero exit code, an exception will be raised
        print(f"Command execution failed with exit code {e.returncode}.")
        print(e)


def replace_content(scenario_file_path, target_file_path):
    start_line = '<!-- GENERATED SCENARIO -->'
    task_hard_file_path = 'src/aquabot/aquabot_gz/worlds/aquabot_task_hard.sdf'
    with open(task_hard_file_path, 'r') as target_file:
        # Find the line number of the seed line
        lines = target_file.readlines()
        scenario_start_line = next((i + 1 for i, line in enumerate(lines) if start_line in line), None)

    if scenario_start_line is not None:
        # Read the content from the target file
        with open(scenario_file_path, 'r') as source_file:
            for _ in range(2):
                next(source_file)
            new_content = source_file.read()

        # Replace the content in the original file
        lines = [line.strip('\n') for line in lines]
        lines[scenario_start_line:] = new_content.split('\n')

        # Write the modified content back to the original file
        with open(target_file_path, 'w') as target_file:
            target_file.write('\n'.join(lines))
    else:
        print(f"Line `{start_line}` not found in {scenario_file_path}.")

if __name__ == '__main__':
    main()
