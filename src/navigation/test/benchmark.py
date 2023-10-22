import subprocess
import time
import signal
import argparse
import os

WORLD = "aquabot_task_easy"

def launch_ros2_commands(gain, sigma):
    # Define the launch commands with the updated gain and sigma values
    launch_command = f"ros2 launch aquabot_gz competition.launch.py world:={WORLD} headless:=true"
    navigation_command = f"ros2 run navigation navigation {gain} {sigma}"

    # Launch the commands in separate processes
    launch_process = subprocess.Popen(launch_command, shell=True)
    navigation_process = subprocess.Popen(navigation_command, shell=True)

    return launch_process, navigation_process


def main(gain_min, gain_max, gain_iter, sigma_min, sigma_max, sigma_iter):
    gain_step = gain_max - gain_min / gain_iter
    sigma_step = sigma_max - sigma_min / sigma_iter
    gain = gain_min
    for i in range (0, gain_iter):
        sigma = sigma_min
        for j in range (0, sigma_iter):
            print(f"Launching with gain={gain} and sigma={sigma}")
            launch_process, navigation_process = launch_ros2_commands(gain, sigma)

            # Wait for the navigation process to complete
            navigation_process.wait()
            print("Navigation process completed.")

            print("Killing processes")
            os.kill(launch_process.pid, signal.SIGTERM)
            launch_process.wait()
            sigma += sigma_step
        gain += gain_step

    print("All iterations completed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ROS2 Launch and Kill Script")
    parser.add_argument("--gmin", type=float, default=0.25, help="Minimum gain value")
    parser.add_argument("--gmax", type=float, default=0.5, help="Maximum gain value")
    parser.add_argument("--giter", type=int, default=1, help="Number of iteration on gain")
    parser.add_argument("--smin", type=float, default=0.1, help="Minimum sigma value")
    parser.add_argument("--smax", type=float, default=0.5, help="Maximum sigma value")
    parser.add_argument("--siter", type=int, default=2, help="Number of iteration on sigma")

    args = parser.parse_args()
    main(args.gmin, args.gmax, args.giter, args.smin, args.smax, args.siter)
