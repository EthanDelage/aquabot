#!/usr/bin/env python3

from aquabot_python.scenario_generator import regenerate_scenario_file

def main():
    regenerate_scenario_file("scenario_easy", False)
    regenerate_scenario_file("scenario_medium", True)
    regenerate_scenario_file("scenario_hard", True)

if __name__ == "__main__":
    main()