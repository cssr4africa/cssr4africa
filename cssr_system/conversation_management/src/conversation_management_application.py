#!/usr/bin/env python3

import os

import rospy

import conversation_management_implementation as cm_imp
import conversation_management_utils as cm_utils


if __name__ == "__main__":
    current_file_dir = os.path.dirname(__file__)
    config_file = os.path.join(os.path.dirname(current_file_dir), "config", "conversation_management_configuration.ini")
    system_prompt_file = os.path.join(os.path.dirname(current_file_dir), "data", "system_prompt_input.txt")
    config = cm_utils.parse_config_file(config_file)

    with open(system_prompt_file, "r") as f:
        system_prompt = f.read()

    cm_imp.initialise(config, system_prompt)

    try:
        cm_imp.run()
    except rospy.ROSInterruptException:
        pass
