"""
speech_event_test_utils.py - utility functions for the speechEventTest ROS node

Author:     Clifford Onyonka, Carnegie Mellon University Africa
Email:      cliffor2@andrew.cmu.edu
Date:       2026-04-21
Version:    v1.1

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

def parse_config_file(config_file_path):
    """ Get a dict representing the configuration options stored in the
    passed configuration file

    Parameters:
        config_file_path (str): path to a configuration file

    Returns:
        dict:    key-value pairs of configurations stored in the passed
            config file
    """
    config = {}

    with open(config_file_path, "r") as f:
        for line in f.readlines():
            if len(line) < 1:
                continue
            a_list = line.split("\t") if "\t" in line else line.split(" ")
            config[a_list[0]] = a_list[-1]
    
    return config
