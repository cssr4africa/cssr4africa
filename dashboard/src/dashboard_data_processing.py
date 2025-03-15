"""
dashboard_data_processing.py 
    Loads and processes survey response data for the Dash application.
    This script handles data extraction from Google Sheets or Excel, filters Rwandan respondents, 
    categorizes survey questions, and prepares data for visualization.

Author:   Eyerusalem Mamuye Birhan
Date:     2025-03-13
Version:  1.0

Copyright (C) 2023 CSSR4Africa Consortium
This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.
Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""
import os
import pandas as pd
import gspread
import re
from google.oauth2.service_account import Credentials

# Define project root and folder paths
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
data_folder = os.path.join(project_root, "data")
config_folder = os.path.join(project_root, "config")

# Load and parse the space-separated configuration file
config_path = os.path.join(config_folder, "dashboard_configuration.ini")

if not os.path.exists(config_path):
    raise FileNotFoundError(f"Error: Configuration file not found at {config_path}")

def read_config(file_path):
    """Reads a space-separated key-value configuration file."""
    config_dict = {}
    with open(file_path, "r", encoding="utf-8") as file:
        for line in file:
            line = line.strip()
            if not line or line.startswith("#"):  # Ignore empty lines and comments
                continue
            parts = re.split(r'\s+', line, maxsplit=1)  # Split by any whitespace
            if len(parts) == 2:
                key, value = parts
                config_dict[key] = value
    return config_dict

# Read config values
config = read_config(config_path)

# Get configuration values
use_spreadsheet = config.get("use_spreadsheet", "false").lower() == "true"
use_excel = config.get("use_excel", "true").lower() == "true"
spreadsheet_id = config.get("spreadsheet_id", "")
excel_file_name = config.get("excel_file", "Cultural Knowledge Survey.xlsx")
excel_sheet_name = config.get("excel_sheet", "Sheet1_English_Version")

# Initialize dataframe
CSSR4all = None

if use_spreadsheet:
    # Google Sheets authentication
    credentials_file = os.path.join(data_folder, "cssr-406009-380cb664c48a.json")

    if not os.path.exists(credentials_file):
        raise FileNotFoundError(f"Error: The credentials file at {credentials_file} does not exist.")

    credentials = Credentials.from_service_account_file(credentials_file, scopes=['https://www.googleapis.com/auth/spreadsheets'])
    gc = gspread.authorize(credentials)

    if not spreadsheet_id:
        raise ValueError("Error: 'spreadsheet_id' is missing in the configuration file.")

    worksheet = gc.open_by_key(spreadsheet_id).sheet1

    # Fetch data
    df = worksheet.get_all_values()
    CSSR4all = pd.DataFrame(df[1:], columns=df[0])

elif use_excel:
    # Load data from Excel
    excel_file_path = os.path.join(data_folder, excel_file_name)

    if not os.path.exists(excel_file_path):
        raise FileNotFoundError(f"Error: The Excel file at {excel_file_path} does not exist.")

    CSSR4all = pd.read_excel(excel_file_path, sheet_name=excel_sheet_name)

else:
    raise ValueError("Error: No data source selected. Enable either 'use_spreadsheet' or 'use_excel' in the configuration file.")

# Ensure required column exists
if "3. Are you Rwandan?" not in CSSR4all.columns:
    raise ValueError("Error: '3. Are you Rwandan?' column is missing in the dataset.")

# Filter Rwandan respondents
are_you_rwandan = CSSR4all["3. Are you Rwandan?"]
CSSR4all = CSSR4all[CSSR4all["3. Are you Rwandan?"] == "Yes"].reset_index(drop=True)

# Extract question columns
all_data_list = list(CSSR4all.columns)
new_list = [i for i in CSSR4all.columns if i[0].isdigit()]

# Remove demographic questions
part_two_column = [col for col in new_list[2:] if col != "3. Are you Rwandan?"]

# Identify question types
if len(new_list) > 48:
    new_list_update = new_list[48:]
else:
    new_list_update = []

new_list_update += [
    "10. Should you pause before responding when someone asks you a question?",
    "11. In an interaction where you and someone else take turns to speak, would you signal that you want to speak?",
]

# Rearrange list
if len(new_list_update) > 2:
    new_list_update.insert(0, new_list_update.pop())
    new_list_update.insert(0, new_list_update.pop())

one_ans_list = new_list_update[:3]
three_ans_list = new_list_update[3:11]
two_ans_list = new_list_update[11:]

# Load questions dictionary
try:
    from src.dashboard_questions_data import questions_dict
except ImportError:
    raise ImportError("Error: Unable to import 'dashboard_questions_data'. Ensure it exists in the 'src' folder.")

# Questions with an "other" option
other_in_choosen = [
    "2. How should you acknowledge someone when passing them?",
    "7. How should you address someone who is older than you and who you haven’t met before?",
    "8. How should you address someone who is the same age as you and who you haven’t met before?",
    "9. How should you address someone who is younger than you and who you haven’t met before?",
]
