
<div align="center">
  <h1> Dashboard</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:80%; height:auto;">
</div>

## Overview
The dashboard Questionnaire Dashboard is an interactive data visualization tool built using Dash and Plotly. It displays survey responses dynamically, supports bar charts, and provides insights into cultural knowledge based on Google Sheets or Excel files.
# Documentation
Accompnaying this code, there are deliverable reports that provides a detailed explanation of the code and how to run the tests. The deliverable reports are can be found in [D1.2 Rwandan Cultural Knowledge, version 1](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D1.2.pdf)

## Features
- Fetches survey data from Google Sheets or Excel
- Provides bar charts, tables, and interactive sliders for navigation
- Displays cultural knowledge insights from a structured questionnaire

## Folder Structure
    dashboard/
    ├── config/                                  
    │   ├── dashboard_configuration.ini          # Configuration file
    ├── data/                                     
    │   ├── cssr-406009-380cb664c48a.json        # Google Sheets credentials (if used)
    │   ├── Cultural Knowledge Survey.xlsx       # Excel file (if used)
    │   ├──.
    │   ├──.
    │   ├──.
    ├── src/                                   
    │   ├── dashboard_application.py                                
    │   ├── dashboard_data_processing.py                     
    │   ├── dashboard_implementation.py                    
    │   ├── dashboard_questions_data.py          # Stores question mappings                  
        ├── assets/                        
        │   ├── dashboard_style.css 
    ├── CSSR4AfricaLogo.svg                      # logo  
    ├── dashboard_requirements.txt               # Required Python libraries
    ├── README.md                                

## Installation Guide

## Clone the Repository
- Clone the CSSR4Africa software from the GitHub repository
     ``` bash 
         git clone https://github.com/cssr4africa/cssr4africa.git
         cd dashboard
     ```
- Install Dependencies
     ``` bash 
        pip install -r dashboard_requirements.txt
     ```
     
- Configure Data Source: Modify the config/dashboard_configuration file to use Google Sheets or Excel:

    - Option 1: Use Google Sheets
    Set up a Google Service Account and download the JSON credentials file. Move the file to the data/ folder, then update dashboard_configuration.ini in the config/ folder as follows.
        ``` bash 
            use_spreadsheet             false                                            # Set this to true
            use_excel                   true                                             # Set this to false
            spreadsheet_id              1nWPZX65-UQ-4nUGiKPIyPFY5AmtvW8ZdeyXEP-K5Vv      # Update this 
            excel_file                  Cultural Knowledge Survey.xlsx                   # default
            excel_sheet                 Sheet1_English_Version                           # default
       ```
    - Option 2: Use Excel File
   The Excel version of the data is in the data/ folder, and if Google Sheets is not needed, update dashboard_configuration.ini in the config/ folder as follows.
         ``` bash 
            use_spreadsheet             false                                            # Set this to false
            use_excel                   true                                             # Set this to true
            spreadsheet_id              1nWPZX65-UQ-4nUGiKPIyPFY5AmtvW8ZdeyXEP-K5Vv      # default
            excel_file                  Cultural Knowledge Survey.xlsx                   # Update with the appropriate Excel file name
            excel_sheet                 Sheet1_English_Version                           # This is the first sheet name of the Excel file.
       ```
- Running the Dashboard
    ``` bash 
       python .\src\dashboard_application.py
    ```
-  Open http://127.0.0.1:8050/ in your browser.

## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:ebirhan@andrew.cmu.edu">ebirhan@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Date:  2025-03-13