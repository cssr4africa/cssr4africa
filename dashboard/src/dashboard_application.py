"""
dashboard_application.py - Dash Web Application for Cultural Knowledge Survey

This script implements a Dash-based interactive dashboard to visualize 
survey data related to Rwandan cultural knowledge. The application includes:
- A slider for question selection
- A bar chart representation of responses
- A dynamic table displaying corresponding data
- An extra section that appears based on specific questions

The script interacts with `dashboard_implementation.py`, which provides 
the data processing and update functions.

------------------------------------------------------
Libraries Used:
    - os (File path handling)
    - dash (For web dashboard)
    - dash_core_components (For charts and sliders)
    - dash_html_components (For UI elements)
    - dash.dependencies (For callback handling)
    - pandas (For data manipulation)
    - plotly (For graph rendering)
    - gspread (For handling Google Sheets data)
    - openpyxl (For working with Excel files)
------------------------------------------------------

Parameters:
    - main_categories (List of main ontology categories)

Configuration File:
    - dashboard_configuration.ini (Stores dashboard settings)

External Resources:
    - FontAwesome (For icons)
    - External CSS for responsive design

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
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
from dashboard_implementation import update_bar_chart, update_table, update_extra_section  #Import functions

from dashboard_implementation import (
    CSSR4all, are_you_rwandan, part_two_column, one_ans_list, two_ans_list, three_ans_list,
    other_in_choosen, questions_dict, file_path_data_folder, update_bar_chart, update_table,
    update_extra_section
)

file_path_data_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

# Use icon from fontawesome
fontawesome_css = "https://use.fontawesome.com/releases/v5.8.1/css/all.css"
# Make page responsive
meta_tags = [{'name': 'viewport', 'content': 'width=device-width'}]
# Use external CSS stylesheet
external_stylesheets = [fontawesome_css, meta_tags]

# Define the main ontology categories
main_categories = ['Spatial Interaction', 'Verbal Interaction', 'Non-verbal Interaction']
# Create the app
app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

# Create layout
app.layout = html.Div([
    #.............................................................................................................
    # Header Section

    
    html.Div([
        html.H5('Culturally Sensitive Social Robotics for Africa', style={'margin-bottom': '0px', 'color': '#AB0000',
                                                                          'font-size': '25px', 'height': '60px',
                                                                          'width': 'auto', 'font-weight': 'bold',
                                                                          'margin-top': '10px', 'text-align': 'center'}),
    ], className='row '),

    # logo 
    html.Div([
        html.Center(
            html.A(
                html.Img(
                    src='https://cssr4africa.github.io/images/CSSRforAfrica_logo_red.png',
                    id='corona-image',
                    style={'height': '50px', 'width': 'auto', 'margin-bottom': '0px', 'margin-top': '0px'}
                ),
                href="https://cssr4africa.github.io/",  # Link to CSSR4Africa website
                target="_blank"  # Opens link in a new tab
            )
        )
    ], className=' '),


    html.Div([
        html.H4('Rwandan Cultural Knowledge Survey', style={'margin-bottom': '0px', 'color': '#AB0000',
                                                            'font-size': '20px', 'height': '60px',
                                                            'width': 'auto', 'font-weight': 'normal',
                                                            'margin-top': '10px', 'text-align': 'center'}),
    ], className=' '),

    html.Div([
        html.H4('Kigali, Rwanda', style={'margin-bottom': '30px', 'color': '#AB0000',
                                         'font-size': '20px', 'height': '0px',
                                         'width': 'auto', 'font-weight': 'normal',
                                         'margin-top': '-30px', 'text-align': 'center'}),
    ], className='row my_display'),
    # ......................................................................................................... 
    html.Div([
        #..........................................................................................................
        html.Div([
            # Section 2: Slider
            html.Div([
                dcc.Slider(
                    id='select_question',
                    max=len(part_two_column),  # Adjust the max value accordingly 
                    step=1,
                    min=-1,
                    included=False,
                    updatemode='drag',
                    tooltip={'always_visible': True},
                    marks={-1: 'RW?', **{str(i): str(i) for i in range(len(part_two_column)+1)}},  # EY
                    #  marks={str(i): str(i) for i in range(len(part_two_column) +1)},#EY
                )
            ], className='slider')
        ]),

        # Section 3: Table and Bar Chart (Bottom Section)
        html.Div([

            #.............................................................................................................
            # Subsection 3.1: Bar Chart
            html.Div([
                dcc.Graph(id='bar_graph',
                          config={'displayModeBar': 'hover'},
                          className='bar_graph_border')  # Placeholder for the bar chart
            ], className='create_container six columns', style={'margin-top': '10px', 'margin-bottom': '10px'}),

            #.............................................................................................................
            # Subsection 3.2: Table Chart
            html.Div([
                # Table
                html.Div([
                    html.H3(id='table-title',
                            style={'textAlign': 'center',
                                   'color': 'black', 'margin-top': '20px',
                                   'font-size': '14px',
                                   'font-family': 'Arial',
                                   'margin-bottom': '20px',
                                   }),

                    html.Div(id='table-container')
                ])
            ], className='create_container six columns', style={'margin-top': '10px', 'margin-bottom': '10px'}),
            #...........................................................................................................
        ], className='row flex_display'),
    ]),
    #..............................................................................................................
    # Extra Section
    html.Div([
        html.Div(id='extra-table-container', style={'margin-top': '20px', 'margin-bottom': '20px'}),
        html.P(id='conclusion-message', style={'textAlign': 'center', 'color': 'black', 'margin-top': '20px', 'font-size': '16px', 'font-family': 'Arial'})
    ], id='extra-section', className='row flex_display', style={'display': 'none'})  # Hidden by default

], className='mainContainer', style={'display': 'flex', 'flex-direction': 'column'})



# Update bar chart using imported function
@app.callback(Output('bar_graph', 'figure'), [Input('select_question', 'value')])
def update_bar(selected_question):
    return update_bar_chart(selected_question)  # Calls the imported function

# Update table using imported function
@app.callback(Output('table-container', 'children'), [Input('select_question', 'value')])
def update_table_data(selected_question):
    return update_table(selected_question)  # Calls the imported function

# Update extra section using imported function
@app.callback(
    [Output('extra-section', 'style'),
     Output('extra-table-container', 'children'),
     Output('conclusion-message', 'children')],
    [Input('select_question', 'value')]
)
def update_extra(selected_question):
    return update_extra_section(selected_question)  # Calls the imported function

if __name__ == "__main__":
    app.run_server(debug=True)
