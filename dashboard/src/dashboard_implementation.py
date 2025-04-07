"""
dashboard_implementation.py 
    Data Processing and Chart/Table Updates for Dash Application
    This script processes survey response data and dynamically updates visualizations, including bar charts, tables, and additional sections in the dashboard.

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
import plotly.graph_objs as go
import dash
from dash import dcc, html, dash_table 
import sys

# Dynamically find the main project directory (cssr4Africa_dashboard)
main_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, main_folder)  # Add the main project folder to Python path

from src.dashboard_data_processing import (
    CSSR4all,
    are_you_rwandan,
    part_two_column,
    one_ans_list,
    two_ans_list,
    three_ans_list,
    other_in_choosen,
    questions_dict
)

file_path_data_folder = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")

# Function to break the title 
def break_title(title, max_width, font_size=12):
    max_chars_per_line = max_width // (font_size * 0.6)  # Assuming an average character width
    words = title.split()
    lines = []
    current_line = words[0]
    for word in words[1:]:
        if len(current_line) + len(word) + 1 <= max_chars_per_line:
            current_line += ' ' + word
        else:
            lines.append(current_line)
            current_line = word
    lines.append(current_line)
    return '<br>'.join(lines)

def update_bar_chart(selected_question):
    # if selected_question is None, set it to -1
    if selected_question is None:
        selected_question = -1
    # print(selected_question)
    
    if selected_question == -1:
        # Use are_you_rwandan data
        labels_rwandan = ['Yes', 'No']
        values_rwandan = are_you_rwandan.value_counts().reindex(labels_rwandan, fill_value=0)

        fig_bar = go.Figure(data=[go.Bar(x=labels_rwandan, y=values_rwandan.values, marker=dict(color='#FF7070'), width=0.3)])
        titles = 'Nationality Information'
        tickvals = list(values_rwandan.values)
    
    elif selected_question == 0:
        col_gender = '2. Which are you?'
        col_age = '1. What age are you?'

        labels = ['Female', 'Male', ' ', '20–29', '30–39', '40–49', '50–59', '60 or more.']

        values_gender = CSSR4all[col_gender].value_counts().reindex(labels, fill_value=0)
        values_age = CSSR4all[col_age].value_counts().reindex(labels, fill_value=0)
        values_combined = values_gender.add(values_age, fill_value=0)

        fig_bar = go.Figure(data=[go.Bar(x=labels, y=values_combined.values, marker=dict(color='#FF7070'), width=0.3)])
        titles = 'Demographic Information'
        tickvals = list(values_combined.values)

    else:
        col_name = part_two_column[selected_question - 1]
        
        x_labels = list(questions_dict.get(col_name, {}).values())[0] if col_name in questions_dict else ["Yes", "No"]
        values_counts = CSSR4all[col_name].value_counts()
        other_values = set(values_counts.index) - set(x_labels)

        if other_values:
            values_counts['Other. Please specify:'] = sum(values_counts[value] for value in other_values)

        title_text = break_title(col_name, max_width=550)

        fig_bar = go.Figure(data=[go.Bar(x=x_labels, y=values_counts.reindex(x_labels, fill_value=0).values,
                                         marker=dict(color='#FF7070'), width=0.3)])
        titles = title_text[title_text.index('.') + 1:]
        tickvals = list(values_counts.reindex(x_labels, fill_value=0).values)

    fig_bar.update_layout(
        title=titles,  # Use the second line of the broken title
        title_font=dict(size=12),  # Set the title font size to 12 #DV 16
        title_x=0.5,  # Center the title
        xaxis_title="",
        yaxis_title='Number of Respondents',
        xaxis=dict(showline=True,
                   showgrid=False,
                   showticklabels=True,
                   linecolor='black',
                   linewidth=1,
                   ticks='outside',
                   tickfont=dict(
                       family='Arial',
                       size=12,
                       color='black')
                   ),
        yaxis=dict(
            visible=True,
            color='gray',
            linecolor='black',  # DV
            linewidth=1,  # DV
            showline=True,  # DV
            showgrid=False,  # DV
            showticklabels=True,  # DV
            ticks='outside',  # DV
            tickfont=dict(  # DV
                family='Arial',
                size=12,
                color='black'),
            tickmode='array',  # EY added
            tickvals=list(tickvals)  # EY added
        )
    )
    return fig_bar

def update_table(selected_question):
    selected_question = selected_question or 0
    description_column = []

    if selected_question == 0 or selected_question == -1:
        return html.P(html.Center(html.Img(src='https://cssr4africa.github.io/images/CSSRforAfrica_logo_red.png',
                                           id='corona-image',
                                           style={'height': '50px',
                                                  'width': 'auto',
                                                  'margin-bottom': '0px',
                                                  'margin-top': '200px'})))

    else:
        column_name = part_two_column[selected_question - 1]
        column_index = list(CSSR4all.columns).index(column_name)

    if column_name not in (two_ans_list + one_ans_list + other_in_choosen + three_ans_list):
        return html.P(html.Center(html.Img(src='https://cssr4africa.github.io/images/CSSRforAfrica_logo_red.png',
                                           id='corona-image',
                                           style={'height': '50px',
                                                  'width': 'auto',
                                                  'margin-bottom': '0px',
                                                  'margin-top': '200px'})))
    if column_name in three_ans_list:
        # the value of the column
        description_column_1 = CSSR4all.iloc[:, column_index + 1].fillna('').astype(str)
        description_column_2 = CSSR4all.iloc[:, column_index + 2].fillna('').astype(str)
        description_column_3 = CSSR4all.iloc[:, column_index + 3].fillna('').astype(str)


        # column name or title
        description_column_nam_1 = CSSR4all.columns[column_index + 1]
        description_column_nam_2 = CSSR4all.columns[column_index + 2]
        description_column_nam_3 = CSSR4all.columns[column_index + 3]

        for desc1, desc2, desc3 in zip(description_column_1, description_column_2, description_column_3):
            description_column.append(f'{desc1}. {desc2}. {desc3}.' if desc1 != 'nan' and desc2 != 'nan' and desc3 != 'nan' else '')

        dynamic_columns = [{'name': description_column_nam_1, 'id': description_column_nam_1},
                           {'name': description_column_nam_2, 'id': description_column_nam_2},
                           {'name': description_column_nam_3, 'id': description_column_nam_3}]

        table_data = [{dynamic_columns[0]['name']: desc1, dynamic_columns[1]['name']: desc2, dynamic_columns[2]['name']: desc3}
                      for desc1, desc2, desc3 in zip(description_column_1, description_column_2, description_column_3)
                      if desc1 != '' or desc2 != '' or desc3 != '']

    elif column_name in two_ans_list:
        description_column_1 = CSSR4all.iloc[:, column_index + 1].fillna('').astype(str)
        description_column_2 = CSSR4all.iloc[:, column_index + 2].fillna('').astype(str)

        description_column_nam_1 = CSSR4all.columns[column_index + 1]
        description_column_nam_2 = CSSR4all.columns[column_index + 2]

        for desc1, desc2 in zip(description_column_1, description_column_2):
            if desc1.strip() or desc2.strip():
                description_column.append(f'{desc1}. {desc2}.')

        dynamic_columns = [{'name': description_column_nam_1, 'id': description_column_nam_1},
                        {'name': description_column_nam_2, 'id': description_column_nam_2}]

        table_data = [{dynamic_columns[0]['name']: desc1, dynamic_columns[1]['name']: desc2}
                    for desc1, desc2 in zip(description_column_1, description_column_2)
                    if desc1.strip() or desc2.strip()]

    elif column_name in one_ans_list:
        description_column = [f'{result}' for result in CSSR4all.iloc[:, column_index + 1].astype(str)]
        description_column_nam = CSSR4all.columns[column_index + 1]
        dynamic_columns = [{'name': description_column_nam, 'id': description_column_nam}]

        table_data = [{dynamic_columns[0]['name']: desc} for desc in description_column if desc != '']

    elif column_name in other_in_choosen:
        x_labels = list(questions_dict[column_name].values())
        x_labels = x_labels[0]
        values_counts = CSSR4all[column_name].value_counts()
        all_values = CSSR4all[column_name].tolist()
        other_values = [value for value in all_values if value not in x_labels]

        if other_values:
            description_column = [f'{result}' for result in other_values]
            description_column_nam = 'Other'
            dynamic_columns = [{'name': description_column_nam, 'id': description_column_nam}]

            table_data = [{dynamic_columns[0]['name']: desc} for desc in description_column if desc != '']
        else:
            return html.P(html.Center(html.Img(src='https://cssr4africa.github.io/images/CSSRforAfrica_logo_red.png',
                                               id='corona-image',
                                               style={'height': '50px',
                                                      'width': 'auto',
                                                      'margin-bottom': '0px',
                                                      'margin-top': '200px'})))

    else:
        description_column = dynamic_columns = table_data = []

    if not table_data:
        return html.P("No data available.")
    else:
        return dcc.Loading(
            type="circle",
            children=[dash_table.DataTable(
                id='table',
                columns=dynamic_columns,
                data=table_data,
                style_table={'height': '410px', 'overflowY': 'auto'},
                style_header={
                    'backgroundColor': '#FF7070',
                    'color': 'white',
                    'fontSize': '14px',
                    'fontWeight': 'bold',
                    'whiteSpace': 'normal',
                    'border': '1px solid black',
                    'maxWidth': '200px'
                },
                style_cell={
                    'whiteSpace': 'normal',
                    'height': 'auto',
                    'textAlign': 'left',
                    'fontSize': '12px',
                    'font-family': 'Arial',
                    'padding': '0px',
                    'border': '1px solid grey'
                }
            )]
        )

def update_extra_section(selected_question):
    # Ensure selected_question is an integer, default to 0 if None
    if selected_question is None:
        selected_question = 0

    extra_section_style = {'display': 'none'}
    extra_table_children = []
    conclusion_message = ''

    if selected_question == 38:
        # Show the extra section
        extra_section_style = {'display': 'block'}
        
        # Generate table data for question 38
        df = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_q3_11.csv'))
        
        # Create a table with 2 columns for question 47
        answer_counts = df['Suggested Phrase or words'].value_counts(normalize=True) * 100
        table_data = [{'simplest version of the survey answer': answer, 
                       'count of the answer': f'{percentage:.2f}%'} 
                      for answer, percentage in answer_counts.items()]

        # Define the table columns
        extra_table_children = dash_table.DataTable(
            columns=[
                {'name': 'simplest version of the survey answer', 'id': 'simplest version of the survey answer'},
                {'name': 'count of the answer', 'id': 'count of the answer'}
            ],
            data=table_data,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Set conclusion message for question 38
        conclusion_message = '\n'

    elif selected_question == 47:
        # Show the extra section
        extra_section_style = {'display': 'block'}
        
        # Generate table data for question 47
        df = pd.read_csv(os.path.join(file_path_data_folder, 'df_20.csv'))
        
        # Create a table with 2 columns for question 47
        answer_counts = df['Suggested Phrase or words'].value_counts(normalize=True) * 100
        table_data = [{'simplest version of the survey answer': answer, 
                       'count of the answer': f'{percentage:.2f}%'} 
                      for answer, percentage in answer_counts.items()]

        # Define the table columns
        extra_table_children = dash_table.DataTable(
            columns=[
                {'name': 'simplest version of the survey answer', 'id': 'simplest version of the survey answer'},
                {'name': 'count of the answer', 'id': 'count of the answer'}
            ],
            data=table_data,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Set conclusion message for question 47
        conclusion_message = 'To draw someone’s attention to something, use a head-nodding gesture while looking at the object.'

    elif selected_question == 48:
        extra_section_style = {'display': 'block'}
       # Load datasets for face, hand, and body
        df_face = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_face_2.csv'))
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_2.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_2.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_face = df_face['Suggested phrase or words'].value_counts(normalize=True) * 100
        answer_counts_hand = df_hand['Suggested phrase or words.1'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested Phrase or words.2'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (face, face_percentage), (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_face.items(), answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'face gesture': face,
                'face response count': f"{face_percentage:.2f}%",
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Face Gesture', 'id': 'face gesture'},
                {'name': 'Face Response Count', 'id': 'face response count'},
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Face, Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 48
        conclusion_message = f'To express gratitude, common gestures include nodding, smiling, and bowing the head, using hand gestures like a thumbs up or clasped hands, and slight bowing of the body.'

    elif selected_question == 49:
        extra_section_style = {'display': 'block'}
        # Load datasets for face, hand, and body
        df_face = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_face_3.csv'))
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_3.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_3.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_face = df_face['Suggested phrase or words'].value_counts(normalize=True) * 100
        answer_counts_hand = df_hand['Suggested phrase or words.1'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested Phrase or words.2'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (face, face_percentage), (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_face.items(), answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'face gesture': face,
                'face response count': f"{face_percentage:.2f}%",
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Face Gesture', 'id': 'face gesture'},
                {'name': 'Face Response Count', 'id': 'face response count'},
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Face, Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 49
        conclusion_message = f'To express agreement, common gestures include nodding the head and giving a thumbs up with the right hand.'
       
    elif selected_question == 50:
        extra_section_style = {'display': 'block'}
        # Load datasets for face, hand, and body
        df_face = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_face_4.csv'))
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_4.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_4.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_face = df_face['Suggested phrase or words'].value_counts(normalize=True) * 100
        answer_counts_hand = df_hand['Suggested phrase or words.1'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested Phrase or words.2'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (face, face_percentage), (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_face.items(), answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'face gesture': face,
                'face response count': f"{face_percentage:.2f}%",
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Face Gesture', 'id': 'face gesture'},
                {'name': 'Face Response Count', 'id': 'face response count'},
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Face, Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 48
        conclusion_message = f'To show respect, common gestures include a slight bow of the head, a greeting or handshake using the right hand supported by the left, and bowing, which is the most frequent body gesture.'

    elif selected_question == 51:
        extra_section_style = {'display': 'block'}
        # Load datasets for face, hand, and body
        df_face = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_face_5.csv'))
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_5.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_5.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_face = df_face['Suggested phrase or words'].value_counts(normalize=True) * 100
        answer_counts_hand = df_hand['Suggested phrase or words.1'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested Phrase or words.2'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (face, face_percentage), (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_face.items(), answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'face gesture': face,
                'face response count': f"{face_percentage:.2f}%",
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Face Gesture', 'id': 'face gesture'},
                {'name': 'Face Response Count', 'id': 'face response count'},
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Face, Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 51
        conclusion_message = f'To express friendliness, people commonly use facial gestures like smiling, hand gestures such as a handshake using both hands or the right hand, and body gestures like hugging.'
    
    elif selected_question == 52:
        extra_section_style = {'display': 'block'}
        # Load datasets for face, hand, and body
        df_face = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_face_6.csv'))
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_6.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_6.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_face = df_face['Suggested phrase or words'].value_counts(normalize=True) * 100
        answer_counts_hand = df_hand['Suggested phrase or words.1'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested Phrase or words.2'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (face, face_percentage), (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_face.items(), answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'face gesture': face,
                'face response count': f"{face_percentage:.2f}%",
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Face Gesture', 'id': 'face gesture'},
                {'name': 'Face Response Count', 'id': 'face response count'},
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Face, Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 52
        conclusion_message = f'When expressing confusion, individuals typically use facial gestures like wrinkling or frowning the brow or tilting the head, hand gestures such as raising both hands or the right hand, and body movements that vary according to the situation.'
    
    elif selected_question == 53:
        extra_section_style = {'display': 'block'}
        # Load datasets for face, hand, and body
        df_face = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_face_7.csv'))
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_7.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_7.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_face = df_face['Suggested phrase or words'].value_counts(normalize=True) * 100
        answer_counts_hand = df_hand['Suggested phrase or words.1'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested Phrase or words.2'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (face, face_percentage), (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_face.items(), answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'face gesture': face,
                'face response count': f"{face_percentage:.2f}%",
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Face Gesture', 'id': 'face gesture'},
                {'name': 'Face Response Count', 'id': 'face response count'},
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Face, Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 53
        conclusion_message = f'When expressing comprehension, individuals typically use head gestures, such as nodding, hand gestures like a right-hand thumbs-up, and body gestures that vary by situation'

    elif selected_question == 54:
        extra_section_style = {'display': 'block'}
        # Load datasets for face, hand, and body
        df_face = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_face_8.csv'))
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_8.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_8.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_face = df_face['Suggested phrase or words'].value_counts(normalize=True) * 100
        answer_counts_hand = df_hand['Suggested phrase or words.1'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested Phrase or words.2'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (face, face_percentage), (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_face.items(), answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'face gesture': face,
                'face response count': f"{face_percentage:.2f}%",
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Face Gesture', 'id': 'face gesture'},
                {'name': 'Face Response Count', 'id': 'face response count'},
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Face, Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 53
        conclusion_message = f'When expressing interest, nodding and smiling are the most common gestures, while hand gestures such as giving a thumbs up with the right hand and body gestures like facing someone are used less frequently.'
    
    elif selected_question == 55:
        extra_section_style = {'display': 'block'}
        # Load datasets for face, hand, and body
        df_face = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_face_9.csv'))
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_9.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_9.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_face = df_face['Suggested phrase or words'].value_counts(normalize=True) * 100
        answer_counts_hand = df_hand['Suggested phrase or words.1'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested Phrase or words.2'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (face, face_percentage), (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_face.items(), answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'face gesture': face,
                'face response count': f"{face_percentage:.2f}%",
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Face Gesture', 'id': 'face gesture'},
                {'name': 'Face Response Count', 'id': 'face response count'},
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Face, Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 55
        conclusion_message = f''
    
    elif selected_question == 56:
        extra_section_style = {'display': 'block'}
        # Load datasets for hand and body
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_10.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_10.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_hand = df_hand['Suggested phrase or words'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested phrase or words.1'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 56
        conclusion_message = f'One should use body and hand gestures while speaking to someone, which depends on the situation. The most recommended gestures are slight body movement and slightly moving both hands.'
    
    elif selected_question == 57:
        extra_section_style = {'display': 'block'}
        # Load datasets for hand and body
        df_hand = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_hand_11.csv'))
        df_body = pd.read_csv(os.path.join(file_path_data_folder, 'sdf_body_11.csv'))

        # Calculate the normalized value counts for each dataset
        answer_counts_hand = df_hand['Suggested phrase or words'].value_counts(normalize=True) * 100
        which_hand_counts = df_hand['Hand'].value_counts(normalize=True) * 100
        answer_counts_body = df_body['Suggested phrase or words.1'].value_counts(normalize=True) * 100

        # Combine the results into a single table structure (for face, hand, and body)
        table_data_face_hand_body = []
        for (hand, hand_percentage), (body, body_percentage) in zip(
            answer_counts_hand.items(), answer_counts_body.items()
        ):
            table_data_face_hand_body.append({
                'hand gesture': hand,
                'hand response count': f"{hand_percentage:.2f}%",
                'body gesture': body,
                'body response count': f"{body_percentage:.2f}%"
            })

        # Create the table data for "which hand"
        table_data_which_hand = []
        for which_hand, which_hand_percentage in which_hand_counts.items():
            table_data_which_hand.append({
                'which hand': which_hand,
                'which hand response count': f"{which_hand_percentage:.2f}%"
            })

        # Define the DataTable for face, hand, and body gestures
        table_face_hand_body = dash_table.DataTable(
            columns=[
                {'name': 'Hand Gesture', 'id': 'hand gesture'},
                {'name': 'Hand Response Count', 'id': 'hand response count'},
                {'name': 'Body Gesture', 'id': 'body gesture'},
                {'name': 'Body Response Count', 'id': 'body response count'}
            ],
            data=table_data_face_hand_body,
            style_table={'height': '400px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Define the DataTable for "which hand"
        table_which_hand = dash_table.DataTable(
            columns=[
                {'name': 'Which Hand', 'id': 'which hand'},
                {'name': 'Which Hand Response Count', 'id': 'which hand response count'}
            ],
            data=table_data_which_hand,
            style_table={'height': '200px', 'overflowY': 'auto'},
            style_header={'backgroundColor': '#FF7070', 'color': 'white', 'fontWeight': 'bold'},
            style_cell={'textAlign': 'left', 'padding': '5px'}
        )

        # Render the two tables in your layout
        extra_table_children = html.Div([
            html.H4('Hand, and Body Gestures Table'),
            table_face_hand_body,
            html.Br(),
            html.H4('Which Hand Table'),
            table_which_hand
        ])

        # Set conclusion message for question 56
        conclusion_message = f''

    return extra_section_style, extra_table_children, conclusion_message

