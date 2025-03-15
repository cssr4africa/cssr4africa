"""
dashboard_questions_data.py

This module contains multiple-choice questions from the CSSR4Africa Questionnaire Survey.
The questions are stored as dictionaries in this file to be used in the dashboard.

Purpose:
- These questions are used to generate bar graphs in the dashboard.
- The stored dictionary allows for easy extraction of value counts for each question.
- The options serve as dynamic labels for the x-axis in the bar graph.

Each question is stored as a key in the dictionary, and its corresponding answer choices 
are provided as a list under the key 'options'. This ensures that the dashboard can 
efficiently categorize and visualize survey responses.

Author:   Eyerusalem Mamuye Birhan
Date:     2025-03-13
Version:  1.0

Copyright (C) 2023 CSSR4Africa Consortium
This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.
Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""


questions_dict = {
    "1. To show respect, one should lower gaze when greeting someone older.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "2. One should suspend work or movements and pay attention when addressed.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "3. One should keep intermittent eye contact; lack of eye contact depicts disrespect as it shows divided attention during the interaction.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "4. One should not make persistent eye contact with an older person.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "5. One should not make eye contact when being corrected by someone.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "6. One should use an open palm of the hand to point to people and objects.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "7. One should not point an upward facing palm of the hand at someone.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "8. One should not use the left hand to point to anything.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "9. To show respect, one should bow slightly when greeting someone older.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "10. To show respect, one should raise both hands when greeting.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    
    "11. One should not wave at someone from a distance; one should move towards them to greet them.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "12. One should not use the left hand to hand something to someone.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "13. To show respect, one should hand over and accept gifts with two hands and do so from the front, facing the recipient.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "14. To show respect, one should shake hands with the right hand and use the left arm to support the right forearm when doing so.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "15. An appreciation of rhythmic sound and movement is valued.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "16. To show respect, one should bow slightly and lower gaze when greeting someone older.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "17. The younger interaction partner should bow when greeting an older person or when rendering a service.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "18. All interactions should begin with a courteous greeting.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "19. The younger interaction partner should enable a greeting to be initiated by an older person.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "20. It is respectful to use local languages and they should be used for verbal interaction when possible.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "21. One should use formal titles when addressing someone.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    
},
    "22. One should engage in a preamble before getting to the point, as being too forward may be regarded as disrespectful.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "23. One should not interrupt or talk over someone when they are speaking.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "24. One should not talk loudly to an older person.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "25. Behaviours should focus on fostering social connections and relationships; they should not be purely functional.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "26. One should not walk between two or more people who are conversing because it is considered rude to do so.": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "27. One should not walk far ahead of an older person, unless leading the person (in which case, one should walk slightly to the side).": {
        'options': [
            'Yes, this is correct.',
            'No, this is not correct.',
            'I am not sure.'
        ]
    },
    "1. What distance should you keep when passing someone?": {
        'options': [
            'Less than 1 m.',
            '1 – 2 m.',
            'More than 2 m.'
        ]
    },
    "2. How should you acknowledge someone when passing them?": {
        'options': [
            'No acknowledgement.',
            'Raise eyebrows slightly.',
            'Say hello.',
            'Other. Please specify:'
        ]
    },
    "3. How should you pass a group of two or more people?": {
        'options': [
            'Pass behind the majority.',
            'Pass between them.',
            'Pass in front of the majority.'
        ]
    },
    "4. When showing someone older than you the way, where should you position yourself?": {
        'options': [
            'Far in front of them.',
            'A little in front of them.',
            'Beside them.',
            'A little behind them.'
        ]
    },
    "5. When showing someone the same age as you the way, where should you position yourself?": {
        'options': [
            'Far in front of them.',
            'A little in front of them.',
            'Beside them.',
            'A little behind them.'
        ]
    },
    "6. When showing someone younger than you the way, where should you position yourself?": {
        'options': [
            'Far in front of them.',
            'A little in front of them.',
            'Beside them.',
            'A little behind them.'
        ]
    },
    "7. How should you address someone who is older than you and who you haven’t met before?": {
        'options': [
            'First name.',
            'Last name.',
            'Title first name.',
            'Title last name.',
            'Other. Please specify:'
        ]
    },
    "8. How should you address someone who is the same age as you and who you haven’t met before?": {
        'options': [
            'First name.',
            'Last name.',
            'Title first name.',
            'Title last name.',
            'Other. Please specify:'
        ]
    },
    "9. How should you address someone who is younger than you and who you haven’t met before?": {
        'options': [
            'First name.',
            'Last name.',
            'Title first name.',
            'Title last name.',
            'Other. Please specify:'
        ]
    },
    "12. If you are explaining something to someone, what is your primary focus of attention, i.e., where do you direct your gaze?": {
        'options': [
            'The object being explained.',
            'The face, eyes, or mouth of the person to whom you are explaining.',
            'Mostly the object and sometimes the person.',
            'Mostly the person and sometimes the object.',
            'Equally the person and the object.'
        ]
    },
    "13. If you are explaining something to someone, how often should you make eye contact?": {
        'options': [
            'Never.',
            'Occasionally.',
            'Often.',
            'Constantly.'
        ]
    },
    "14. If you are explaining something to someone, how often would you make eye contact if the person was older than you?": {
        'options': [
            'Less often.',
            'More often.',
            'No difference.'
        ]
    },
    "15. If you are explaining something to someone, how often would you make eye contact if the person was younger than you?": {
        'options': [
            'Less often.',
            'More often.',
            'No difference.'
        ]
    },
    "16. If someone is explaining something to you, what is your primary focus of attention, i.e., where do you direct your gaze?": {
        'options': [
            'The object being explained.',
            'The face, eyes, or mouth of the person to whom you are explaining.',
            'Mostly the object and sometimes the person.',
            'Mostly the person and sometimes the object.',
            'Equally the person and the object.'
        ]
    },
    "17. If someone is explaining something to you, how often should you make eye contact?": {
        'options': [
            'Never.',
            'Occasionally.',
            'Often.',
            'Constantly.'
        ]
    },
    "18. If someone is explaining something to you, how often would you make eye contact if the person was older than you?": {
        'options': [
            'Less often.',
            'More often.',
            'No difference.'
        ]
    },
    "19. If someone is explaining something to you, how often would you make eye contact if the person was younger than you?": {
        'options': [
            'Less often.',
            'More often.',
            'No difference.'
        ]
    }
}
