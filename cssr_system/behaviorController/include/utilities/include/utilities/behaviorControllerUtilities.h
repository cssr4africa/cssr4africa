/* behaviorControllerUtilities.h   Interface source code for the utility functions
 *
 * Copyright (C) 2023 CSSR4Africa Consortium
 *
 * This project is funded by the African Engineering and Technology Network (Afretec)
 * Inclusive Digital Transformation Research Grant Programme.
 *
 * Website: www.cssr4africa.org
 *
 * This program comes with ABSOLUTELY NO WARRANTY.
 *
 */
 
#include <stdio.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef BEHAVIOR_CONTROLLER_UTILITIES_INTERFACE_H
#define BEHAVIOR_CONTROLLER_UTILITIES_INTERFACE_H

#define NUMBER_OF_CONFIGURATION_KEYS   3
#define MAX_STRING_LENGTH            300

/***************************************************************************************************************************
 
   Utility function prototypes 
   
****************************************************************************************************************************/


/* print message passed as argument and take appropriate action */

int error(char *s);

/* prompt the user to exit */

void prompt_and_exit(int status);

/* print a message to a specified file */

void print_message_to_file(FILE *fp, char message[]);


#endif