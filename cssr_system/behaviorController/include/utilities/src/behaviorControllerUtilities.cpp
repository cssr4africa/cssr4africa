/* behaviorControllerUtilities.cpp   Source code for the utility functions common to both knowledge bases
 *  
 * Date: April 17, 2025
 * Version: 1.0
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

#include <utilities/behaviorControllerUtilities.h>
/*
 * Utility functions 
 * ================
 */


/*
 * int error(char *s)
 * ------------------
 *
 * Print an error message to the terminal 
 */

int error(char *s) {

   printf("Error: %s\n",s);

   exit(0);
}


/*
 * void print_message_to_file(FILE *fp, char message[]) 
 * ----------------------------------------------------
 *
 * Print a message in a file 
 */

void print_message_to_file(FILE *fp, char message[]) {
 
   fprintf(fp,"The message is: %s\n", message);
} 



/*
 * void prompt_and_exit(int status)
 * --------------------------------
 *
 * Prompt the user and exit when they press a key
 */

void prompt_and_exit(int status) {
   printf("Press any key to continue\n");
   getchar();
   exit(status);
}