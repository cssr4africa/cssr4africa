/* environmentKnowledgeBaseApplication.cpp  Application source code to demonstrate how to instantiate and use the environment knowledge base helper class: EnvironmentKnowledgeBase
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


/*
 * This software implements a C++ helper class to read the culture knowledge base file and query the knowledge through access methods.
 * The helper class uses a binary search tree (BST) dictionary to represent the knowledge base.
 * It provides three public methods to access the knowledge:
 * 
 *    getValue()
 *    getTour()
 *    printToScreen()
 *
 * The knowledge base is built automatically by the constructor when the EnvironmentKnowledgeBase class is instantiated as an object, 
 * reading the key-value types and the key-value pairs from the files specified in the configuration file.
 * The destructor deletes the dictionary data structure.
 *
 *
 * Libraries
 * ---------
 * stdio
 * string
 * stdlib
 *
 * ROS libraries
 * -------------
 * None
 *
 *
 * Parameters
 * ----------
 * None.
 *
 *
 * Command-line Parameters
 * ----------------------
 * None
 *
 *
 * Configuration File Parameters 
 * -----------------------------
 *
 * Key              | Value 
 * knowledgeBase    | environmentKnowledgeBase.dat
 * verboseMode      | false
 *
 *
 * Subscribed Topics and Message Types
 * -----------------------------------
 * None
 *
 *
 * Published Topics and Message Types
 * ----------------------------------
 * None
 *
 *
 * Advertised Service 
 * ------------------
 * None
 * 
 *
 * Invoked Service
 * --------------
 * None.
 *
 *
 * Input Data Files
 * ----------------
 * environmentKnowledgeBase.dat
 * 
 * This file specifies the environment knowledge key-value pairs.  
 * It is specified using the knowledgeBase key in the configuration file.
 *
 *
 * Output Data Files
 * ----------------
 * None
 *
 *
 * Configuration Files
 * -------------------
 * environmentKnowledgeBaseConfiguration.ini
 *
 * This is the file that contains the configuration file parameters specified above.
 *
 *
 *
 * Example Instantiation of the Module
 * -----------------------------------
 * Either do
 *    rosrun utilities environmentKnowledgeBaseExample
 * or
 *    roslaunch utilities environmentKnowledgBaseExample.launch
 *
 * The example application in environmentKnowledgeBaseApplication.cpp illustrates the use of an object instantiation of the class 
 * to read the environment knowledge base file,  build the knowledge base (implemented using a binary search tree dictionary data structure),  
 * and use the public access method printToScreen() to print each key-value pair, along with its value type and initialization flag.  
 * It also provides four examples of how to retrieve the values of keys using the getValue() access method, each one with a different value type, 
 * and write the associated values to the terminal.
 *
 * Author:   David Vernon, Carnegie Mellon University Africa
 * Email:    dvernon@andrew.cmu.edu
 * Date:     18 March 2025
 * Version:  v1.1
 * Revision: Added CulturalKnowledge key-value pair, with multiple cultural knowledge keys as the elements of the value
 *
 */
 
#include <utilities/environmentKnowledgeBaseInterface.h>

int main() {

   KeyValueType          keyValue; // structure with key and values
   TourSpecificationType tour;     // list of tour locations
   int                   idNumber; // location id
   int                   i;        // counter
   int                   k;        // counter

   /* instantiate the environment knowledge base object                      */
   /* this reads the knowledge value types file and the knowledge base file  */
   /* as specified in the environmentKnowledgeBaseConfiguration.ini file     */
  
   EnvironmentKnowledgeBase knowledgebase;  

   /* verify that the knowledge base was read correctly */

   printf("main: the environment knowledge base data:\n");
   printf("------------------------------------------\n\n");
   
   knowledgebase.printToScreen();


   printf("main: the environment knowledge base tour:\n");
   printf("-----------------------------------------\n\n");

   knowledgebase.getTour(&tour);
  
   /* query the contents of the knowledge base:               */
   /* retrieve all the locations on a tour                    */
   /* and print them in the order in which they are specified */

   for (i = 0; i <= tour.numberOfLocations; i++) {
      idNumber = tour.locationIdNumber[i];
      if (knowledgebase.getValue(idNumber, &keyValue) == true) {
         printf("main:\n"
	        "Key                               %-4d \n"
	        "Location Description              %s \n"
                "Robot Location                    (%.1f, %.1f  %.1f) \n"
                "Gesture Target                    (%.1f, %.1f  %.1f) \n"
                "Pre-Gesture Message English       %s \n"
	        "Pre-Gesture Message isiZulu       %s \n"
                "Pre-Gesture Message Kinyarwanda   %s \n"
	        "Post-Gesture Message English      %s \n"
	        "Post-Gesture Message isiZulu      %s \n"
                "Post-Gesture Message Kinyarwanda  %s \n",
	        keyValue.key,
	        keyValue.robotLocationDescription, 
                keyValue.robotLocation.x, keyValue.robotLocation.y, keyValue.robotLocation.theta,
	        keyValue.gestureTarget.x, keyValue.gestureTarget.y, keyValue.gestureTarget.z,
	        keyValue.preGestureMessageEnglish,
	        keyValue.preGestureMessageIsiZulu,
	        keyValue.preGestureMessageKinyarwanda,
		keyValue.postGestureMessageEnglish,
		keyValue.postGestureMessageIsiZulu,
		keyValue.postGestureMessageKinyarwanda);

	 printf("Cultural Knowledge                ");
         for (k=0; k<keyValue.culturalKnowledge.numberOfKeys; k++) {
            printf("%s ", keyValue.culturalKnowledge.key[k]);
         }
         printf("\n\n");
      }
   }
}


