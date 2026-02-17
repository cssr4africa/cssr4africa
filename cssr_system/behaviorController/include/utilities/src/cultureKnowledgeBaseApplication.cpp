/* cultureKnowledgeBaseApplication.cpp  Application source code to demonstrate how to instantiate and use the culture knowledge base helper class: CultureKnowledgeBase
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
 * It provides two public methods to access the knowledge:
 * 
 *    getValue()
 *    printToScreen()
 *
 * The knowledge base is built automatically by the constructor when the CultureKnowledgeBase class is instantiated as an object, 
 * reading the key-value types and the key-value pairs from the files specified in the configuration file.
 * The destructor deletex the dictionary data structure.
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
 * knowledgeBase    | cultureKnowledgeBase.dat
 * valueTypes       | cultureKnowledgeValueTypes.dat
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
 * cultureKnowledgeBase.dat
 * 
 * This file specifies the cultural knowledge key-value pairs.  
 * It is specified using the knowledgeBase key in the configuration file.
 *
 * cultureKnowledgeValueTypes.dat
 *
 * This file specifies the types of the values in each key-value pair.
 * It is specified using the valuesType key in the configuration file.
 *
 *
 * Output Data Files
 * ----------------
 * None
 *
 *
 * Configuration Files
 * -------------------
 * cultureKnowledgeBaseConfiguration.ini
 *
 * This is the file that contains the configuration file parameters specified above.
 *
 *
 *
 * Example Instantiation of the Module
 * -----------------------------------
 * Either do
 *    rosrun utilities cultureKnowledgeBaseExample
 * or
 *    roslaunch utilities cultureknowledgBaseExample.launch
 *
 * The example application in cultureKnowledgeBaseApplication.cpp illustrates the use of an object instantiation of the class 
 * to read the culture knowledge base file,  build the knowledge base (implemented using a binary search tree dictionary data structure),  
 * and use the public access method printToScreen() to print each key-value pair, along with its value type and initialization flag.  
 * It also provides four examples of how to retrieve the values of keys using the getValue() access method, each one with a different value type, 
 * and write the associated values to the terminal.
 *
 * Note that it uses two utility functions, valueType2Alphanumberic and initialized 2Alphanumeric,
 * which convert the integer value type valueType and the Boolean initialized value to their equivalent
 * alphanumberic strings for ease of interpretation when reporting them on screen.
 *
 *
 * Author:  David Vernon, Carnegie Mellon University Africa
 * Email:   dvernon@andrew.cmu.edu
 * Date:    28 February 2025
 * Version: v1.0
 *
 */
 
#include <utilities/cultureKnowledgeBaseInterface.h>
using namespace Culture;

int main() {

   KeyValueType keyValue; // structure with key, value, value type, initialization flag
   Keyword      key;      // string
   int          i;        // counter
   
   /* instantiate the cultural knowledge base object                         */
   /* this reads the knowledge value types file and the knowledge base file  */
   /* as specified in the culturalKnowledgeBaseConfiguration.ini file        */
  
   CultureKnowledgeBase knowledgebase;  

   /* verify that the knowledge base was read correctly */
   
   knowledgebase.printToScreen();
  
   /* query the contents off the knowledge base */
   /* retrieve example of the four value types  */

   for (i=0; i<4; i++) {
      switch (i) {
         case UNDEFINED: strcpy(key,"ExpressionSpeed");    
                         break;
	  
         case NUMBER:    strcpy(key,"PassingDistance");         
                         break;
		
         case WORD:      strcpy(key,"PassingPositionAvoid");    
                         break;
		
         case PHRASE:    strcpy(key,"PhrasePreambleEnglish"); 
 	                 break;
      }
     
      if (knowledgebase.getValue(key, &keyValue) == true) {
      
         switch (keyValue.valueType) {

	    case UNDEFINED: printf("main: attempting to retrieve a value for a key-value pair with an UNDEFINED value type\n");
                            break;

	    case NUMBER:    printf("main: %-40s %-40d %-20s %-20s\n", keyValue.key,
				                                      keyValue.integerValue,
				                                      valueType2Alphanumeric(keyValue.valueType),
				                                      initialized2Alphanumeric(keyValue.initialized));
                            break;

	    case WORD:      printf("main: %-40s %-40s %-20s %-20s\n", keyValue.key,
				                                      keyValue.alphanumericValue,
				                                      valueType2Alphanumeric(keyValue.valueType), 
				                                      initialized2Alphanumeric(keyValue.initialized));
                            break;

            case PHRASE:    printf("main: %-40s %-40s %-20s %-20s\n", keyValue.key,
				                                      keyValue.alphanumericValue,
				                                      valueType2Alphanumeric(keyValue.valueType),
				                                      initialized2Alphanumeric(keyValue.initialized));
                            break;
			    
            default:        printf("main: invalid value type %d\n",   keyValue.valueType);
                            break;
         }
      }
      else {
	 printf("main: invalid key %s\n", key);
      }
   }
}


