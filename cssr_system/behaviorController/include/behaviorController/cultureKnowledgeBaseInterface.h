/* cultureKnowledgeBaseInterface.h   Interface source code for the culture knowledge base helper class: CultureKnowledgeBase
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
#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

using namespace std;

/***************************************************************************************************************************

   ROS package name

****************************************************************************************************************************/
// #ifndef ROS_PACKAGE_NAME
//   #define ROS_PACKAGE_NAME "utilities"
// #endif


/***************************************************************************************************************************

   General purpose definitions 

****************************************************************************************************************************/
#define TRUE                           1
#define FALSE                          0
#define MAX_FILENAME_LENGTH          200
#define STRING_LENGTH                200
#define KEY_LENGTH                   100


/***************************************************************************************************************************
 
   Class CultureKnowledgeBase
   
   A class to represent the culture knowledge base as a binary search tree dictionary

****************************************************************************************************************************/

#define NUMBER_OF_VALUE_TYPES          4
#define NUMBER_OF_CONFIGURATION_KEYS   3
#define MAX_STRING_LENGTH            200

/* constant definitions for valueType flag to identify which element of the union is to be used */

#define UNDEFINED 0 // value hasn't been initialized
#define NUMBER    1 // value is integer
#define WORD      2 // value is alphanumeric but just one word
#define PHRASE    3 // value is alphanumeric but several words


#ifndef CULTURE_KNOWLEDGE_BASE_INTERFACE_H
#define CULTURE_KNOWLEDGE_BASE_INTERFACE_H
namespace Culture{
typedef char Keyword[KEY_LENGTH];

typedef struct {
   char knowledgeBase[MAX_FILENAME_LENGTH];
   char valueTypes[MAX_FILENAME_LENGTH];
   bool verboseMode;
} ConfigurationDataType;

typedef  struct {
    char *key;
    union {
        int   integerValue;
        char* alphanumericValue;  
    };
    int valueType;
    bool initialized;
} KeyValueType;

typedef struct node *NodeType;

typedef struct node {
            KeyValueType keyValue;
            NodeType left, right;
         } Node;

typedef NodeType BinaryTreeType;

typedef BinaryTreeType WindowType;

class CultureKnowledgeBase {

public:
   CultureKnowledgeBase();
   ~CultureKnowledgeBase();
   
   bool                  getValue(char *key, KeyValueType *keyValue);                         // return true if the key is in the dictionary; false otherwise  - abstract version
   void                  printToScreen();                                                     // print all elements in a BST dictionary by traversing inorder  - abstract version

private:
   KeyValueType          keyValue;
   BinaryTreeType        tree = NULL; 
   ConfigurationDataType configurationData;
   char                  configuration_filename[MAX_STRING_LENGTH] = "cultureKnowledgeBaseConfiguration.ini";

   void                  assign_key_attributes(KeyValueType *keyValue, char key[], int valueType, bool operational); // assign a key, value, value type, and operational flag to a key-value pair
   void                  assign_key_value(KeyValueType *keyValue, int integerValue,  bool operational);              // assign an integer value to a key-value pair
   void                  assign_key_value(KeyValueType *keyValue, char *alphanumericValue, bool operational);        // assign a string value to a key-value pair
   BinaryTreeType        *delete_element(KeyValueType keyValue, BinaryTreeType *tree);                               // delete a key-value pair in a BST 
   KeyValueType          delete_min(BinaryTreeType *tree);                                                           // return & delete the smallest node in a BST (i.e. the left-most node)
   bool                  exists(char *key);                                                                          // return true if the key is in the dictionary; false otherwise  - abstract version
   bool                  exists(KeyValueType keyValue, BinaryTreeType *tree);                                        // return true if the key is in the BST; false otherwise
   int                   height();                                                                                   // return the height of a BST                                    - abstract version
   int                   height(BinaryTreeType tree, int n);                                                         // return the height of a BST
   int                   getValueType(char *key);                                                                    // return the integer code for the value type of the key  - abstract version
   bool                  getValue(char *key, KeyValueType *keyValue, BinaryTreeType *tree);                          // return true and key-value pair if the key is in the dictionary; false otherwise
   void                  initialize(BinaryTreeType *tree);                                                           // initialize a BST
   int                   inorder_print_to_file(BinaryTreeType tree, int n, FILE *fp_out);                            // inorder traversal of a BST, printing node elements to file 
   int                   inorder_print_to_screen(BinaryTreeType tree, int n);                                        // inorder traversal of a BST, printing node elements
   BinaryTreeType        *insert(KeyValueType keyValue, BinaryTreeType *tree, bool update);                          // insert a key-value pair in a BST. if update is true & the key exists, overwrite it
   int                   postorder_delete_nodes(BinaryTreeType tree);                                                // postorder traversal of a BST, deleting node elements 
   int                   print_to_file(FILE *fp_out);                                                                // print all elements in a BST by traversing inorder             - abstract version
   int                   print_to_file(BinaryTreeType tree, FILE *fp_out);                                           // print all key-value pairs in a BST by traversing inorder
   int                   print_to_screen(BinaryTreeType tree);                                                       // print all elements in a BST by traversing inorder 
   int                   size(BinaryTreeType tree);                                                                  // returns the size of a BST 
   void                  readConfigurationData();                                                                    // read configuration parameters key-value pairs from the configuration file 
   void                  readKnowledgeBase();                                                                        // read knowledge base key-value pairs from the data file
   void                  readKnowledgeBaseValueTypes();                                                              // read the types of the values in the knowledge base key-value pairs from the data file
   int                   size();                                                                                     // returns the size of a BST                                     - abstract version
   int                   total_number_of_probes();                                                                   // returns the total number of probes                            - abstract version
   int                   total_number_of_probes(BinaryTreeType tree, int n);                                         // returns the total number of probes
};




/***************************************************************************************************************************
 
   Utility function prototypes 
   
****************************************************************************************************************************/

/* convert integer valueType to alphnumeric */

char *valueType2Alphanumeric(int valueType);

/* convert Boolean operational to alphnumeric */

char *initialized2Alphanumeric(bool initialized);

/* print message passed as argument and take appropriate action */

int error(char *s);

/* prompt the user to exit */

void prompt_and_exit(int status);

/* print a message to a specified file */

void print_message_to_file(FILE *fp, char message[]);
}
#endif