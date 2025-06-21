/* environmentKnowledgeBaseInterface.h   Interface source code for the environment knowledge base helper class: EnvironmentKnowledgeBase
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
#define MAX_STRING_LENGTH            200
#define MAX_FILENAME_LENGTH          200
#define STRING_LENGTH                200
#define KEY_LENGTH                   100
#define MAX_CULTURE_KEYS               2
#define NUMBER_OF_CONFIGURATION_KEYS   3


/***************************************************************************************************************************
 
   Class EnvironmentKnowledgeBase
   
   A class to represent the environment knowledge base as a binary search tree dictionary

****************************************************************************************************************************/
#define NUMBER_OF_VALUE_KEYS          13
#define MAX_NUMBER_OF_TOUR_LOCATIONS  20


#ifndef ENVIRONMENT_KNOWLEDGE_BASE_INTERFACE_H
#define ENVIRONMENT_KNOWLEDGE_BASE_INTERFACE_H

namespace Environment{
typedef char Keyword[KEY_LENGTH];

typedef struct {
   char knowledgeBase[MAX_FILENAME_LENGTH];
   bool verboseMode;
} ConfigurationDataType;

typedef struct {
   float x;
   float y;
   float theta;
} RobotLocationType;

typedef struct {
   float x;
   float y;
   float z;
} GestureTargetType;

typedef struct {
   int numberOfLocations;
   int locationIdNumber[MAX_NUMBER_OF_TOUR_LOCATIONS];
} TourSpecificationType;

typedef struct {
    char* key[MAX_CULTURE_KEYS];
    int numberOfKeys;
} CulturalKnowledgeType;

typedef  struct {
    int                   key; // i.e., idNumber
    RobotLocationType     robotLocation;
    GestureTargetType     gestureTarget;
    CulturalKnowledgeType   culturalKnowledge;
    char                  robotLocationDescription[STRING_LENGTH];
    char                  preGestureMessageEnglish[STRING_LENGTH];
    char                  preGestureMessageIsiZulu[STRING_LENGTH];
    char                  preGestureMessageKinyarwanda[STRING_LENGTH];
    char                  postGestureMessageEnglish[STRING_LENGTH];
    char                  postGestureMessageIsiZulu[STRING_LENGTH];
    char                  postGestureMessageKinyarwanda[STRING_LENGTH];
} KeyValueType;

typedef struct node *NodeType;

typedef struct node {
            KeyValueType keyValue;
            NodeType left, right;
         } Node;

typedef NodeType BinaryTreeType;

typedef BinaryTreeType WindowType;

class EnvironmentKnowledgeBase {

public:
   EnvironmentKnowledgeBase();
   ~EnvironmentKnowledgeBase();
   
   bool                  getValue(int key, KeyValueType *keyValue);                           // return true if the key is in the dictionary; false otherwise  - abstract version
   bool                  getTour(TourSpecificationType  *tour);                               // return true if the tour has one or more locations; false otherwise  - abstract version
   void                  printToScreen();                                                     // print all elements in a BST dictionary by traversing inorder  - abstract version

private:
   BinaryTreeType        tree = NULL; 
   TourSpecificationType tourSpecification;
   KeyValueType          keyValue;
   ConfigurationDataType configurationData;
   char                  configuration_filename[MAX_STRING_LENGTH] = "environmentKnowledgeBaseConfiguration.ini";

   BinaryTreeType        *delete_element(KeyValueType keyValue, BinaryTreeType *tree);        // delete a key-value pair in a BST 
   KeyValueType          delete_min(BinaryTreeType *tree);                                    // return & delete the smallest node in a BST (i.e. the left-most node)
   bool                  getValue(int key, KeyValueType *keyValue, BinaryTreeType *tree);     // return true and key-value pair if the key is in the dictionary; false otherwise
   void                  initialize(BinaryTreeType *tree);                                    // initialize a BST
   int                   inorder_print_to_file(BinaryTreeType tree, int n, FILE *fp_out);     // inorder traversal of a BST, printing node elements to file 
   int                   inorder_print_to_screen(BinaryTreeType tree, int n);                 // inorder traversal of a BST, printing node elements
   BinaryTreeType        *insert(KeyValueType keyValue, BinaryTreeType *tree, bool update);   // insert a key-value pair in a BST. if update is true & the key exists, overwrite it
   int                   postorder_delete_nodes(BinaryTreeType tree);                         // postorder traversal of a BST, deleting node elements 
   int                   print_to_file(FILE *fp_out);                                         // print all elements in a BST by traversing inorder             - abstract version
   int                   print_to_file(BinaryTreeType tree, FILE *fp_out);                    // print all key-value pairs in a BST by traversing inorder
   int                   print_to_screen(BinaryTreeType tree);                                // print all elements in a BST by traversing inorder 
   void                  readConfigurationData();                                             // read configuration parameters key-value pairs from the configuration file 
   void                  readKnowledgeBase();                                                 // read knowledge base key-value pairs from the data file
};


/***************************************************************************************************************************
 
   Utility function prototypes 
   
****************************************************************************************************************************/

/* print message passed as argument and take appropriate action */

int error(char *s);

/* prompt the user to exit */

void prompt_and_exit(int status);

/* print a message to a specified file */

void print_message_to_file(FILE *fp, char message[]);


}
#endif