/* environmentKnowledgeBaseImplementation.cpp   Source code for the implementation of the environment knowledge base helper class: EnvironmentKnowledgeBase
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
 * Author:   David Vernon, Carnegie Mellon University Africa
 * Email:    dvernon@andrew.cmu.edu
 * Date:     18 March 2025
 * Version:  v1.1
 * Revision: Added CulturalKnowledge key-value pair, with multiple cultural knowledge keys as the elements of the value
 *
 *
 * Author:   David Vernon, Carnegie Mellon University Africa
 * Email:    dvernon@andrew.cmu.edu
 * Date:     14 April 2025
 * Version:  v1.2
 * Revision: Updated readKnowledgeBase() to use camel case keys
 *
 */

 
#include <behaviorController/environmentKnowledgeBaseInterface.h>

namespace Environment{
/*
 * EnvironmentKnowledgeBase::EnvironmentKnowledgeBase
 * --------------------------------------------------
 *
 * Class constructor
 *   Read the configuration data from the configuration file
 *   Read the knowledge base value types and build the binary search tree dictionary data structure
 *   Read the knowledge base value and update the binary search tree dictionary data structure 
 */

EnvironmentKnowledgeBase::EnvironmentKnowledgeBase() {

   readConfigurationData();

   if (configurationData.verboseMode) printf("EnvironmentKnowledgeBase()\n");

   initialize(&tree); // need to delete nodes if tree is not empty

   tourSpecification.numberOfLocations = 0;

   readKnowledgeBase();
}



/*
 * EnvironmentKnowledgeBase::~EnvironmentKnowledgeBase
 * ---------------------------------------------------
 *
 * Class destructor
 *    Delete all nodes in the binary search tree dictionary data structure
 */

EnvironmentKnowledgeBase::~EnvironmentKnowledgeBase() {

   if (configurationData.verboseMode) printf("~EnvironmentKnowledgeBase()\n");

   postorder_delete_nodes(tree);

}



/*
 * BinaryTreeType *EnvironmentKnowledgeBase::delete_element(KeyValueType keyValue, BinaryTreeType *tree)
 * -----------------------------------------------------------------------------------------------------
 *
 * Delete a key-value pair in a BST 
 */

BinaryTreeType *EnvironmentKnowledgeBase::delete_element(KeyValueType keyValue, BinaryTreeType *tree) {

   BinaryTreeType p;

   if (*tree != NULL) {

     if (keyValue.key < (*tree)->keyValue.key)  // keyValue.key is the key
         delete_element(keyValue, &((*tree)->left));        

      else if (keyValue.key > (*tree)->keyValue.key)
         delete_element(keyValue, &((*tree)->right));

      else if (((*tree)->left == NULL) && ((*tree)->right == NULL)) {

         /* leaf node containing e - delete it */

         p = *tree;
         free(p);
         *tree = NULL;
      } 
      else if ((*tree)->left == NULL) {

         /* internal node containing e and it has only a right child */
         /* delete it and make treepoint to the right child          */
         
         p = *tree;
         *tree = (*tree)->right;
         free(p);
      }
      else if ((*tree)->right == NULL) {

         /* internal node containing e and it has only a left child */
         /* delete it and make treepoint to the left child          */
         
         p = *tree;
         *tree = (*tree)->left;
         free(p);
      }
      else {

         /* internal node containing e and it has both left and right child */
         /* replace it with leftmost node of right sub-tree                 */      
         (*tree)->keyValue = delete_min(&((*tree)->right));
      }
   }     
   return(tree);
}



/*
 * KeyValueType EnvironmentKnowledgeBase::delete_min(BinaryTreeType *tree) 
 * -----------------------------------------------------------------------
 *
 * Return & delete the smallest node in a tree (i.e. the left-most node)
 */

KeyValueType EnvironmentKnowledgeBase::delete_min(BinaryTreeType *tree) {

   KeyValueType keyValue;
   BinaryTreeType p;

   if ((*tree)->left == NULL) {

      /* tree points to the smallest element */
      
      keyValue = (*tree)->keyValue;
      
      /* replace the node pointed to by tree by its right child */

      p = *tree;
      *tree = (*tree)->right;
      free(p);

      return(keyValue);
   }
   else {

      /* the node pointed to by tree has a left child */

      return(delete_min(&((*tree)->left)));
   }

}



/*
 * bool EnvironmentKnowledgeBase::getTour(TourSpecificationType  *tour)
 * -------------------------------------------------------------------- 
 *
 * return true if the tour has one or more locations; false otherwise  - abstract version
 */

bool EnvironmentKnowledgeBase::getTour(TourSpecificationType  *tour) {

  int i;
  
  if (tourSpecification.numberOfLocations > 0) {
     tour->numberOfLocations = tourSpecification.numberOfLocations;

     for (i=0; i<tourSpecification.numberOfLocations; i++) {
        tour->locationIdNumber[i] = tourSpecification.locationIdNumber[i];
     }

     return (true);
  }
  else {
     tour->numberOfLocations = 0;

     return (false);
  }
}


/*
 * bool EnvironmentKnowledgeBase::getValue(int key, KeyValueType *keyValue)
 * -------------------------------------------------------------------------- 
 *
 * Return true if the key is in the tree; false otherwise - abstract version
 */

bool EnvironmentKnowledgeBase::getValue(int key, KeyValueType *keyValue) {

   return(getValue(key, keyValue, &tree));
}




/*
 * bool EnvironmentKnowledgeBase::getValue(char *key, KeyValueType *keyValue,  BinaryTreeType *tree)
 * -------------------------------------------------------------------------------------------------
 *
 * Return true if the key is in the tree; false otherwise 
 */

bool EnvironmentKnowledgeBase::getValue(int key, KeyValueType *keyValue,  BinaryTreeType *tree ) {

   bool found;
   int  i;

   if (*tree == NULL) {

      /* we are at an external node: the key isn't in the tree */
     
      found = false;
      
      keyValue->key = 0;
      keyValue->robotLocation.x     = 0;
      keyValue->robotLocation.y     = 0;
      keyValue->robotLocation.theta = 0;
      strcpy(keyValue->robotLocationDescription, "");
      keyValue->gestureTarget.x     = 0;
      keyValue->gestureTarget.y     = 0;
      keyValue->gestureTarget.z     = 0;
      strcpy(keyValue->preGestureMessageEnglish, "");
      strcpy(keyValue->preGestureMessageIsiZulu, "");
      strcpy(keyValue->preGestureMessageKinyarwanda, "");
      strcpy(keyValue->postGestureMessageEnglish, "");
      strcpy(keyValue->postGestureMessageIsiZulu, "");
      strcpy(keyValue->postGestureMessageKinyarwanda, "");
      keyValue->culturalKnowledge.numberOfKeys = 0;
   }
   else if ((*tree)->keyValue.key > key) { 
     found = getValue(key, keyValue, &((*tree)->left));
   }
   else if ((*tree)->keyValue.key < key) {
     found = getValue(key, keyValue, &((*tree)->right));
   }
   else if ((*tree)->keyValue.key == key) {
     
      /* found it */

      found = true;
      
      *keyValue = (*tree)->keyValue;
      /*
      keyValue->key = (*tree)->keyValue.key;
      keyValue->robotLocation.x     = (*tree)->keyValue.robotLocation.x;
      keyValue->robotLocation.y     = (*tree)->keyValue.robotLocation.y;
      keyValue->robotLocation.theta = (*tree)->keyValue.robotLocation.theta;
      strcpy(keyValue->robotLocationDescription, (*tree)->keyValue.robotLocationDescription);
      keyValue->gestureTarget.x     = (*tree)->keyValue.gestureTarget.x;
      keyValue->gestureTarget.y     = (*tree)->keyValue.gestureTarget.y;
      keyValue->gestureTarget.z     = (*tree)->keyValue.gestureTarget.z;
      strcpy(keyValue->preGestureMessageEnglish, (*tree)->keyValue.preGestureMessageEnglish);
      strcpy(keyValue->preGestureMessageisiZulu, (*tree)->keyValue.preGestureMessageIsiZulu);
      strcpy(keyValue->preGestureMessageKinyarwanda, (*tree)->keyValue.preGestureMessageKinyarwanda);
      strcpy(keyValue->postGestureMessageEnglish, (*tree)->keyValue.postGestureMessageEnglish);
      strcpy(keyValue->postGestureMessageIsiZulu, (*tree)->keyValue.postGestureMessageIsiZulu);
      strcpy(keyValue->postGestureMessageKinyarwanda, (*tree)->keyValue.postGestureMessageKinyarwanda;
      keyValue->culturalKnowledge.numberOfKeys = (*tree)->keyValue.culturalKnowledge.numberOfKeys;
      for (i=0; i<keyValue->culturalKnowledge.numberOfKeys; i++) {
          keyValue->culturalKnowledge.key[i] = (char *) malloc(sizeof(char) * (strlen((*tree)->keyValue.culturalKnowledge.key[i])+1));
          strcpy(keyValue->culturalKnowledge.key[i], (*tree)->keyValue.culturalKnowledge.key[i]);
      }
      */
   }
   return(found);
}




/*
 * void EnvironmentKnowledgeBase::initialize(BinaryTreeType *tree)
 * ----------------------------------------------------------------
 * 
 * Initialize a binary search tree (BST) 
 */

void EnvironmentKnowledgeBase::initialize(BinaryTreeType *tree) {

   static bool first_call = true;

   /* we don't know what value *tree has when the program is launched      */
   /* so we have to be careful not to dereference it                       */
   /* if it's the first call to initialize, there is no tree to be deleted */
   /* and we just set *tree to NULL                                        */

   if (first_call) {
      first_call = false;
      *tree = NULL;
   }
   else {
      if (*tree != NULL) postorder_delete_nodes(*tree);
      *tree = NULL;
   }
}



/*
 * BinaryTreeType *EnvironmentKnowledgeBase::insert(KeyValueType keyValue,  BinaryTreeType *tree, bool update)
 * -----------------------------------------------------------------------------------------------------------
 *
 * Insert a key-value pair in a BST. if update is true & the key exists, overwrite it
 */

BinaryTreeType *EnvironmentKnowledgeBase::insert(KeyValueType keyValue,  BinaryTreeType *tree, bool update) {

   bool debug = false;
   WindowType temp;
   int i;

   if (*tree == NULL) {

      /* we are at an external node: create a new node and insert it */

      if ((temp = (NodeType) malloc(sizeof(Node))) == NULL) 
         error((char *)"function insert: unable to allocate memory");
      else {
         temp->keyValue = keyValue;
         temp->left    = NULL;
         temp->right   = NULL;
         *tree = temp;
	 if (debug) printf("insert: key = %d\n",keyValue.key);
      }
   }
   else if (keyValue.key < (*tree)->keyValue.key) { 
      insert(keyValue, &((*tree)->left), update);
   }
   else if (keyValue.key > (*tree)->keyValue.key) {
      insert(keyValue, &((*tree)->right), update);
   }
   else if (keyValue.key == (*tree)->keyValue.key) {
      if (update) {
	 //(*tree)->keyValue = keyValue;
	
         /* key-value pair exists and update is true, so overwrite it. */
	
         (*tree)->keyValue.key                 = keyValue.key;
         (*tree)->keyValue.robotLocation.x     = keyValue.robotLocation.x;
         (*tree)->keyValue.robotLocation.y     = keyValue.robotLocation.y;
         (*tree)->keyValue.robotLocation.theta = keyValue.robotLocation.theta;
         strcpy((*tree)->keyValue.robotLocationDescription, keyValue.robotLocationDescription);
         (*tree)->keyValue.gestureTarget.x     = keyValue.gestureTarget.x;
         (*tree)->keyValue.gestureTarget.y     = keyValue.gestureTarget.y;
         (*tree)->keyValue.gestureTarget.z     = keyValue.gestureTarget.z;
         strcpy((*tree)->keyValue.preGestureMessageEnglish, keyValue.preGestureMessageEnglish);
	 strcpy((*tree)->keyValue.preGestureMessageIsiZulu, keyValue.preGestureMessageIsiZulu);
         strcpy((*tree)->keyValue.preGestureMessageKinyarwanda, keyValue.preGestureMessageKinyarwanda);
         strcpy((*tree)->keyValue.postGestureMessageEnglish, keyValue.postGestureMessageEnglish);
	 strcpy((*tree)->keyValue.postGestureMessageIsiZulu, keyValue.postGestureMessageIsiZulu);
         strcpy((*tree)->keyValue.postGestureMessageKinyarwanda, keyValue.postGestureMessageKinyarwanda);
         (*tree)->keyValue.culturalKnowledge.numberOfKeys = keyValue.culturalKnowledge.numberOfKeys;

         for (i=0; i<keyValue.culturalKnowledge.numberOfKeys; i++) {
            (*tree)->keyValue.culturalKnowledge.key[i] = (char *) malloc(sizeof(char) * (strlen(keyValue.culturalKnowledge.key[i])+1));
            strcpy((*tree)->keyValue.culturalKnowledge.key[i], keyValue.culturalKnowledge.key[i]);
         }

	 //printToScreen();
	 
      }
      else {
         printf("EnvironmentKnowledgeBase::insert: key %d already exists and update is false\n",keyValue.key);
      }
   }

   return(tree);
}



/* 
 * int EnvironmentKnowledgeBase::inorder_print_to_file(BinaryTreeType tree, int n, FILE *fp_out) 
 * ---------------------------------------------------------------------------------------------
 *
 * Inorder traversal of a tree, printing node elements to file 
 */

int EnvironmentKnowledgeBase::inorder_print_to_file(BinaryTreeType tree, int n, FILE *fp_out) {

  int k;
  
   if (tree != NULL) {
      inorder_print_to_file(tree->left, n+1, fp_out);

      fprintf(fp_out,
	      "Key                   %-4d \n"
	     "Location Description  %s \n"
             "Robot Location        (%.1f, %.1f  %.1f)\n"
             "Gesture Target        (%.1f, %.1f  %.1f) \n"
             "Pre-Gesture Message English       %s \n"
	     "Pre-Gesture Message isiZulu       %s \n"
             "Pre-Gesture Message Kinyarwanda   %s \n"
             "Post-Gesture Message English      %s \n"
	     "Post-Gesture Message isiZulu      %s \n"
             "Post-Gesture Message Kinyarwanda  %s \n",
	     tree->keyValue.key,
	     tree->keyValue.robotLocationDescription,
	     tree->keyValue.robotLocation.x, tree->keyValue.robotLocation.y, tree->keyValue.robotLocation.theta,
	     tree->keyValue.gestureTarget.x, tree->keyValue.gestureTarget.y, tree->keyValue.gestureTarget.z,
      	     tree->keyValue.preGestureMessageEnglish,
             tree->keyValue.preGestureMessageIsiZulu,
             tree->keyValue.preGestureMessageKinyarwanda,
             tree->keyValue.postGestureMessageEnglish,
             tree->keyValue.postGestureMessageIsiZulu,
             tree->keyValue.postGestureMessageKinyarwanda);

      fprintf(fp_out, "Cultural Knowledge                ");
      for (k=0; k<tree->keyValue.culturalKnowledge.numberOfKeys; k++) {
         fprintf(fp_out," %s", tree->keyValue.culturalKnowledge.key[k]);
      }
      fprintf(fp_out, "\n\n");
      
      inorder_print_to_file(tree->right, n+1, fp_out);
   }
   return(0);
}



/*
 * int EnvironmentKnowledgeBase::inorder_print_to_screen(BinaryTreeType tree, int n)
 * ---------------------------------------------------------------------------------
 *
 * Inorder traversal of a tree, printing node elements to the screen 
 */

int EnvironmentKnowledgeBase::inorder_print_to_screen(BinaryTreeType tree, int n) {

   int k;

   if (tree != NULL) {
      inorder_print_to_screen(tree->left, n+1);
      
      printf("Key                               %-4d \n"
	     "Location Description              %s \n"
             "Robot Location                    (%.1f, %.1f  %.1f)\n"
             "Gesture Target                    (%.1f, %.1f  %.1f) \n"
             "Pre-Gesture Message English       %s \n"
	     "Pre-Gesture Message isiZulu       %s \n"
             "Pre-Gesture Message Kinyarwanda   %s \n"
             "Post-Gesture Message English      %s \n"
	     "Post-Gesture Message isiZulu      %s \n"
             "Post-Gesture Message Kinyarwanda  %s \n",
	      tree->keyValue.key,
	      tree->keyValue.robotLocationDescription,
	      tree->keyValue.robotLocation.x, tree->keyValue.robotLocation.y, tree->keyValue.robotLocation.theta,
	      tree->keyValue.gestureTarget.x, tree->keyValue.gestureTarget.y, tree->keyValue.gestureTarget.z,
	      tree->keyValue.preGestureMessageEnglish,
	      tree->keyValue.preGestureMessageIsiZulu,
	      tree->keyValue.preGestureMessageKinyarwanda,
	      tree->keyValue.postGestureMessageEnglish,
	      tree->keyValue.postGestureMessageIsiZulu,
	      tree->keyValue.postGestureMessageKinyarwanda);
   
      printf("Cultural Knowledge                ");
      for (k=0; k<tree->keyValue.culturalKnowledge.numberOfKeys; k++) {
	 printf("%s ", tree->keyValue.culturalKnowledge.key[k]);
      }
      printf("\n\n");
      
      inorder_print_to_screen(tree->right, n+1);
   }
   return(0);
}


/*
 * int EnvironmentKnowledgeBase::postorder_delete_nodes(BinaryTreeType tree)
 * -------------------------------------------------------------------------
 *
 * Postorder traversal of a tree, deleting node elements 
 */

int EnvironmentKnowledgeBase::postorder_delete_nodes(BinaryTreeType tree) {

   if (tree != NULL) {
      postorder_delete_nodes(tree->left);
      postorder_delete_nodes(tree->right);
      free(tree);
   }
   return(0);
}



/*
 * void EnvironmentKnowledgeBase::printToScreen() 
 * ----------------------------------------------
 *
 *  Print all elements in a tree by traversing inorder - abstract version  
 */

void EnvironmentKnowledgeBase::printToScreen() {

   print_to_screen(tree);
     
   printf("\n");
   
}



/* 
 * int EnvironmentKnowledgeBase::print_to_file(FILE *fp_out) 
 * ---------------------------------------------------------
 *
 * Print all elements in a tree by traversing inorder - abstract version 
 */

int EnvironmentKnowledgeBase::print_to_file(FILE *fp_out) {
 
   print_to_file(tree, fp_out);

   return(0);
}



/*
 * int EnvironmentKnowledgeBase::print_to_file(BinaryTreeType tree, FILE *fp_out)
 * ------------------------------------------------------------------------------
 *
 * Print all elements in a tree by traversing inorder 
 */

int EnvironmentKnowledgeBase::print_to_file(BinaryTreeType tree, FILE *fp_out) {
 
   inorder_print_to_file(tree, 0, fp_out);

   return(0);
}



/*
 * int EnvironmentKnowledgeBase::print_to_screen(BinaryTreeType tree)
 * --------------------------------------------------------------
 *
 * Print all elements in a binary search tree by traversing inorder 
 */

int EnvironmentKnowledgeBase::print_to_screen(BinaryTreeType tree) {
 
   inorder_print_to_screen(tree, 0);

   return(0);
}





/*
 * void EnvironmentKnowledgeBase::readConfigurationData() 
 * ------------------------------------------------------
 *
 * Read configuration parameters key-value pairs from the configuration file 
*/

void EnvironmentKnowledgeBase::readConfigurationData() {

   bool        debug = false;
   int         i; 
   int         j;
   int         k;
   int         number_of_keys;
   std::string packagedir;
   char        filename[MAX_FILENAME_LENGTH];
   char        path_and_configuration_filename[MAX_FILENAME_LENGTH] = "";
   char        path_and_knowedgebase_filename[MAX_FILENAME_LENGTH] = "";
   char        *end_of_file;

   Keyword keylist[NUMBER_OF_CONFIGURATION_KEYS] = {
      "knowledgeBase",
      "verboseMode"
   };

   Keyword key;                  // the key string 
   Keyword value;                // the value string

   char input_string[STRING_LENGTH];
   FILE *fp_config;

   number_of_keys = NUMBER_OF_CONFIGURATION_KEYS;
   
   /* open the configuration file and read the data */
   /* ============================================= */


   /* construct the full path and filename */
   /* ------------------------------------ */
   
   packagedir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
   packagedir = packagedir + "/behaviorController";
   if (debug) cout << "Package directory: " << packagedir << endl;

   strcat(path_and_configuration_filename, packagedir.c_str());  
   strcat(path_and_configuration_filename, "/config/"); 
   strcat(path_and_configuration_filename, configuration_filename);

   
   if (debug) printf("configuration file is  %s\n", path_and_configuration_filename);
   
   if ((fp_config = fopen(path_and_configuration_filename,"r")) == 0) {
      printf("Error: can't open %s\n",path_and_configuration_filename);
      prompt_and_exit(1);
   }


   /* get the key-value pairs */
   /* ----------------------- */
 

   end_of_file = fgets(input_string, STRING_LENGTH, fp_config);

   if (debug) printf ("input string: %s\n",input_string);

   while (end_of_file != NULL) {
     
      if (debug)  printf ("Input string: %s",input_string);

      number_of_keys++;
      
      /* extract the key */

      sscanf(input_string, " %s", key);

      for (j=0; j < number_of_keys; j++) {
         if (strcmp(key,keylist[j]) == 0) {
            switch (j) {
	    case 0:  sscanf(input_string, "%s %s", key, filename);
   
                     /* construct the full path and filename     */

                     strcpy(path_and_knowedgebase_filename,"");
                     strcat(path_and_knowedgebase_filename, packagedir.c_str());  
                     strcat(path_and_knowedgebase_filename, "/data/"); 
                     strcat(path_and_knowedgebase_filename, filename);
		     strcpy(configurationData.knowledgeBase, path_and_knowedgebase_filename);

                     break;

		     
	    case 1:  sscanf(input_string, " %s %s", key, value);       
	             for (k=0; k < (int) strlen(value); k++)
                        value[k] = tolower(value[k]);
                     if (strcmp(value,"true") == 0)
		       configurationData.verboseMode = true;
		     else if (strcmp(value,"false") == 0)
		        configurationData.verboseMode = false;
		     else 
		        printf("readConfigurationData:verboseMode invalid value\n");
                     break;
            }
         }
      }

      end_of_file = fgets(input_string, STRING_LENGTH, fp_config);

   }

   if (configurationData.verboseMode) { 
      printf("readConfigurationData: knowledgeBase %s\n",configurationData.knowledgeBase);
      printf("readConfigurationData: verboseMode   %d\n",configurationData.verboseMode);
   }
}



/*
 * void EnvironmentKnowledgeBase::readKnowledgeBase()
 * --------------------------------------------------
 *
 * Read knowledge base key-value pairs from the data file 
 */

void EnvironmentKnowledgeBase::readKnowledgeBase() {

   bool         debug = false;
   bool         insertFlag = false;
   char         *end_of_file;
   int          number_of_keys;
   char         input_string[MAX_STRING_LENGTH];
   char         alphanumericValue[MAX_STRING_LENGTH];
   int          integerValue;
   int          i;
   int          j;
   int          k;
   int          n;
   int          numberOfCultureKnowledgeKeys;
   int          idNumber;
   char         ch;
   FILE         *fp_in;

   Keyword keylist[NUMBER_OF_VALUE_KEYS] = {
      "robotLocationPose",
      "robotLocationDescription",
      "gestureTarget",
      "preGestureMessageEnglish",
      "preGestureMessageIsiZulu",
      "preGestureMessageKinyarwanda",
      "postGestureMessageEnglish",
      "postGestureMessageIsiZulu",
      "postGestureMessageKinyarwanda",
      "tourSpecification",
      "culturalKnowledge"
   };
      
   Keyword              key;                  // the key string 
   Keyword              value;                // the value string
   Keyword              cultureKey;           // the culture key string 

   number_of_keys = NUMBER_OF_VALUE_KEYS;

   if (configurationData.verboseMode) printf("readKnowledgeBase: initializing knowledge base values\n");
   
  /* open the knowledge base file */

   if ((fp_in = fopen(configurationData.knowledgeBase,"r")) == 0) {  
      printf("Error can't open knowledge base file %s\n",configurationData.knowledgeBase);
      prompt_and_exit(1);
   }

   /* read the key value pairs */
   
   end_of_file = fgets(input_string, STRING_LENGTH, fp_in); // read a line from the file

   while (end_of_file != NULL) {

       insertFlag = false;
     
      /* extract the key */

      if (sscanf(input_string, "%s %d", key, &idNumber) == 2) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST

	 getValue(idNumber, &keyValue);                 // get the key-value pair, if it exists, so that we can overwrite the various fields in the following

         for (j=0; j < number_of_keys; j++) {
            if (strcmp(key,keylist[j]) == 0) {
               switch (j) {
	           
	       /* robotLocationPose idNumber, x, y, theta */
	      
               case 0: if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
		
                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber 
			    i++;
		          }

		          while(!isalnum(input_string[i])) { // skip over the following whitespace
			     i++;
		          }
		       
	                  if (sscanf(input_string+i, "%f %f %f", &(keyValue.robotLocation.x),  &(keyValue.robotLocation.y),  &(keyValue.robotLocation.theta)) == 3) {
			     insertFlag = true;
			  
		             if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s %d %5.1f %5.1f %5.1f\n", key, keyValue.key, keyValue.robotLocation.x, keyValue.robotLocation.y, keyValue.robotLocation.theta);
		          }
		          else {
		             printf("readKnowledgeBase: unsuccessful attempt to assign three floating point value to  key-value x, y, and theta\n");
		          }
	               }
	               else {
		          printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
		          insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
		       }
		    
                       break;

		    
	       /* robotLocationDescription idNumber description */
		    
	       case 1: if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
		      
                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber
			     i++;
		          }

		       
		          while(!isalnum(input_string[i])) { // skip over the following whitespace
			     i++;
		          }

                          strcpy(keyValue.robotLocationDescription, input_string+i);                                      // copy the remaining text as the description
		       
		          keyValue.robotLocationDescription[strlen(keyValue.robotLocationDescription)-1]='\0';            // overwrite the newline \n

		          insertFlag = true;

		          if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s %d %s\n", key, keyValue.key, keyValue.robotLocationDescription);

	               }
	               else {
		         printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
                          insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
                       }
		    
                       break;

		    
	       /* gestureTarget idNumber, x, y, z */
		    
	       case 2:  if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
		
                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber e
			     i++;
		          }

		          while(!isalnum(input_string[i])) { // skip over the following whitespace
			     i++;
		          }
		       
	                  if (sscanf(input_string+i, "%f %f %f", &(keyValue.gestureTarget.x),  &(keyValue.gestureTarget.y),  &(keyValue.gestureTarget.z)) == 3) {
		              insertFlag = true;
			      if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s %d %5.1f %5.1f %5.1f\n", key, keyValue.key, keyValue.gestureTarget.x, keyValue.gestureTarget.y, keyValue.gestureTarget.z);
		          }
		          else {
			     printf("readKnowledgeBase: unsuccessful attempt to assign three floating point value to  key-value x, y, and z\n");
		          }
	               }
	               else {
		          printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
	                  insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
		       }
		    
                       break;

		    
	       /* preGestureMessageEnglish idNumber text */
		    
               case 3: if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
		      
                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber
			     i++;
		          }

		       
		          while(!isalnum(input_string[i])) { // skip over the following whitespace
			    i++;
		          }

                          strcpy(keyValue.preGestureMessageEnglish, input_string+i);                                             // copy the remaining text as the description
		       
		          keyValue.preGestureMessageEnglish[strlen(keyValue.preGestureMessageEnglish)-1]='\0';                   // overwrite the newline \n

                          insertFlag = true;
		       
		          if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s %d %s\n", key, keyValue.key, keyValue.preGestureMessageEnglish);

	               }
	               else {
		         printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
		          insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
		       }
		    
                       break;

		    
	       /* preGestureMessageIsiZulu idNumber text */
		    
               case 4: if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
		      
                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber
			     i++;
		          }

		       
		          while(!isalnum(input_string[i])) { // skip over the following whitespace
		             i++;
		          }

                          strcpy(keyValue.preGestureMessageIsiZulu, input_string+i);                                             // copy the remaining text as the description
		       
		          keyValue.preGestureMessageIsiZulu[strlen(keyValue.preGestureMessageIsiZulu)-1]='\0';                   // overwrite the newline \n

		          insertFlag = true;
		       
		          if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s %d %s\n", key, keyValue.key, keyValue.preGestureMessageIsiZulu);

	               }
	               else {
		         printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
		          insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
		       }
		    
                       break;


	       /* preGestureMessageKinyarwanda idNumber text */
	    
               case 5: if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
		      
                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber
			    i++;
		          }

		       
		          while(!isalnum(input_string[i])) { // skip over the following whitespace
			     i++;
		          }

                          strcpy(keyValue.preGestureMessageKinyarwanda, input_string+i);                                             // copy the remaining text as the description
		       
		          keyValue.preGestureMessageKinyarwanda[strlen(keyValue.preGestureMessageKinyarwanda)-1]='\0';               // overwrite the newline \n

		          insertFlag = true;
		       
	                  if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s %d %s\n", key, keyValue.key, keyValue.preGestureMessageKinyarwanda);

	               }
	               else {
		          printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
		          insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
		       }
		    
                       break;

		    
	       /* postGestureMessageEnglish idNumber text */

	       case 6: if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
		      
                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber
		             i++;
		          }

		       
		          while(!isalnum(input_string[i])) { // skip over the following whitespace
		             i++;
		          }

                          strcpy(keyValue.postGestureMessageEnglish, input_string+i);                                             // copy the remaining text as the description
		       
		          keyValue.postGestureMessageEnglish[strlen(keyValue.postGestureMessageEnglish)-1]='\0';                  // overwrite the newline \n

		          insertFlag = true;
		       
		          if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s %d %s\n", key, keyValue.key, keyValue.postGestureMessageEnglish);

	               }
	               else {
		         printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
	                  insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
                       }
		    
                       break;


		    	    
	       /* postGestureMessageIsiZulu idNumber text */

	       case 7: if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
		      
                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber
		             i++;
		          }

		       
		          while(!isalnum(input_string[i])) { // skip over the following whitespace
			    i++;
		          }

                          strcpy(keyValue.postGestureMessageIsiZulu, input_string+i);                                             // copy the remaining text as the description
		       
		          keyValue.postGestureMessageIsiZulu[strlen(keyValue.postGestureMessageIsiZulu)-1]='\0';                         // overwrite the newline \n

		          insertFlag = true;
		       
		          if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s %d %s\n", key, keyValue.key, keyValue.postGestureMessageIsiZulu);

	               }
	               else {
		         printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
	                  insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
                       }
		    
                       break;

		    	    
	       /* postGestureMessageKinyarwanda idNumber text */

	       case 8: if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
	 	      
                          i=strlen(key);
	                  while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber
		             i++;
		          }

		       
		          while(!isalnum(input_string[i])) { // skip over the following whitespace
		             i++;
		          }

                          strcpy(keyValue.postGestureMessageKinyarwanda, input_string+i);                                             // copy the remaining text as the description
		       
		          keyValue.postGestureMessageKinyarwanda[strlen(keyValue.postGestureMessageKinyarwanda)-1]='\0';              // overwrite the newline \n

		          insertFlag = true;
		       
		          if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s %d %s\n", key, keyValue.key, keyValue.postGestureMessageKinyarwanda);

	               }
	               else {
		         printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
	                  insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
                       }
		    
                       break;
		    

	       /* tourSpecification n idNum1 idNum2 ... idNumn */
		    
               case 9: if (sscanf(input_string, "%s", key)) { // read the alphanumberic key

		          if (configurationData.verboseMode) printf("readKnowledgeBase: %-30s", key);

                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
		       
	                  if (sscanf(input_string+i, "%d", &n) == 1) { // read the number of locations

		             tourSpecification.numberOfLocations = n;
			 
			     if (configurationData.verboseMode) printf(" %d", tourSpecification.numberOfLocations);
				 
		             while(isalnum(input_string[i])) { // skip over the numberOfLocations 
	                        i++;
		             }

		             while(!isalnum(input_string[i])) { // skip over the following whitespace
	                        i++;
                             }

		 	     for (k=0; k<n; k++) {
			        if (sscanf(input_string+i, "%d", &(tourSpecification.locationIdNumber[k])) == 1) {

			           if (configurationData.verboseMode) printf(" %d", tourSpecification.locationIdNumber[k]);

			           while(isalnum(input_string[i])) { // skip over the locationIdNumber 
			              i++;
		                   }
			       
		                   while(!isalnum(input_string[i])) { // skip over the following whitespace
		                      i++;
		                   }
			        }
			        else  {
		                   printf("readKnowledgeBase: unsuccessful attempt to assign value to  key-value locationIdNumber\n");
			        }
		             }
			     
			     if (configurationData.verboseMode) printf("\n");
		          }
	                  else {
		             printf("readKnowledgeBase: unsuccessful attempt to assign value to  key-value numberOfLocations\n");
			  }
	               }
	               else {
		          printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }
	      
                       break;
                   	    		    

	       /* cultureKnowledge idNumber cultureKey1 cultureKey2 ... cultureKeyn */
		    
               case 10: if (sscanf(input_string, "%s %d", key, &(keyValue.key))) { // read the alphanumberic key and the idNumber; the latter is used as the key in the BST
		      
                          i=strlen(key);
		          while(!isalnum(input_string[i])) { // skip over the key and the following whitespace
		             i++;
		          }
	      
		          while(isalnum(input_string[i])) { // skip over the idNumber
		             i++;
		          }

		       
		          while(!isalnum(input_string[i])) { // skip over the following whitespace
		             i++;
		          }

		          numberOfCultureKnowledgeKeys = 0;

	                  while (i<strlen(input_string)) {

			     sscanf(input_string+i, "%s", cultureKey);

                             keyValue.culturalKnowledge.key[numberOfCultureKnowledgeKeys] = (char *) malloc(sizeof(char) * (strlen(cultureKey)+1));
                             strcpy(keyValue.culturalKnowledge.key[numberOfCultureKnowledgeKeys], cultureKey);

			     while (isalnum(input_string[i])) { // skip over the key
		                i++;
		             }

			     while(!isalnum(input_string[i])) { // skip over the following whitespace
			        i++;
			     }

			     numberOfCultureKnowledgeKeys++;
		          }

		          keyValue.culturalKnowledge.numberOfKeys = numberOfCultureKnowledgeKeys;

		          insertFlag = true;

		          if (configurationData.verboseMode) {
		             printf("readKnowledgeBase: %-30s %d ", key, keyValue.key);

			     for (k=0; k< keyValue.culturalKnowledge.numberOfKeys; k++) {
			        printf("%s ", keyValue.culturalKnowledge.key[k]);
			     }

		             printf("\n");
                          }
	               }
	               else {
		         printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to  key-value idNumber\n");
		       }

	               /* add to the knowledgebase */

	               if (insertFlag) {
	                  insert(keyValue, &tree, true); // true implies update allowed, so overwrite an existing key-value pair with new data
                       }
		    
                       break;
                   
		    
               default: printf("readKnowledgeBase: invalid key %s\n",key);
                       break;
               } // switch
            } // if
         } // for
      } // if

      end_of_file = fgets(input_string, STRING_LENGTH, fp_in); // read a line from the file

   } // while

   fclose(fp_in);

}

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
}