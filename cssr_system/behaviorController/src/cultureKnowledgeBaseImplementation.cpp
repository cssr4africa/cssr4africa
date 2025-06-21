/* cultureKnowledgeBaseImplementation.cpp   Source code for the implementation of the culture knowledge base helper class: CultureKnowledgeBase
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
 * Author:  David Vernon, Carnegie Mellon University Africa
 * Email:   dvernon@andrew.cmu.edu
 * Date:    28 February 2025
 * Version: v1.0
 *
 */

#include <behaviorController/cultureKnowledgeBaseInterface.h>
namespace Culture {

/*
 * CultureKnowledgeBase::CultureKnowledgeBase
 * ------------------------------------------
 *
 * Class constructor
 *   Read the configuration data from the configuration file
 *   Read the knowledge base value types and build the binary search tree dictionary data structure
 *   Read the knowledge base value and update the binary search tree dictionary data structure 
 */

CultureKnowledgeBase::CultureKnowledgeBase() {

   bool debug = false;
   
   if (debug) printf("CultureKnowledgeBase()\n");

   initialize(&tree); // need to delete nodes if tree is not empty

   readConfigurationData();

   readKnowledgeBaseValueTypes();

   readKnowledgeBase();
   
}



/*
 * CultureKnowledgeBase::~CultureKnowledgeBase
 * -------------------------------------------
 *
 * Class destructor
 *    Delete all nodes in the binary search tree dictionary data structure
 */

CultureKnowledgeBase::~CultureKnowledgeBase() {

   if (configurationData.verboseMode) printf("~CultureKnoweldgeBase()\n");

   postorder_delete_nodes(tree);

}



/*
 * CultureKnowledgeBase::assign_key_attributes(KeyValueType *keyValue, char key[], int valueType, bool initialized)
 * ----------------------------------------------------------------------------------------------------------------
 *
 * Assign a key, value, value type, and initialized flag to a key-value pair 
 * and set the initialized flag to false because the value has not yet been assigned 
 */

void  CultureKnowledgeBase::assign_key_attributes(KeyValueType *keyValue, char key[], int valueType, bool initialized) {

   keyValue->key = (char *) malloc(sizeof(char) * (strlen(key)+1));
   strcpy(keyValue->key, key);
   keyValue->valueType = valueType;
   keyValue->initialized = false;
}


/*
 * void  CultureKnowledgeBase::assign_key_value(KeyValueType *keyValue, int integerValue, bool initialized)
 * --------------------------------------------------------------------------------------------------------
 *
 * Assign an integer value to a key-value pair, and set the initialized value to true 
 */

void  CultureKnowledgeBase::assign_key_value(KeyValueType *keyValue, int integerValue, bool initialized) {
   keyValue->integerValue = integerValue;
   keyValue->initialized = true;
}


/*
 * void CultureKnowledgeBase::assign_key_value(KeyValueType *keyValue, char *alphanumericValue, bool initialized)
 * --------------------------------------------------------------------------------------------------------------
 *
 * Assign a string value to a key-value pair, and set the initialized value to true
 */

void CultureKnowledgeBase::assign_key_value(KeyValueType *keyValue, char *alphanumericValue, bool initialized) {
   keyValue->alphanumericValue = (char *) malloc(sizeof(char) * (strlen(alphanumericValue)+1));
   strcpy(keyValue->alphanumericValue, alphanumericValue);
   keyValue->initialized = true;
} 



/*
 * BinaryTreeType *CultureKnowledgeBase::delete_element(KeyValueType keyValue, BinaryTreeType *tree)
 * -------------------------------------------------------------------------------------------------
 *
 * Delete a key-value pair in a BST 
 */

BinaryTreeType *CultureKnowledgeBase::delete_element(KeyValueType keyValue, BinaryTreeType *tree) {

   BinaryTreeType p;

   if (*tree != NULL) {

     if (keyValue.integerValue < (*tree)->keyValue.integerValue)  // assume keyValue.integerValue is the key
         delete_element(keyValue, &((*tree)->left));        

      else if (keyValue.integerValue > (*tree)->keyValue.integerValue)
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
 * KeyValueType CultureKnowledgeBase::delete_min(BinaryTreeType *tree) 
 * -------------------------------------------------------------------
 *
 * Return & delete the smallest node in a tree (i.e. the left-most node)
 */

KeyValueType CultureKnowledgeBase::delete_min(BinaryTreeType *tree) {

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
 * bool CultureKnowledgeBase::exists(char *s)
 * ------------------------------------------
 *
 * Return true if the key is in the binary search tree; false otherwise - abstract version 
 */

bool CultureKnowledgeBase::exists(char *s) {

   assign_key_attributes(&keyValue, s, 0, false); // valid key but dummy valueType and initialized arguments

   return(exists(keyValue, &tree));
}


/*
 * bool CultureKnowledgeBase::exists(KeyValueType keyValue,  BinaryTreeType *tree)
 * ------------------------------------------------------------------------------
 *
 * Return true if the key is in the tree; false otherwise 
 */

bool CultureKnowledgeBase::exists(KeyValueType keyValue,  BinaryTreeType *tree ) {

   bool found;

   if (*tree == NULL) {

      /* we are at an external node: the key isn't in the tree */

      found = false;

   }
   else if (strcmp(keyValue.key, (*tree)->keyValue.key) < 0) { /* assume the number field is the key */
      found = exists(keyValue, &((*tree)->left));
   }
   else if (strcmp(keyValue.key, (*tree)->keyValue.key) > 0) {
      found = exists(keyValue, &((*tree)->right));
   }
   else if (strcmp(keyValue.key, (*tree)->keyValue.key) == 0) {
     
      /* found it */
      found = true;
   }

   return(found);
}




/*
 * bool CultureKnowledgeBase::getValue(char *key, KeyValueType *keyValue)
 * ---------------------------------------------------------------------- 
 *
 * Return true if the key is in the tree; false otherwise - abstract version
 */

bool CultureKnowledgeBase::getValue(char *key, KeyValueType *keyValue) {

   return(getValue(key, keyValue, &tree));
}




/*
 * bool CultureKnowledgeBase::getValue(char *key, KeyValueType *keyValue,  BinaryTreeType *tree)
 * ---------------------------------------------------------------------------------------------
 *
 * Return true if the key is in the tree; false otherwise 
 */

bool CultureKnowledgeBase::getValue(char *key, KeyValueType *keyValue,  BinaryTreeType *tree ) {

   bool found;

   if (*tree == NULL) {

      /* we are at an external node: the key isn't in the tree */

      found = false;

   }
   else if (strcmp(key, (*tree)->keyValue.key) < 0) { /* assume the number field is the key */
     found = getValue(key, keyValue, &((*tree)->left));
   }
   else if (strcmp(key, (*tree)->keyValue.key) > 0) {
     found = getValue(key, keyValue, &((*tree)->right));
   }
   else if (strcmp(key, (*tree)->keyValue.key) == 0) {
     
      /* found it */
     
      found = true;
      *keyValue= (*tree)->keyValue;
   }
   return(found);
}




/*
 * int CultureKnowledgeBase::height()
 * ----------------------------------
 *
 * Return the height of a BST  -  abstract version 
 */

int CultureKnowledgeBase::height() {

  return(height(tree, 0));          
}



/*
 * int CultureKnowledgeBase::height(BinaryTreeType tree, int n)
 * ------------------------------------------------------------
 *
 * Return the height of a BST 
*/

int CultureKnowledgeBase::height(BinaryTreeType tree, int n) {

   int hl, hr, h;

   h = n;  // height is current level ... better to write h_max = n

   // now find a deeper level

   if (tree != NULL) {
      
      hl = height(tree->left, n+1);
      hr = height(tree->right, n+1);

      // find the max heigth of left and right trees

      if (hl>hr) {
         h = hl;
      }
      else {
         h = hr;
      }

      // if these heights are less than the current height (which is the max found so far) then don't use them

      if (h<n) h = n;  // probably better to write h_max = max(h,n);
   }
   return(h);          // return h_max
}



/*
 * void CultureKnowledgeBase::initialize(BinaryTreeType *tree)
 * -----------------------------------------------------------
 * 
 * Initialize a binary search tree (BST) 
 */

void CultureKnowledgeBase::initialize(BinaryTreeType *tree) {

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
 * BinaryTreeType *CultureKnowledgeBase::insert(KeyValueType keyValue,  BinaryTreeType *tree, bool update)
 * -------------------------------------------------------------------------------------------------------
 *
 * Insert a key-value pair in a BST. if update is true & the key exists, overwrite it
 */

BinaryTreeType *CultureKnowledgeBase::insert(KeyValueType keyValue,  BinaryTreeType *tree, bool update) {

    bool debug = false;
   WindowType temp;

   if (*tree == NULL) {

      /* we are at an external node: create a new node and insert it */

      if ((temp = (NodeType) malloc(sizeof(Node))) == NULL) 
	error((char *)"function insert: unable to allocate memory");
      else {
         temp->keyValue = keyValue;
         temp->left    = NULL;
         temp->right   = NULL;
         *tree = temp;
	 if (debug) printf("insert: %s\n",keyValue.key);
      }
   }
   else if (strcmp(keyValue.key, (*tree)->keyValue.key) < 0) { /* assume the number field is the key */
     insert(keyValue, &((*tree)->left), update);
   }
   else if (strcmp(keyValue.key, (*tree)->keyValue.key) > 0) {
     insert(keyValue, &((*tree)->right), update);
   }
   else if (strcmp(keyValue.key, (*tree)->keyValue.key) == 0) {
     if (update) {
        (*tree)->keyValue = keyValue; // key-value pair exist and update is true, so overwrite it.
     }
     else {
       printf("CultureKnowledgeBase::insert: key %s already exists and update is false\n",keyValue.key);
     }
   }

   return(tree);
}



/* 
 * int CultureKnowledgeBase::inorder_print_to_file(BinaryTreeType tree, int n, FILE *fp_out) 
 * -----------------------------------------------------------------------------------------
 *
 * Inorder traversal of a tree, printing node elements to file 
 */

int CultureKnowledgeBase::inorder_print_to_file(BinaryTreeType tree, int n, FILE *fp_out) {

   if (tree != NULL) {
      inorder_print_to_file(tree->left, n+1, fp_out);

      if (tree->keyValue.valueType == 1) {
	fprintf(fp_out,"%-40s %-50d %-20s %-20s Level: %2d\n", tree->keyValue.key, tree->keyValue.integerValue, valueType2Alphanumeric(tree->keyValue.valueType), initialized2Alphanumeric(tree->keyValue.initialized), n);
      }
      else {
	fprintf(fp_out,"%-40s %-50s %-20s %-20s Level: %2d\n", tree->keyValue.key, tree->keyValue.alphanumericValue, valueType2Alphanumeric(tree->keyValue.valueType), initialized2Alphanumeric(tree->keyValue.initialized), n);
      }
	    
      inorder_print_to_file(tree->right, n+1, fp_out);
   }
   return(0);
}



/*
 * int CultureKnowledgeBase::inorder_print_to_screen(BinaryTreeType tree, int n)
 * -----------------------------------------------------------------------------
 *
 * Inorder traversal of a tree, printing node elements to the screen 
 */

int CultureKnowledgeBase::inorder_print_to_screen(BinaryTreeType tree, int n) {

   int i;

   if (tree != NULL) {
      inorder_print_to_screen(tree->left, n+1);
      
      if (tree->keyValue.valueType == 1) {
	printf("%-40s %-50d %-20s %-20s\n", tree->keyValue.key, tree->keyValue.integerValue, valueType2Alphanumeric(tree->keyValue.valueType), initialized2Alphanumeric(tree->keyValue.initialized));
      }
      else {
	printf("%-40s %-50s %-20s %-20s\n", tree->keyValue.key, tree->keyValue.alphanumericValue, valueType2Alphanumeric(tree->keyValue.valueType), initialized2Alphanumeric(tree->keyValue.initialized));
      }
      
      inorder_print_to_screen(tree->right, n+1);
   }
   return(0);
}




/*
 * int CultureKnowledgeBase::postorder_delete_nodes(BinaryTreeType tree)
 * ---------------------------------------------------------------------
 *
 * Postorder traversal of a tree, deleting node elements 
 */

int CultureKnowledgeBase::postorder_delete_nodes(BinaryTreeType tree) {

   if (tree != NULL) {
      postorder_delete_nodes(tree->left);
      postorder_delete_nodes(tree->right);
      free(tree);
   }
   return(0);
}



/*
 * void CultureKnowledgeBase::printToScreen() 
 * ------------------------------------------
 *
 *  Print all elements in a tree by traversing inorder - abstract version  
 */

void CultureKnowledgeBase::printToScreen() {

   print_to_screen(tree);
  
   printf("\n");
   
}



/* 
 * int CultureKnowledgeBase::print_to_file(FILE *fp_out) 
 * -----------------------------------------------------
 *
 * Print all elements in a tree by traversing inorder - abstract version 
 */

int CultureKnowledgeBase::print_to_file(FILE *fp_out) {
 
   print_to_file(tree, fp_out);

   return(0);
}



/*
 * Print all elements in a tree by traversing inorder 
 */

int CultureKnowledgeBase::print_to_file(BinaryTreeType tree, FILE *fp_out) {
 
   inorder_print_to_file(tree, 0, fp_out);

   return(0);
}



/*
 * int CultureKnowledgeBase::print_to_screen(BinaryTreeType tree)
 * --------------------------------------------------------------
 *
 * Print all elements in a binary search tree by traversing inorder 
 */

int CultureKnowledgeBase::print_to_screen(BinaryTreeType tree) {
 
   inorder_print_to_screen(tree, 0);

   return(0);
}





/*
 * void CultureKnowledgeBase::readConfigurationData() 
 * --------------------------------------------------
 *
 * Read configuration parameters key-value pairs from the configuration file 
*/

void CultureKnowledgeBase::readConfigurationData() {

   bool debug = false;
   int i; 
   int j;
   int k;
   int                  number_of_keys;
   std::string          packagedir;
   char                 filename[MAX_FILENAME_LENGTH];
   char                 path[MAX_FILENAME_LENGTH];
   char                 path_and_configuration_filename[MAX_FILENAME_LENGTH] = "";
   char                 path_and_knowedgebase_filename[MAX_FILENAME_LENGTH] = "";
   char                 *end_of_file;

   Keyword keylist[NUMBER_OF_CONFIGURATION_KEYS] = {
      "knowledgeBase",
      "valueTypes",
      "verboseMode"
   };

   Keyword key;                  // the key string 
   Keyword value;                // the value string

   char input_string[STRING_LENGTH];
   FILE *fp_config;

   number_of_keys = NUMBER_OF_CONFIGURATION_KEYS;

   if (debug) printf("readConfigurationData: initializing configuration data\n");

   
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

   //for (i=0; i<NUMBER_OF_KEYS; i++) {
 

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

	    case 1:  sscanf(input_string, "%s %s", key, filename);
   
                     /* construct the full path and filename     */

                     strcpy(path_and_knowedgebase_filename,"");
                     strcat(path_and_knowedgebase_filename, packagedir.c_str());  
                     strcat(path_and_knowedgebase_filename, "/data/"); 
                     strcat(path_and_knowedgebase_filename, filename);
		     strcpy(configurationData.valueTypes, path_and_knowedgebase_filename);

                     break;
		     
	    case 2:  sscanf(input_string, " %s %s", key, value);       
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
      printf("readConfigurationData: valueTypes    %s\n",configurationData.valueTypes);
      printf("readConfigurationData: verboseMode   %d\n",configurationData.verboseMode);
   }
}



/*
 * void CultureKnowledgeBase::readKnowledgeBase()
 * ----------------------------------------------
 *
 * Read knowledge base key-value pairs from the data file 
 */

void CultureKnowledgeBase::readKnowledgeBase() {

   bool                 debug = false;  
   char                 *end_of_file;
   char                 input_string[MAX_STRING_LENGTH];
   char                 alphanumericValue[MAX_STRING_LENGTH];
   int                  integerValue;
   unsigned int         i, j;
   char                 ch;
   FILE                 *fp_in;
   Keyword              key;                  // the key string 
   Keyword              value;                // the value string


   if (configurationData.verboseMode) printf("readKnowledgeBase: initializing knowledge base values\n");
   
  /* open the knowledge base file */

   if ((fp_in = fopen(configurationData.knowledgeBase,"r")) == 0) {  
      printf("Error can't open knowledge base file %s\n",configurationData.knowledgeBase);
      prompt_and_exit(1);
   }

   /* read the key value pairs */
   
   end_of_file = fgets(input_string, STRING_LENGTH, fp_in); // read a line from the file

   while (end_of_file != NULL) {

      if (configurationData.verboseMode) printf("readKnowledgeBase: %s\n",input_string);

      /* extract the key */

      sscanf(input_string, " %s %s", key, value);

      if (getValue(key, &keyValue) == true) {
      
         switch (keyValue.valueType) {
            // UNDEFINED
            case 0: printf("readKnowledgeBase: attempting to assign a value to a key-value pair with an UNDEFINED value type\n");
                    break;

	   // NUMBER
	    case 1: if (sscanf(value, "%d", &integerValue) == 1) { // string successfully converted to a (single) integer
		       assign_key_value(&keyValue, integerValue, true);
		    }
		    else {
		      printf("readKnowledgeBase: unsuccessful attempt to assign an integer value to a key-value pair with an NUMERIC value type\n");
		    }
                    break;

	    // WORD
	    case 2: if (sscanf(value, "%s", alphanumericValue) == 1) { // string successfully converted to a (single) word
		       assign_key_value(&keyValue, alphanumericValue, true);
		    }
		    else {
		       printf("readKnowledgeBase: unsuccessful attempt to assign a string value to a key-value pair with an WORD value type\n");
		    }
                    break;

	    // PHRASE
            case 3: for (i=strlen(key); !isalnum(input_string[i]); i++) {}; // skip over key and the following whitespace 
	            strcpy(alphanumericValue, input_string+i);              // copy the phrase
		    alphanumericValue[strlen(alphanumericValue)-1]='\0';                // overwrite the newline \n
		    assign_key_value(&keyValue, alphanumericValue, true);
                    break;

            default: printf("readKnowledgeBase: invalid value type %d\n",keyValue.valueType);
                    break;
         }
      }
      else {
	printf("readKnowledgeBase: unsuccessful attempt to get the value of this key %s\n",key);
      }
		
      /* add to the knowledgebase */

      insert(keyValue, &tree, true); // true implies update allows, so overwrite an existing key-value pair with new data

      end_of_file = fgets(input_string, STRING_LENGTH, fp_in); // read a line from the file

   }

   fclose(fp_in);

}



/*
 * void CultureKnowledgeBase::readKnowledgeBaseValueTypes()
 * --------------------------------------------------------
 *
 * Read the types of the values in the knowledge base key-value pairs from the data file 
 */

void CultureKnowledgeBase::readKnowledgeBaseValueTypes() {

   bool                 debug = false;  
   char                 *end_of_file;
   char                 input_string[MAX_STRING_LENGTH];
   unsigned int         i, j;
   char                 ch;
   FILE                 *fp_in;
   Keyword              key;                  // the key string 
   Keyword              valueType;            // the valueType string

   Keyword keylist[NUMBER_OF_VALUE_TYPES] = {
      "UNDEFINED",
      "NUMBER",
      "WORD",
      "PHRASE"
   };


   if (configurationData.verboseMode) printf("readKnowledgeBaseValueTypes: initializing knowledge base value types\n");

   /* open the knowledge base value types file */

   if ((fp_in = fopen(configurationData.valueTypes,"r")) == 0) {  
      printf("Error can't open knowledge base file %s\n",configurationData.knowledgeBase);
      prompt_and_exit(1);
   }

   /* read the key value type pairs */
   
   end_of_file = fgets(input_string, STRING_LENGTH, fp_in); // read a line from the file

   while (end_of_file != NULL) {

     if (configurationData.verboseMode) printf("readKnowledgeBaseValueTypes: %s\n",input_string);
     
      /* extract the key */

      sscanf(input_string, " %s %s", key, valueType);

      for (j=0; j < NUMBER_OF_VALUE_TYPES; j++) {
         if (strcmp(valueType,keylist[j]) == 0) {
            switch (j) {
	    case 0:  assign_key_attributes(&keyValue, key, UNDEFINED, false); // false => not initialized
                     break;
		     
	    case 1:  assign_key_attributes(&keyValue, key, NUMBER, false);      
                     break;

	    case 2:  assign_key_attributes(&keyValue, key, WORD, false);      
                     break;

	    case 3:  assign_key_attributes(&keyValue, key, PHRASE, false);      
                     break;

	    default: printf("readKnowledgeBaseValueTypes: invalid value type\n");
	             break;
            }
         }
      }

      insert(keyValue, &tree, false); // false implies no update so duplicates are not allowed
  
      end_of_file = fgets(input_string, STRING_LENGTH, fp_in); // read a line from the file
   }

   fclose(fp_in);

}



/*
 * Return the size of a binary search tree, i.e. the total number of nodes - abstract version
 */
 
int CultureKnowledgeBase::size() {

  return(size(tree));
}


/*
 * int CultureKnowledgeBase::size(BinaryTreeType tree) 
 * ---------------------------------------------------
 *
 * Return the size of a binary search tree, i.e. the total number of nodes
 */

int CultureKnowledgeBase::size(BinaryTreeType tree) {

   int count;
   int count_left;
   int count_right;

   if (tree != NULL) {
      count_left  = size(tree->left);
      count_right = size(tree->right);
      count = 1 + count_left + count_right;
   }
   else {
      count = 0;
   }

   return(count);
}


/*
 * int CultureKnowledgeBase::total_number_of_probes()
 * --------------------------------------------------
 *
 * Postorder traversal of a tree, computing the total number of probes to find a given word in the tree 
 * this is the sum of the level at which each word appears, starting at level 1 for the root          
 *
 * abstract version                                                                                     
 */

int CultureKnowledgeBase::total_number_of_probes() {

  return(total_number_of_probes(tree, 1));

}
    

/*
 * int CultureKnowledgeBase::total_number_of_probes(BinaryTreeType tree, int n)
 * ----------------------------------------------------------------------------
 * 
 * Postorder traversal of a tree, computing the total number of probes to find a given word in the tree 
 * this is the sum of the level at which each word appears, starting at level 1 for the root          
 *
 */

int CultureKnowledgeBase::total_number_of_probes(BinaryTreeType tree, int n) {

   int count;
   int count_left;
   int count_right;

   if (tree != NULL) {
      count_left  = total_number_of_probes(tree->left, n+1);
      count_right = total_number_of_probes(tree->right, n+1);
      count = n + count_left + count_right;
   }
   else {
      count = 0;
   }

   return(count);
}


/*
 * Utility functions 
 * ================
 */



/*
 * char *initialized2Alphanumeric(bool initialized)
 * ------------------------------------------------
 * 
 * Convert Boolean initialized to alphnumeric
 */

char *initialized2Alphanumeric(bool initialized) {

  static Keyword alphanumeric;
  
  if (initialized) strcpy(alphanumeric,"Initilaized Value");
  else             strcpy(alphanumeric,"Uninitialized Value");

  return(alphanumeric);
}



/* 
 * char *valueType2Alphanumeric(int valueType) 
 * -------------------------------------------
 *
 * Convert integer valueType to alphnumeric 
 */

char *valueType2Alphanumeric(int valueType) {

   static Keyword keylist[NUMBER_OF_VALUE_TYPES] = {
      "UNDEFINED",
      "NUMBER",
      "WORD",
      "PHRASE"
   };


   if (valueType >=0 && valueType <=3) {
       return(keylist[valueType]); 
   }
   else {
      printf("valueType2Alphanumeric: invalid value type\n");
      return(NULL);
   }
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