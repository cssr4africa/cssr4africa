#!/usr/bin/env python

__import__('pysqlite3')
import sys
sys.modules['sqlite3'] = sys.modules.pop('pysqlite3')

import sys
from pathlib import Path

parent_dir = Path(__file__).parent
parent_dir = parent_dir
print(f"Parent directory being added to sys.path: {parent_dir}")
sys.path.append(str(parent_dir))  # Ensure parent directory is in sys.path

import rospy
import actionlib
# import cssr_system.msg
from cssr_system.srv import Prompt, PromptResponse, CreateCollection, CreateCollectionResponse
from conversation_management_implementation import *

# Static config options (not set via config file)
NODE_NAME = "conversation_management"
GET_INTENT_SERVICE = "/conversationManagement/get_intent"
PROMPT_SERVICE = "/conversationManagement/prompt"
CREATE_COLLECTION = "/conversationManagement/create_collection"
HEARTBEAT_MSG_PERIOD = 10  # seconds

config = None
conversation_history = None
collection = None

def handle_prompt_request(req):
    """
    Callback function to handle prompt service requests.
    """
        
    global conversation_history, collection, config
    
    verbose_mode = config.get('verboseMode', 'false').lower() == 'true'
    
    if verbose_mode:
        rospy.loginfo("Received prompt request: %s", req.prompt)

    ai_response = handle_rag_query(collection, req.prompt, conversation_history, config.get('llm', 'meta-llama/Llama-3.1-8B-Instruct'),
                                    verbose_mode, int(config.get('topK', 3)), 
                                    float(config.get('similarity_threshold', 0.05)))

    conversation_history.append({"role": "user", "content": req.prompt})
    conversation_history.append({"role": "assistant", "content": ai_response})

    response = PromptResponse()
    
    # Keep conversation history manageable
    max_conversation_history_length = int(config.get('maxConversationHistoryLen', 3))
    if len(conversation_history) > max_conversation_history_length:
        conversation_history = conversation_history[-max_conversation_history_length:]

    if verbose_mode:
        rospy.loginfo("AI response: %s", ai_response)

    response.response = ai_response
    return response

def handle_prompt_intent_request(req):
    """
    Callback function to handle prompt intent service requests.
    """
        
    global conversation_history, collection, config
    
    verbose_mode = config.get('verboseMode', 'false').lower() == 'true'
    
    if verbose_mode:
        rospy.loginfo("Received prompt request: %s", req.prompt)

    ai_response = handle_rag_query(collection, req.prompt, conversation_history, verbose_mode, int(config.get('topK', 3)), intent=True)

    # conversation_history.append({"role": "user", "content": req.prompt, "response": ai_response})

    response = PromptResponse()
    
    # Keep conversation history manageable
    max_conversation_history_length = int(config.get('maxConversationHistoryLen', 3))
    if len(conversation_history) > max_conversation_history_length:
        conversation_history = conversation_history[-max_conversation_history_length:]

    if verbose_mode:
        rospy.loginfo("AI response: %s", ai_response)

    response.response = ai_response
    return response

def handle_create_collection_request(req):
    """
    Callback function to handle create collection service requests.
    """
        
    global collection, config
    
    verbose_mode = config.get('verboseMode', 'false').lower() == 'true'
    
    if verbose_mode:
        rospy.loginfo("Received create collection request: %s, %s, %s", req.name, req.datafile_path, req.description)
        
    global collection
    collection = create_collection_and_load_data(req.name, req.description, req.datafile_path, verbose_mode)

    response = CreateCollectionResponse()
    
    if collection:
        response.success = 1
        response.message = f"Collection '{req.name}' created successfully."

        if verbose_mode:
            rospy.loginfo(response.message)
    else:
        response.success = 0
        response.message = f"Failed to create collection '{req.name}'. Debug logs for details."
        if verbose_mode:
            rospy.loginfo(response.message)
                
    
    return response

def rag_service_server():
    global config
    config = read_config(Path(__file__).parent.parent / 'config' / 'conversation_management_configuration.ini')

    initialize_clients(config.get('openaiBaseUrl', "http://localhost:8080/v1"), 
                       config.get('openaiApiKey', "sk-no-key-required"))

    """
    Initializes the ROS node and advertises the service.
    """
    rospy.init_node('rag_service_server_node')
    s = rospy.Service(PROMPT_SERVICE, Prompt, handle_prompt_request)
    s2 = rospy.Service(CREATE_COLLECTION, CreateCollection, handle_create_collection_request)
    s3 = rospy.Service(GET_INTENT_SERVICE, Prompt, handle_prompt_intent_request)
    rospy.Timer(rospy.Duration(HEARTBEAT_MSG_PERIOD),lambda _: rospy.loginfo("conversationManagement: running"))

    rospy.loginfo("RAG service server ready.")
    
    # Get collection for RAG system
    global collection
    collection = get_similarity_search_collection(config.get('collection_name', 'interactive_upanzi_search'))

    if collection is None:
        rospy.logerr("Call the create_collection service to create a collection before using the RAG system.")

    global conversation_history
    conversation_history = []
    
    rospy.loginfo(
        f"""conversationManagement v1.0

        \r{' ' * 28}This project is funded by the African Engineering and Technology Network (Afretec)
        \r{' ' * 28}Inclusive Digital Transformation Research Grant Programme.

        \r{' ' * 28}Website: www.cssr4africa.org

        \r{' ' * 28}This program comes with ABSOLUTELY NO WARRANTY.
        """
    )
    rospy.loginfo("conversationManagement: start-up")

    rospy.loginfo(f"conversationManagement: {GET_INTENT_SERVICE} service advertised")
    rospy.loginfo(f"conversationManagement: {PROMPT_SERVICE} service advertised")
    
    # server = RAGActionServer('rag_action_server')

    rospy.spin() # Keep the node alive until shutdown

if __name__ == "__main__":
    rag_service_server()