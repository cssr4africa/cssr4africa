import os
import sys

import chromadb
import openai
import pymupdf
import rospy

from cssr_system.srv import get_intent, get_intentResponse, prompt, promptResponse


# Static config options (not set via config file)
NODE_NAME = "conversation_management"
GET_INTENT_SERVICE = "/conversationManagement/get_intent"
PROMPT_SERVICE = "/conversationManagement/prompt"
FALLBACK_RESPONSE = "I don't understand what you mean. Try rephrasing your question!"
MAX_DIST_SCORE = 0.5
DB_CHUNK_SIZE = 400

# Config options set via config file
COLLECTION_NAME = "collection_name"
COLLECTION_DESCRIPTION = "collection description"
DOCS_DIR = "/path/to/docs/dir/"
OPENAI_BASE_URL = "http://localhost:8080/v1"
OPENAI_API_KEY = "no-secret-key-needed"
LLM = "HuggingFaceTB/SmolLM3-3B"
VERBOSE_MODE = True
MAX_COVERSATION_HISTORY_LEN = 20
TOP_K = 3
HEARTBEAT_MSG_PERIOD = 10  # seconds

# Config options that are resolved dynamically
SYSTEM_PROMPT = ""

# Global variables
_collection = None
_conversation_history = []
_chroma_client = None
_openai_client = None


def _save_text_in_db(text, description, text_idx, text_idx_prefix):
    chunks = [text[i:i+DB_CHUNK_SIZE] for i in range(0, len(text), DB_CHUNK_SIZE)]
    for i, chunk in enumerate(chunks):
        _collection.add(
            documents=[chunk],
            metadatas=[{"description": description, "id": f"{text_idx_prefix}{text_idx}_{i}"}],
            ids=[f"{text_idx_prefix}{text_idx}_{i}"]
        )


def _save_txt_text_in_db(txt_path, txt_description, txt_idx):
    with open(txt_path, "r") as f:
        text = f.read()
    _save_text_in_db(text, txt_description, txt_idx, "txt")


def _save_pdf_text_in_db(pdf_path, pdf_description, pdf_idx):
    text = " ".join([i.get_text() for i in pymupdf.open(pdf_path)])
    _save_text_in_db(text, pdf_description, pdf_idx, "pdf")


def _retrieve_documents_from_db(query):
    documents = _collection.query(query_texts=[query], n_results=TOP_K)
    return [
        documents["documents"][0][idx]
        for idx, distance in enumerate(documents["distances"][0])
        if distance < MAX_DIST_SCORE
    ]


def _prompt_srv_handler(req):
    """ Function that gets called every time the /conversationManagement/prompt ROS
    service is invoked. It generates a response to a passed query.

    Parameters:
        req:    request object containing a query
    """
    global _conversation_history

    response = FALLBACK_RESPONSE
    context = "\n".join(_retrieve_documents_from_db(req.query))
    system_message = [{"role": "system", "content": SYSTEM_PROMPT}]
    examples = [
        {"role": "user", "content": "Who are you?"},
        {"role": "assistant","content": "Pepper: I am Pepper, a lab assistant here in the AI and Robotics Lab at CMU-Africa. How can I help you today?"},

        {"role": "user","content": "What can you do?"},
        {"role": "assistant","content": "Pepper: I can answer questions you have about the AI and Robotics Lab at CMU-Africa."},

        {"role": "user","content": "Hello Pepper"},
        {"role": "assistant","content": "Pepper: Hello too. How can I help you today?"},
    ]
    _conversation_history += [
        {
            "role": "user",
            "content": f"{req.query} \nRelevant context that you may use: {context}"
        }
    ]
    generated_response = _openai_client.chat.completions.create(
        model=LLM, messages=system_message + examples + _conversation_history
    )

    if len(generated_response.choices) > 0:
        response = generated_response.choices[0].message.content.strip()

    _conversation_history.append({"role": "assistant", "content": response})

    if len(_conversation_history) > MAX_COVERSATION_HISTORY_LEN:
        _conversation_history = _conversation_history[-MAX_COVERSATION_HISTORY_LEN:]

    return promptResponse(response)


def _get_intent_srv_handler(req):
    """ Function that gets called every time the /conversationManagement/get_intent ROS
    service is invoked. It extracts a yes/no intent from a message.

    Parameters:
        req:    request object containing the message whose intent is to be determined
    """
    response = "negative"
    messages = [
        {
            "role": "system",
            "content":
                """
                You need to determine the intent (positive or negative) of messages. Respond only with either 'positive'
                or 'negative', and provide no further text. No further explanations are to be provided. Always check the
                text you generate at least one more time to ensure that it contains only one word, either the word
                'positive' or the word 'negative', and rectifying the generated text to generate only either 'positive'
                or 'negative' without any other text or explanation being provided. Remember, never generate anything
                other than 'positive' or 'negative', and never give extra information or explanations.
                """
        },
        {"role": "user", "content": "Yes"}, {"role": "assistant","content": "positive"},
        {"role": "user","content": "No"}, {"role": "assistant","content": "negative"},
        {"role": "user","content": "Give me a tour of the lab"}, {"role": "assistant","content": "positive"},
        {"role": "user","content": "I don't want a tour"}, {"role": "assistant","content": "negative"},
        {"role": "user","content": f"{req.message}"}
    ]
    generated_response = _openai_client.chat.completions.create(model=LLM, messages=messages)

    if len(generated_response.choices) > 0:
        response = generated_response.choices[0].message.content.strip()

    return get_intentResponse(response)


def run():
    """
    Run a conversationManagement ROS node
    """
    rospy.Service(GET_INTENT_SERVICE, get_intent, _get_intent_srv_handler)
    rospy.Service(PROMPT_SERVICE, prompt, _prompt_srv_handler)
    rospy.Timer(rospy.Duration(HEARTBEAT_MSG_PERIOD),lambda _: rospy.loginfo("conversationManagement: running"))

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

    rospy.spin()


def initialise(config, system_prompt):
    """ Make preparatory initialisations before running a conversationManagement
    ROS node

    Patameters:
        config: object containing configuration arguments
    """
    global COLLECTION_NAME, COLLECTION_DESCRIPTION, DOCS_DIR, OPENAI_BASE_URL
    global OPENAI_API_KEY, LLM, VERBOSE_MODE, MAX_COVERSATION_HISTORY_LEN, TOP_K
    global HEARTBEAT_MSG_PERIOD, SYSTEM_PROMPT
    global _collection, _conversation_history, _chroma_client, _openai_client

    rospy.init_node(NODE_NAME, anonymous=True)

    if config["verboseMode"].strip().lower() not in ["true", "false"]:
        rospy.logerr(
            "conversationManagement: the verboseMode '%s' is not supported, supported "
            "verboseModes are true and false" % config["verboseMode"].strip()
        )
        sys.exit(1)

    COLLECTION_NAME = config["collection_name"].strip()
    COLLECTION_DESCRIPTION = config["collection_description"].strip()
    DOCS_DIR = config["docsDir"].strip()
    OPENAI_BASE_URL = config["openaiBaseUrl"].strip()
    OPENAI_API_KEY = config["openaiApiKey"].strip()
    LLM = config["llm"].strip()
    VERBOSE_MODE = True if config["verboseMode"].strip().lower() == "true" else False
    MAX_COVERSATION_HISTORY_LEN = int(config["maxConversationHistoryLen"].strip())
    TOP_K = int(config["topK"].strip())
    HEARTBEAT_MSG_PERIOD = int(config["heartbeatMsgPeriod"].strip())
    SYSTEM_PROMPT = system_prompt

    _conversation_history = []
    _chroma_client = chromadb.Client()
    _openai_client = openai.OpenAI(base_url=OPENAI_BASE_URL, api_key=OPENAI_API_KEY)

    try:
        _chroma_client.get_collection(name=COLLECTION_NAME)
        _chroma_client.delete_collection(name=COLLECTION_NAME)
    except chromadb.errors.NotFoundError:
        pass

    _collection = _chroma_client.create_collection(
        name=COLLECTION_NAME,
        metadata={"description": COLLECTION_DESCRIPTION},
        configuration={
            "hnsw": {"space": "cosine"},
            "embedding_function": chromadb.utils.embedding_functions.SentenceTransformerEmbeddingFunction(
                model_name="all-MiniLM-L6-v2"
            )
        }
    )
    all_docs = os.listdir(DOCS_DIR)
    pdf_paths = [os.path.join(DOCS_DIR, i) for i in all_docs if os.path.splitext(i)[1] == ".pdf"]
    txt_paths = [os.path.join(DOCS_DIR, i) for i in all_docs if os.path.splitext(i)[1] == ".txt"]
    pdf_descriptions = [os.path.splitext(os.path.basename(i))[0] for i in pdf_paths]
    txt_descriptions = [os.path.splitext(os.path.basename(i))[0] for i in txt_paths]
    [
        _save_txt_text_in_db(txt_path, txt_description, idx)
        for idx, (txt_path, txt_description) in enumerate(zip(txt_paths, txt_descriptions))
    ]
    [
        _save_pdf_text_in_db(pdf_path, pdf_description, idx)
        for idx, (pdf_path, pdf_description) in enumerate(zip(pdf_paths, pdf_descriptions))
    ]
