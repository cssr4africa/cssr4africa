import json
import sys

import chromadb
import openai
import rospy

from cssr_system.srv import get_intent, get_intentResponse, prompt, promptResponse


# Static config options (not set via config file)
NODE_NAME = "conversation_management"
GET_INTENT_SERVICE = "/conversationManagement/get_intent"
PROMPT_SERVICE = "/conversationManagement/prompt"
FALLBACK_RESPONSE = "I don't understand what you mean. Try rephrasing your question!"
MIN_SIM_SCORE=0.5

# Config options set via config file
COLLECTION_NAME = "collection_name"
COLLECTION_DESCRIPTION = "collection description"
DATA_FILE_PATH = "/path/to/data/file"
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


def _load_json_data(file_path):
    subsections = []

    with open(file_path, "r", encoding="utf-8") as file:
        json_data = json.load(file)

    for i, item in enumerate(json_data):
        item["doc_id"] = str(i + 1) if "doc_id" not in item else str(item["doc_id"])

        if "section" not in item:
            item["section"] = ""

        if "subsections" not in item:
            item["subsections"] = []

        if "content" not in item:
            item["content"] = ""

        for idx, sub_section in enumerate(item.get("subsections", [])):
            sub_section["doc_id"] = str(sub_section.get("doc_id", idx))
            sub_section["doc_id"] = str(item["doc_id"]+"_"+sub_section["doc_id"])
            sub_section["section"] = sub_section.get("section", "")
            sub_section["content"] = sub_section.get("content", "")

            for sidx, sub_sub_section in enumerate(sub_section.get("subsections", [])):
                sub_sub_section["doc_id"] = str(sub_sub_section.get("doc_id", sidx))
                sub_sub_section["doc_id"] = str(sub_section["doc_id"]+"_"+sub_sub_section["doc_id"])
                sub_sub_section["section"] = sub_sub_section.get("section", "")
                sub_sub_section["content"] = sub_sub_section.get("content", "")
                subsections.append(sub_sub_section)

            del sub_section["subsections"]
            subsections.append(sub_section)

        del item["subsections"]

    return json_data + subsections


def _populate_collection(collection, data_items):
    """ Fill a collection with data

    Parameters:
        collection: a Chroma DB collection object
        data_items: data to fill in the passed collection
    """
    documents, metadatas, ids = [], [], []
    used_ids = set()

    for i, data in enumerate(data_items):
        if data.get("content", "") == "":
            continue

        text = f"{data['section']}: {data.get('content', '')}. "
        base_id = str(data.get("doc_id", i))
        unique_id = base_id
        counter = 1

        while unique_id in used_ids:
            unique_id = f"{base_id}_{counter}"
            counter += 1

        used_ids.add(unique_id)
        documents.append(text)
        ids.append(unique_id)
        metadatas.append({"section": data["section"]})

    collection.add(documents=documents, metadatas=metadatas, ids=ids)


def _retrieve_similar_documents(collection, query, n_results=5):
    results = collection.query(query_texts=[query], n_results=n_results)
    formatted_results = []

    if len(results) == 0 or len(results["ids"]) == 0 or len(results["ids"][0]) == 0:
        return []

    for i in range(len(results["ids"][0])):
        similarity_score = 1 - results["distances"][0][i]

        if similarity_score >= MIN_SIM_SCORE:
            formatted_results.append({
                "doc_id": results["ids"][0][i],
                "section": results["metadatas"][0][i]["section"],
                "content": results["documents"][0][i],
                "similarity_score": similarity_score,
                "distance": results["distances"][0][i]
            })

    return formatted_results


def _prompt_srv_handler(req):
    """ Function that gets called every time the /conversationManagement/prompt ROS
    service is invoked. It generates a response to a passed query.

    Parameters:
        req:    request object containing a query
    """
    global _conversation_history

    response = FALLBACK_RESPONSE
    context = "\n".join([
        f"{result['section']} - {result['content']}"
        for result in _retrieve_similar_documents(_collection, req.query, TOP_K)
    ])
    system_message = [{"role": "system", "content": SYSTEM_PROMPT}]
    _conversation_history += [
        {
            "role": "user",
            "content": f"{req.query} \nRelevant context that you may use: {context}"
        }
    ]
    generated_response = _openai_client.chat.completions.create(
        model=LLM, messages=system_message + _conversation_history
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
    Run a speechEvent ROS node
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
    global COLLECTION_NAME, COLLECTION_DESCRIPTION, DATA_FILE_PATH, OPENAI_BASE_URL
    global OPENAI_API_KEY, LLM, VERBOSE_MODE, MAX_COVERSATION_HISTORY_LEN, TOP_K
    global HEARTBEAT_MSG_PERIOD, SYSTEM_PROMPT
    global _collection, _conversation_history, _chroma_client, _openai_client

    rospy.init_node(NODE_NAME, anonymous=True)

    if config["verboseMode"].strip().lower() not in ["true", "false"]:
        rospy.logerr(
            "speechEvent: the verboseMode '%s' is not supported, supported "
            "verboseModes are true and false" % config["verboseMode"].strip()
        )
        sys.exit(1)

    COLLECTION_NAME = config["collection_name"].strip()
    COLLECTION_DESCRIPTION = config["collection_description"].strip()
    DATA_FILE_PATH = config["dataFilePath"].strip()
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
            "embedding_function": chromadb.utils.embedding_functions.SentenceTransformerEmbeddingFunction(model_name="all-MiniLM-L6-v2")
        }
    )
    _populate_collection(_collection, _load_json_data(DATA_FILE_PATH))
