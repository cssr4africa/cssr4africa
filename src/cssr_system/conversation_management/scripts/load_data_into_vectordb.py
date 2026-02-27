from chromadb.utils import embedding_functions
import chromadb
import pymupdf
import sys
import os

DATA_DIR = "docs_dir"
COLLECTION_DESCRIPTION = "collection description"
COLLECTION_NAME = "collection_name"
DB_CHUNK_SIZE = 400


_collection = None
_chroma_client = None

def _initialize_chroma_client():
    global _chroma_client
    if _chroma_client is None:
        _chroma_client = chromadb.HttpClient(host='localhost', port=8000)
        
    sentence_transformer_ef = embedding_functions.SentenceTransformerEmbeddingFunction(
        model_name="all-MiniLM-L6-v2"
    )
    
    global _collection
    _collection = _chroma_client.get_or_create_collection(
        name=COLLECTION_NAME, 
        metadata={'description': COLLECTION_DESCRIPTION},
        configuration={
            "hnsw": {"space": "cosine"},
            "embedding_function": sentence_transformer_ef
        }
    )

def _save_text_in_db(text, description, idx):
    chunks = [text[i:i+DB_CHUNK_SIZE] for i in range(0, len(text), DB_CHUNK_SIZE)]
    for i, chunk in enumerate(chunks):
        _collection.add(
            documents=[chunk],
            metadatas=[{"description": description, "id": f"{idx}_{i}"}],
            ids=[f"{idx}_{i}"]
        )
        
        print(f"Saved chunk {i} for description {description}")
        
def _load_txt_file_into_db(file_path, description):
    with open(file_path, 'r') as f:
        text = f.read()
    _save_text_in_db(text, description, "txt")
    
def _load_pdf_file_into_db(file_path, description):
    doc = pymupdf.open(file_path)
    text = " ".join([page.get_text() for page in doc])
    _save_text_in_db(text, description, "pdf")
    
def main():
    # recieve collection name and collection description as args with default values
    
    if len(sys.argv) < 3:
        print("Usage: python load_data_into_vectordb.py <collection_name> <docs_dir>")
        sys.exit(1)
        
    global COLLECTION_NAME, DATA_DIR, COLLECTION_DESCRIPTION
    COLLECTION_NAME = sys.argv[1]
    DATA_DIR = sys.argv[2]
    
    if len(sys.argv) > 3:
        COLLECTION_DESCRIPTION = sys.argv[3]
    
    _initialize_chroma_client()
    
    # Load data into the database
    for file in os.listdir(DATA_DIR):
        if file.endswith(".txt"):
            _load_txt_file_into_db(os.path.join(DATA_DIR, file), file.split(".")[0])
        elif file.endswith(".pdf"):
            _load_pdf_file_into_db(os.path.join(DATA_DIR, file), file.split(".")[0])
            

if __name__ == "__main__":
    main()