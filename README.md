# LLM
1. llama-server --hf-repo ggml-org/Meta-Llama-3.1-8B-Instruct-Q4_0-GGUF --hf-file meta-llama-3.1-8b-instruct-q4_0.gguf -c 2048 --n-gpu-layers 1000

# Chromadb
1. docker run -v ./chroma-data:/data -p 8000:8000 chromadb/chroma

# speechEvent
1 [Optional] -> catkin_make
2 [Required] -> source /opt/venv_speech_event/bin/activate
3 [Required] -> rosrun cssr_system speech_event_application.py

# conversationManagement
1 [Optional] -> catkin_make
2 [Required] -> source /opt/venv_conversation_management/bin/activate
3 [Required] -> rosrun cssr_system conversation_management_application.py

# textToSpeech
1 [Optional] -> catkin_make
2 [Required] -> source /opt/venv_text_to_speech/bin/activate
3 [Required] -> rosrun cssr_system streaming_tts_to_nao_service.py

# soundDetection
1 [Required] -> sudo apt install python3.8-venv -y
2 [Required] -> python3.8 -m venv cssr4africa_sound_detection_env
3 [Required] -> source cssr4africa_sound_detection_env/bin/activate
4 [Required] -> pip install --upgrade pip
5 [Required] -> pip install -r src/cssr_system/sound_detection/sound_detection_requirements.txt


# if gpu device not found in container env
sudo systemctl stop nvidia-persistenced
sudo modprobe -r nvidia_uvm
sudo modprobe nvidia_uvm
sudo systemctl start nvidia-persistenced
