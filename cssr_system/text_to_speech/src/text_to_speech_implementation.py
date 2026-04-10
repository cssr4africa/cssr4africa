"""
text_to_speech_implementation.py - implementation of the text-to-speech functionality

Author:     Muhirwa Richard
Date:       2025-04-18
Version:    v2.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import os
import sys
import tempfile
import subprocess
import contextlib

import rospy
import torch
import soundfile as sf
from pathlib import Path
from configparser import ConfigParser

from TTS.utils.synthesizer import Synthesizer
from TTS.tts.configs.xtts_config import XttsConfig
from TTS.tts.models.xtts import Xtts

HEARTBEAT_MSG_PERIOD = 10


class TTSImplementation:

    # Class variables for paths (set by the application before instantiation)
    _config_file_path  = None
    _model_dir_path    = None   # Kinyarwanda YourTTS model directory
    _python2_script_path = None

    @classmethod
    def set_paths(cls, config_file_path, model_dir_path, python2_script_path):
        cls._config_file_path    = config_file_path
        cls._model_dir_path      = model_dir_path
        cls._python2_script_path = python2_script_path

    # ------------------------------------------------------------------

    def __init__(self):
        self.config = {
            'language':              'english',
            'interface':             'service',
            'playback_mode':         'naoqi',
            'verboseMode':           True,
            'ip':                    '172.29.111.240',
            'port':                  '9559',
            'useCuda':               False,
            'kinyarwandaModelPath':  self._model_dir_path or '',
            'englishModelPath':      str(Path.home() / 'models' / 'v2.0.2'),
            'englishSpeakerWav':     '',
        }

        self.config_file_path  = self._config_file_path
        self.model_files_dir   = self._model_dir_path   # Kinyarwanda model files (default)
        self.python2_path      = '/usr/bin/python2'
        self.python2_script    = self._python2_script_path
        self.supported_languages = ['english', 'kinyarwanda']

        self.read_config()
        self.initialize_components()

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def read_config(self):
        try:
            if not os.path.exists(self.config_file_path):
                rospy.logwarn(
                    f"Config file not found: {self.config_file_path}. Using defaults.")
                return

            parser = ConfigParser()
            parser.read(self.config_file_path)
            sec = 'DEFAULT'

            def get(key):
                return parser[sec].get(key, '').strip()

            if get('language'):
                self.config['language'] = get('language').lower()
            if get('interface'):
                self.config['interface'] = get('interface').lower()
            if get('playback_mode'):
                self.config['playback_mode'] = get('playback_mode').lower()
            if get('verboseMode'):
                self.config['verboseMode'] = get('verboseMode') == 'True'
            if get('ip'):
                self.config['ip'] = get('ip')
            if get('port'):
                self.config['port'] = get('port')
            if get('useCuda'):
                self.config['useCuda'] = get('useCuda').lower() in ('true', '1', 'yes', 'on')
            if get('englishModelPath'):
                self.config['englishModelPath'] = get('englishModelPath')
            if get('englishSpeakerWav'):
                self.config['englishSpeakerWav'] = get('englishSpeakerWav')
            if get('kinyarwandaModelPath'):
                self.config['kinyarwandaModelPath'] = get('kinyarwandaModelPath')

            rospy.loginfo(f"Configuration loaded from {self.config_file_path}")

        except Exception as exc:
            rospy.logwarn(f"Unable to read configuration file: {exc}. Using defaults.")

    # ------------------------------------------------------------------
    # Component initialisation
    # ------------------------------------------------------------------

    @contextlib.contextmanager
    def _suppress_output(self):
        if self.config.get('verboseMode', True):
            yield
        else:
            old_out, old_err = sys.stdout, sys.stderr
            try:
                with open(os.devnull, 'w') as devnull:
                    sys.stdout = devnull
                    sys.stderr = devnull
                    yield
            finally:
                sys.stdout = old_out
                sys.stderr = old_err

    def initialize_components(self):
        try:
            # Apply kinyarwandaModelPath from config (may override the class-level default)
            if self.config.get('kinyarwandaModelPath'):
                self.model_files_dir = self.config['kinyarwandaModelPath']

            use_cuda = False
            if self.config['useCuda']:
                use_cuda = torch.cuda.is_available()
                if not use_cuda:
                    rospy.logwarn("CUDA requested but not available. Falling back to CPU.")
            self.use_cuda = use_cuda

            self._load_kinyarwanda_model()
            self._load_english_model()

            if self.config['verboseMode']:
                rospy.loginfo(f"Model files directory (Kinyarwanda): {self.model_files_dir}")
                rospy.loginfo(f"English model directory: {self.config['englishModelPath']}")
                rospy.loginfo(f"Using CUDA: {self.use_cuda}")
                rospy.loginfo("TTS components initialized successfully.")

            rospy.Timer(
                rospy.Duration(HEARTBEAT_MSG_PERIOD),
                lambda _: rospy.loginfo("textToSpeech: running"))

        except Exception as exc:
            rospy.logerr(f"Unable to initialize TTS components: {exc}")
            raise

    def _load_kinyarwanda_model(self):
        rospy.loginfo("Loading Kinyarwanda YourTTS model...")
        with self._suppress_output():
            self.kin_synth = Synthesizer(
                os.path.join(self.model_files_dir, "model.pth"),
                os.path.join(self.model_files_dir, "config.json"),
                tts_speakers_file=os.path.join(self.model_files_dir, "speakers.pth"),
                encoder_checkpoint=os.path.join(self.model_files_dir, "SE_checkpoint.pth.tar"),
                encoder_config=os.path.join(self.model_files_dir, "config_se.json"),
                use_cuda=self.use_cuda,
            )
        rospy.loginfo("Kinyarwanda model loaded.")

    def _load_english_model(self):
        rospy.loginfo("Loading English XTTS v2 model...")
        model_dir = Path(self.config['englishModelPath'])
        cfg = XttsConfig()
        cfg.load_json(str(model_dir / "config.json"))
        self.eng_model = Xtts.init_from_config(cfg)
        self.eng_model.load_checkpoint(cfg, checkpoint_dir=str(model_dir), eval=True)
        if self.use_cuda:
            self.eng_model.cuda()
        self.eng_config = cfg
        rospy.loginfo("English XTTS v2 model loaded.")

    # ------------------------------------------------------------------
    # Synthesis
    # ------------------------------------------------------------------

    def _english_speaker_wav(self):
        """Return the speaker reference WAV path for XTTS v2."""
        wav = self.config.get('englishSpeakerWav', '')
        if wav:
            return wav
        # Fall back to Kinyarwanda conditioning audio
        return os.path.join(self.model_files_dir, "conditioning_audio.wav")

    def synthesize(self, text, language):
        """Synthesize speech and return the path to a temporary WAV file.

        The caller is responsible for deleting the file when done.
        """
        if language == 'kinyarwanda':
            with self._suppress_output():
                wav = self.kin_synth.tts(
                    text,
                    speaker_wav=os.path.join(self.model_files_dir, "conditioning_audio.wav"),
                )
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                with self._suppress_output():
                    self.kin_synth.save_wav(wav, fp)
                return fp.name

        elif language == 'english':
            outputs = self.eng_model.synthesize(
                text,
                self.eng_config,
                speaker_wav=self._english_speaker_wav(),
                gpt_cond_len=3,
                language="en",
                speed=1.0,
            )
            wav_data = outputs["wav"]
            sr = self.eng_config.audio.sample_rate
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                sf.write(fp.name, wav_data, sr)
                return fp.name

        else:
            raise ValueError(
                f"Unsupported language: '{language}'. "
                "Valid values: 'kinyarwanda', 'english'."
            )

    # ------------------------------------------------------------------
    # Playback
    # ------------------------------------------------------------------

    def play_audio(self, audio_file):
        """Play a WAV file according to the configured playback_mode."""
        mode = self.config['playback_mode']

        if mode == 'naoqi':
            env = dict(os.environ)
            env['VERBOSE_AUDIO'] = '1' if self.config.get('verboseMode') else '0'
            kwargs = {}
            if not self.config.get('verboseMode'):
                kwargs = {'stdout': subprocess.DEVNULL, 'stderr': subprocess.DEVNULL}
            subprocess.run(
                [
                    self.python2_path,
                    self.python2_script,
                    audio_file,
                    self.config['ip'],
                    self.config['port'],
                ],
                env=env,
                check=True,
                **kwargs,
            )

        else:  # local
            subprocess.run(
                ["pactl", "suspend-sink", "RDPSink", "0"],
                check=False,
            )
            subprocess.run(["paplay", "--volume=65536", audio_file], check=True)

    # ------------------------------------------------------------------
    # Convenience: synthesize + play in one call (for service handler)
    # ------------------------------------------------------------------

    def say_text(self, text, language):
        """Synthesize and play speech.

        Returns:
            tuple: (success: bool, message: str, audio_file_path: str)
        """
        if language not in self.supported_languages:
            msg = f"Unsupported language: '{language}'"
            rospy.logerr(msg)
            return False, msg, ""

        audio_file = ""
        try:
            rospy.loginfo(f"Synthesizing '{text}' [{language}]")
            audio_file = self.synthesize(text, language)

            rospy.loginfo(f"Playing audio [{self.config['playback_mode']}]")
            self.play_audio(audio_file)

            return True, f"Spoken '{text}' in {language}", audio_file

        except Exception as exc:
            rospy.logerr(f"Error in say_text: {exc}")
            return False, str(exc), audio_file

        finally:
            if audio_file and os.path.exists(audio_file):
                try:
                    os.unlink(audio_file)
                except OSError:
                    pass
