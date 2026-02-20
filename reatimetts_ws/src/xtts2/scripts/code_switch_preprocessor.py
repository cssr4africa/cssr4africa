#!/usr/bin/env python3
import re
from enum import Enum
import rospy

class Language(Enum):
    KINYARWANDA = "rw"
    ENGLISH = "en"

class CodeSwitchPhonemizer:
    def __init__(self, lexicon_path=None):
        self.lexicon = set()
        if lexicon_path:
            self.load_lexicon(lexicon_path)
        else:
            # Basic embedded common English words found in tech/general conversation
            self.lexicon = {
                "computer", "robot", "system", "program", "code", "file",
                "data", "network", "internet", "software", "hardware", "device",
                "user", "interface", "application", "server", "client", "database",
                "function", "variable", "loop", "condition", "array", "string",
                "integer", "float", "boolean", "class", "object", "method",
                "thread", "process", "algorithm", "debug", "compile", "execute",
                "error", "exception", "library", "framework", "module", "package",
                "version", "update", "install", "uninstall", "config", "settings",
                "login", "logout", "password", "email", "message", "chat",
                "call", "video", "audio", "media", "file", "document",
                "download", "upload", "search", "browser", "website", "page",
                "link", "button", "menu", "option", "select", "click",
                "drag", "drop", "scroll", "zoom", "view", "edit","lynnx", "motion",
                "save", "open", "close", "print", "share", "send", "Pepper","roomba",
                "receive", "connect", "disconnect", "sync", "backup", "restore",
                "pick and place"
            }
    
    def load_lexicon(self, path):
        try:
            with open(path, 'r') as f:
                for line in f:
                    word = line.strip().lower()
                    if word:
                        self.lexicon.add(word)
        except Exception as e:
            rospy.logwarn(f"Failed to load lexicon: {e}")

    def normalize_text(self, text):
        return text.strip()

    def is_english(self, text):
        # Heuristics:
        # 1. Check if words are in lexicon
        # 2. Check for English-specific patterns (like 'th', 'sh' ending words, 'ght') 
        #    though Kinyarwanda has 'sh', it doesn't have 'th', 'ght', 'x', 'q' usually.
        
        words = text.split()
        if not words:
            return False
            
        english_count = 0
        for word in words:
            w = word.lower().strip(".,!?\"':;")
            if w in self.lexicon:
                english_count += 1
            # elif "th" in w or "ght" in w or "tion" in w: # Weak heuristic
            #     english_count += 1
        
        return (english_count / len(words)) > 0.5

    def process_text(self, text, known_english_terms=None):
        """
        Splits text into segments of Kinyarwanda and English.
        Optionally uses a list of known english terms to force segmentation.
        Returns: (processed_text_placeholder, segments_list)
        """
        class Segment:
            def __init__(self, text, lang, start=0, end=0):
                self.text = text
                self.language = lang
                self.start_idx = start
                self.end_idx = end
        
        segments = []
        
        # 1. If known terms are provided, use the regex splitting approach
        if known_english_terms:
            import re
            # Escape terms and join with OR
            pattern = f"({'|'.join(map(re.escape, known_english_terms))})"
            # Split keeping delimiters
            parts = re.split(pattern, text, flags=re.IGNORECASE)
            
            current_idx = 0
            for part in parts:
                if not part.strip():
                    continue
                
                # Check if matches any English term
                is_english = any(part.lower() == term.lower() for term in known_english_terms)
                lang = Language.ENGLISH if is_english else Language.KINYARWANDA
                
                # Further processing for Kinyarwanda chunks?
                # For now, treat the whole non-matching chunk as Kinyarwanda potentially 
                # (or recursively check for other English words? Let's keep it simple as per request)
                
                segments.append(Segment(part, lang, current_idx, current_idx + len(part)))
                current_idx += len(part)
                
            return text, segments

        # 2. Existing logic for general processing if no specific terms
        chunks = re.split('([.,;!?]+)', text)

        segments = []
        current_idx = 0
        
        # If the split leaves empty strings (e.g. delimiters at end), filter them
        # Reconstruct chunks with their delimiters attached if possible or process sequentially
        
        # Re-merge delimiters with previous chunk or treat as separate? 
        # Let's iterate and build.
        
        temp_chunks = []
        for i in range(0, len(chunks), 2):
            c = chunks[i]
            delim = chunks[i+1] if i+1 < len(chunks) else ""
            full_chunk = c + delim
            if full_chunk.strip():
                temp_chunks.append(full_chunk)

        for chunk in temp_chunks:
            lang = Language.ENGLISH if self.is_english(chunk) else Language.KINYARWANDA
            
            # Optimization: Merge with previous if same language
            if segments and segments[-1].language == lang:
                segments[-1].text += " " + chunk
                segments[-1].end_idx += len(chunk) + 1
            else:
                segments.append(Segment(chunk, lang, current_idx, current_idx + len(chunk)))
            
        return text, segments
