"""
speech_event_gui.py - graphical interface for displaying transcribed text

Author:     Clifford Onyonka
Date:       2025-07-11
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import subprocess
import threading
import tkinter as tk


class _FixedSizeList(list):
    def __init__(self, capacity, *args):
        super().__init__(*args)
        self._capacity = capacity

    def append(self, object):
        while len(self) >= self._capacity:
            self.pop(0)
        super().append(object)


class GUI:
    def _get_main_window(title):
        """ Return the main top-level Tkinter widget

        Parameters:
            title (str): the title of the top-level window

        Returns:
            tk.Tk:  top-level Tkinter window
        """
        window = tk.Tk()
        window.title(title)
        window.minsize(720, 480)

        return window

    def _get_main_frame(window):
        """ Return the main frame widget that all other widgets are to reside in

        Parameters:
            window (Tk): the top-level window that wraps the main frame

        Returns:
            tk.Frame:   main frame widget
        """
        frame = tk.Frame(window)
        frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        frame.pack(side="top", fill="both", expand=True)
        
        return frame

    def _listen_on_topic(window, text, process, max_num_of_words):
        """ A forever running loop that listens on the /speechEvent/text ROS topic
        and updates the GUI with any text that gets published on the topic

        Parameters:
            window (tk.Tk):             the main/root Tkinter window
            text (tk.Label):            the Tkinter widget to be updated with text
                transcriptions published on /speechEvent/text
            process (subprocess.Popen): process that reads text published on
                /speechEvent/text
            max_num_of_words (int):     max number of words to display on GUI

        Returns:
            None
        """
        text_list = _FixedSizeList(max_num_of_words)

        while True:
            if process.poll():
                break
            stdout = process.stdout.readline().decode("UTF-8")
            if "data" in stdout:
                [text_list.append(i) for i in stdout[7:-2].split(" ")]
                window.after(0, lambda: text.configure(text=" ".join(text_list)))

    def run(pub_topic, max_num_of_words=10):
        """
        Run GUI application to display text transcriptions being published on the
        /speechEvent/text ROS topic
        """
        window = GUI._get_main_window(f"SpeechEvent output (rostopic echo {pub_topic})")
        frame = GUI._get_main_frame(window)
        process = subprocess.Popen(
            ["rostopic", "echo", pub_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        text = tk.Label(frame, font=("Helvetica", 16))
        text.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        text.pack(side="top", fill="both", expand=True)

        topic_p = threading.Thread(
            target=GUI._listen_on_topic,
            args=(window, text, process, max_num_of_words)
        )
        topic_p.start()
        window.mainloop()
        topic_p.join()

        window.destroy()
        process.kill()
