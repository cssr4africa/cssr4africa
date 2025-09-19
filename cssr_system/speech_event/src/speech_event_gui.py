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

import multiprocessing
import signal
import subprocess
import threading
import time
import tkinter as tk


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
        window.geometry("720x480")

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

    def _listen_on_topic(pub_topic, q):
        """ A forever running loop that listens on the /speechEvent/text ROS topic

        Parameters:
            pub_topic (str): topic to listen on
            q (queue.Queue): a queue to write incoming transcriptions into

        Returns:
            None
        """
        process = subprocess.Popen(
            ["rostopic", "echo", pub_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        signal.signal(signal.SIGINT, lambda _, __: process.kill())
        signal.signal(signal.SIGTERM, lambda _, __: process.kill())

        while not process.poll():
            stdout = process.stdout.readline().decode("UTF-8")
            incoming = stdout[7:-2] if "data" in stdout else f" {stdout[4:-2]}" if stdout[:-2].strip() != "--" else "\n\n"
            q.put(incoming) if len(incoming) > 0 else "pass"

    def _write_text(text, q, stop_event):
        """ A forever running loop that updates the GUI with text

        Parameters:
            text (tk.Text): the Tkinter widget to be updated with text
                transcriptions published on /speechEvent/text
            q (queue.Queue): a queue to write incoming transcriptions into

        Returns:
            None
        """
        while not stop_event.is_set():
            if q.empty():
                time.sleep(0.5)
                continue
            incoming = q.get()
            text.config(state="normal")
            text.insert(tk.END, incoming)
            text.config(state="disabled")

    def run(pub_topic):
        """
        Run GUI application to display text transcriptions being published on the
        /speechEvent/text ROS topic
        """
        q = multiprocessing.Queue()
        stop_event = multiprocessing.Event()
        window = GUI._get_main_window(f"SpeechEvent output (rostopic echo {pub_topic})")
        frame = GUI._get_main_frame(window)

        text = tk.Text(frame, font=("Helvetica", 16))
        text.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        text.config(state="disabled")
        text.pack(side="top", fill="both", expand=True)

        topic_p = multiprocessing.Process(target=GUI._listen_on_topic, args=(pub_topic, q))
        write_p = threading.Thread(target=GUI._write_text, args=(text, q, stop_event))

        def kill_processes(_, __):
            topic_p.kill()
            q.close()
            q.join_thread()
            stop_event.clear()

        signal.signal(signal.SIGINT, kill_processes)
        signal.signal(signal.SIGTERM, kill_processes)

        topic_p.start()
        write_p.start()

        window.mainloop()

        topic_p.kill()
        stop_event.set()
        write_p.join()

        q.close()
        q.join_thread()
        stop_event.clear()

        window.destroy()
