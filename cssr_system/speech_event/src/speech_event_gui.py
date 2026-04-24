"""
speech_event_gui.py - graphical interface for displaying transcribed text

Author:     Clifford Onyonka, Carnegie Mellon University Africa
Email:      cliffor2@andrew.cmu.edu
Date:       2026-04-21
Version:    v1.1

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
        """ A forever running loop that listens on some ROS topic

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

        def stop_process(_, __):
            process.kill()
            process.wait()

        signal.signal(signal.SIGINT, stop_process)
        signal.signal(signal.SIGTERM, stop_process)

        while not process.poll():
            stdout = process.stdout.readline().decode("UTF-8")
            try:
                incoming = stdout[18:-2] if "transcription" in stdout else \
                    f" {stdout[3:-2]}" if stdout.strip()[0] == "\\" else \
                        "\n\n" if stdout.strip() == "---" else ""
            except IndexError:
                incoming = ""
            q.put(incoming) if len(incoming) > 0 else "pass"

    def _write_text(text, q, stop_event):
        """ A forever running loop that updates the GUI with text

        Parameters:
            text (tk.Text):                 the Tkinter widget to be updated with
                text transcriptions published on some ROS topic
            q (queue.Queue):                a queue to write incoming transcriptions into
            stop_event (threading.Event):   an event for flagging the termination
                of this thread

        Returns:
            None
        """
        def write_text(incoming):
            text.config(state="normal")
            text.insert(tk.END, incoming)
            text.config(state="disabled")

        while not stop_event.is_set():
            if q.empty():
                time.sleep(0.5)
                continue
            incoming = q.get()
            text.after(0, write_text, incoming)

    def run(pub_topic):
        """
        Run GUI application to display text transcriptions being published on some ROS topic
        """
        q = multiprocessing.Queue()
        stop_event = threading.Event()
        window = GUI._get_main_window(f"SpeechEvent output (rostopic echo {pub_topic})")
        frame = GUI._get_main_frame(window)

        text = tk.Text(frame, font=("Helvetica", 16))
        text.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        text.config(state="disabled")
        text.pack(side="top", fill="both", expand=True)

        topic_p = multiprocessing.Process(target=GUI._listen_on_topic, args=(pub_topic, q))
        write_p = threading.Thread(target=GUI._write_text, args=(text, q, stop_event), daemon=True)

        def stop_processes(sig_num=None, frame=None):
            stop_event.set()

            if topic_p.is_alive():
                topic_p.terminate()
                topic_p.join()

            window.quit()

        signal.signal(signal.SIGINT, stop_processes)
        signal.signal(signal.SIGTERM, stop_processes)

        topic_p.start()
        write_p.start()

        window.mainloop()

        q.close()
        q.join_thread()

        window.destroy()
