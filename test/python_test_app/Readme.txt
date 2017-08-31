graph_rssi.py is a python script that graphs the data received from the LapRSSI timer.

It can be used to visualize the RSSI data, to aid in tuning the LapRSSI configuration parameters.

Follow these steps to get it running:
===================================================================================
1. Install Python 3.5 from here: https://www.python.org/downloads/release/python-350/
2. Install PyQtGraph from here: http://www.pyqtgraph.org/
3. From a command prompt with Administrator privileges:
    pip install PyQt5
    pip install numpy
    pip install PySerial
4. Install the latest Win32 extensions for your platform (for example pywin32-221.win-amd64-py3.5.exe
   for 64-bit Windows) from here: https://sourceforge.net/projects/pywin32/files/pywin32/
5. From a command prompt with Administrator privileges:
    pip install pyttsx3
   Note: this must be done *after* installing the Win32 extensions as indicated above.
6. Open graph_rssi.py in the Idle Python Gui
7. Edit the comPort variable near the top to match your COM port
8. Hit F5 to run the script
