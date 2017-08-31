graph_rssi.py is a python script that graphs the data received from the LapRSSI timer.

It can be used to visualize the RSSI data, to aid in tuning the LapRSSI configuration parameters.

Follow these steps to get it running:
===================================================================================
1. Install Python 3.5 from here: https://www.python.org/downloads/release/python-350/
2. Install PyQtGraph from here: http://www.pyqtgraph.org/
3. From a command prompt:
    pip install PyQt5
    pip install numpy
    pip install PySerial
4. Open graph_rssi.py in the Idle Python Gui
5. Edit the comPort variable near the top to match your COM port
6. Hit F5 to run the script
