# IK with Processing and Tkinter
This is a simple example of how to use the Inverse Kinematics (IK) algorithm with Processing and Tkinter. Through an interactive drawing mat using Tkinter, the user writes a free drawing which is passed on to a Processing sketch. The sketch uses a 3-DOF robot with IK to draw the same drawing with the robot's end-effector.

## Requirements
Python requirements:
```bash
pip install numpy
sudo apt install python3-tk
```

Processing [PDE](https://processing.org/download/)

A bash script that runs the python and processing code is also provided. To execute it, the processing-java command must be in the PATH. This can be done by running the following commands:
```bash
export PATH=$PATH:/opt/processing-path # would be something like /opt/processing-4.3/
# if you want to make it permanent, add the line to your .bashrc file
echo 'export PATH=$PATH:/opt/processing' >> ~/.bashrc
# Create an alias to call from the terminal
sudo ln -s /opt/processing-path/processing-java /bin/processing-java
```

Make the Python and bash script executable:
```bash
chmod +x draw_and_show.sh tkinter_drawer.py
```

## Usage
To run the code, simply execute the bash script:
```bash
./draw_and_show.sh
```

## Further Information and Demonstration
Code explanation is available in the included [PDF](./IK_with_Processing_and_Tkinter.pdf) file. A demonstration of the code can be found [here for cartesian drawing](https://youtu.be/sfuk7INZ7JQ) and [here for polar drawing](https://youtu.be/eVJJieALkS8).


