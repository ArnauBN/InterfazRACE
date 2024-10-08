# InterfazGraficaRobot
This program *tries* to follow an MVC (Model-View-Controller) architecture. 
- The model contains and manages the data and devices.
- The view contains all graphical and GUI elements.
- The controller links the model to the view.

For example, imagine we want to turn an LED on/off with a button. The view would define how the window and the button look, the model would contain the current state of the LED and the communication (maybe serial) with the LED, and the controller would change the LED's state when the button is pressed.

This graphical interface is created using PyQt5 and the *Model ↔ Controller ↔ View* communication is usually performed using PyQt's signals and slots.

There have been several *quick fixes* that had to be made on-the-fly when testing the complete system with all collaborators present. This, and with the initially poor definition of the project that this interface had to consolidate, make ---in my opinion--- a somewhat convoluted code. I would recommend forgoing the MVC architecture.

The full program can be found here: https://github.com/ArnauBN/InterfazRACE and contains the following files and folders.

## FILE: mainROS.py
Main entry point of the program. It initiates the interface **with** ROS.

## FILE: mainInterface.py
Execute this file to initiate the interface **without** ROS. Can be used for debugging.

## FILE: endostitch_serial.py
ROS node for the endostitch. Uses serial communication.

## FOLDER: config
Any configuration or initial state of the program should be in this folder: fixed suture points, steps of the experiment to be performed, etc.
### FOLDER: experiments
Each file in this folder should be a text file and contain the definition of all the processes of a specific experiment. 
- The first line of the file can optionally start with '#' which will indicate the name of the experiment. If this is not provided, the name of the file will be used instead.
- Each line should define a single process with the following format: ```process <symbol> <name>```. For example: ```process P1 Initial phase```.
- The program will read the file and draw each process as an ellipse with its name inside it.
- Ideally, the program could read relationships between processes, conditions, etc. and draw a complete diagram of the experiment. This is not yet implemented.
### FILEs: Puntos.txt and PuntosCam.txt
These files where created and used manually on-the-fly to test and show the capability of drawing suture points on the live camera feed. The program reads the coordinates of the points from ```PuntosCam.txt```, which were originally taken from ```Puntos.txt```. ```Puntos.txt``` is currently not used.


## FOLDER: controllers
### FILE: endostitch_controller.py
Connects the buttons of the endostitch control window (endostitch view) with the endostitch model.
### FILE: experiment_controller.py
Connects the buttons of the experiment view with the experiment model, starts the camera feed, adds the experiment processes to the drawing zone, changes the processes color if they are active or not.
### FILE: main_controller.py
Adds functionality to all buttons in the main window.

## FOLDER: docs
Could contain any documentation file needed. This ```README``` file is found in the root folder for convenience, so the ```docs``` folder is empty for now.

## FOLDER: models
This folder contains:
- ```device_model.py```: a generic serial device model that can be imported by others. Currently unused.
- ```camara_model.py```, ```endostitch_model.py```, ```phantom_model.py```, ```razonador_model.py```, ```urautonomo_model.py```, ```urteleoperado_model.py```: one model for each device.
- ```experiment_model.py```: one model for an experiment (defining its DFD or Data Flow Diagram).
- ```experiments_model.py```: one model for the list of all experiments loaded.
- ```main_model.py```: one main model for the set of all devices used.

## FOLDER: resources
Contains any multimedia files like images, videos, audio, etc. Currently, it contains an ```icons``` folder with four status icons as pngs, and a ```logos``` folder with the logos of the three universities involved and the logo of the project itself.

## FOLDER: styles
Empty folder for now. Could contain the definition of a standard visual style for all buttons and assets of the program.

## FOLDER: tests
Contains any independent or stand-alone tests that do not require the interface and should be directly executed. ```RealSenseCam.py```, ```realsense-cv_test.py``` and ```realsense-pyqt-threading_test.py``` all test the realsense depth camera in different ways.

## FOLDER: translations
Empty folder for now. Could contain possible translations Spanish ↔ English if needed.

## FOLDER: ui_files
The files contained in this folder are XML-based and created using Qt Designer. They define the appearance of the different windows of the program, which are then imported by Python. Current files: ```cam_dialog.ui```, ```endostitch_view.ui```, ```experiment_view.ui```, ```main_view.ui```, ```realsenseCam_dialog.ui```. To import a UI file, use ```PyQt5.uic.loadUi(UIPath, self)```.

## FOLDER: utils
Contains utility files and functions.
### FILE: camera_threads.py
Defines the camera feed threads and camera configuration.
### FILE: DFDparsers.py
Defines how the experiments text files should be read.
### FILE: generic.py
Functions related to generic computations like finding the first number in a string or drawing circles on top of an image given its coordinates.
### FILE: globals.py
Global variables: ```PATH_TO_PROJECT```, ```PATH_TO_INTERFAZ_CLIENT_CAMERA``` and ```ACCEPTED_DFD_ITEM_TYPES```.

## FOLDER: venv
Contains the files that define the environment, dependencies, packages, etc. Currently, there are two ```.yml``` files:
- ```environment-no_deps.yml```: defines the environment without SO-specific dependencies. **Rospy not included**.
- ```environment-windows-with_deps.yml```: defines the environment with SO-specific (Windows) dependencies. **Rospy not included**.

Using conda, these files can be used to create the environment with 

```
conda env create -f environment-no_deps.yml
```

## FOLDER: views
Contains one view file for every window. Every view loads its corresponding ```.ui``` file.

## FOLDER: widgets
Contains the definition of all graphical elements of the experiment's Flow Chart and any other custom widgets that could be needed.

# Improvements to be made
- Forgo MVC architecture and make the codebase less convoluted.
- Implement a complete and proper DFD (Data Flow Diagram) to define and draw the steps, conditions, relationships... of the experiments. Could be a state machine parser. See https://github.com/pbauermeister/dfd
- Add functionality to the experiment buttons Stop, Start, etc.
- Homogenize language (Spanish or English) in variable names, class names, function names, etc.