# Full-Body Action Classifier
## The flow of the system: 

1. To use the kNN, the kNN.py script alone is sufficient. It'll send trigger whever an action is detected, which can be sent to the controller. 
2. For the angle based system:<br/>
    a. LandmarkDetection.py : Runs the mediapipe library and outputs the video feed with landmarks and publishes landmark coordinates. <br/>
    b. tempaction.py : Classifies action and outputs stamped int to be used by the controller.<br/>
    c. GestureController.cpp : Can be modified to control a single drone. <br/>