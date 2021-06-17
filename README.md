# RoboCup project

General idea of our project is to program a Humanoid robot such that it will be able to perform tasks such as:
- Detecting the box, its size and color
- Picking up the box
- Walking with the box
- Droping it off at a pre-defined position(sorting it to the correct container) and going to the next box

## Extensions 
TBA

## Task 1: Detect the box and classify it

The task requires for all the positions of the box to be known in advance such that the robot is capable of finding them in the environment.
Once robot detects the box with the camera it classifies it according to the size and color.
Why is that important?
- Depending upon the size of box robot will be able to determine the correct joint angles in order to pick it
- Depending upon the color the robot knows in which container to sort the box

Proof-of-concept design of classification algorithm will be done by Raspberry Pi and Raspberry Pi Camera using OpenCV. Additionally the task will be verified in the simulator.

In order to as much as possible precisely determine both parameters, intrinsic calibration of the camera will be performed.

Possible algorithm used for color classification is:
- Define a range of colour and a corresponding mask that will be used to find colours in each image frame
- Camera Stream read as image frames
- Convert each image frame into a BGR/RGB colour space
- Dilation (process of removing noise from the images)
- Extract/Detect colours in the image frame with an operation with the bitmask
- Create contours for each detected colour and calculate its corresponding region
- Output detection of the colour in Real-time

Possible algorithm used for size determination is:
- we need to know what is the conversion from a pixel to a wished metric(meters)
- We perform edge detection
- find contours that corresponds to the objects in our edge map
- For each contour compute the bounding box
- Determine the size of the bounding box

If the size measurement won't be precise enough we will determine the size of the object according to it's color.
