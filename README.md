## NAO Robot Intro

The most important thing on the NAO for us was 2x Head camera we used for object recongition, while all other joints were relevant for motions.

![image](https://user-images.githubusercontent.com/48418580/125603897-e288db98-0732-488f-8171-77c382e7b307.png)


## Webots Intro

Initially we started with SPARK, Choregraphe but endep up using Webots.
The reason for that are:
- We had problem setting up the environments on our computers, since everybody needed a different python setup etc.
- Adding and modifying object in Webots is really convenient and additionaly lots of tutorials for that is provided


## Task Description

General idea of our project is to program a NAO Humanoid robot such that it will be able to perform tasks such as:
- Searching for the box in the near area
- Detecting the box and its color
- Move towards the box
- Pick up the box
- Walk with the box towards the drop of location
- Drop it off at a pre-defined position
- Back away from the box
And these steps can go on and on in a loop...

### Initial Planning

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

All movement keyframes will be created via choreographe.
This includes:
- walking with the box
- kneeling down to pick the box up
- turning around 180° to find the box


### Final Plan

We needed to adjust and replan initial task due to time constraint and limited knowledge about Webots, Choregraphe and other tools we used.
Steps that were performed on a final design are summarized below.

### Our Webots Environment

Background of our environment is a random living room, where red T-shaped object represent a box we will carry to the blue drop off box location.

![image](https://user-images.githubusercontent.com/48418580/125654907-271ddfee-12a0-4c72-90e1-149c3a10b1c0.png)


### Box detection Setup

For box detection and its position relative to the camera on the NAO robot, Webots API is used. 
Detecting object position with monocular camera is quite unprecise, so we had to take that into an account. 
In case when object was seen ob both Bottom and Top Camera, Top camera was way more accurate. 
That's why we are innitialy checking whether object/drop off location is seen on the Top Camera and then as a fallback we check if the object/drop off location is seen on the Bottom camera. 
We needed to do that since we don't have any reference to determine when the Top Camera object detection is more accurate than Bottom camera and in our case the object will mostly be searched through the Top Camera.

We took aprox. measurement of the system to mathematically formulate what the robot is actually capable of seeing.
Image below denotes both camera Field of Views and gives a sense to what NAO is constrained to.

<img src="https://user-images.githubusercontent.com/48418580/125604669-9964c653-2583-44c4-a4e0-32ac0fad222f.png" width="450" height="450"> <img src="https://user-images.githubusercontent.com/48418580/125604723-97ef5800-9802-4174-a52b-7ec5fa39491a.png" width="450" height="450">


### Searching for Box

NAO spins on spot and after each in-place rotation it checks whether is sees a red T-shaped object. Once red object is detected and the orientation error is aprox. the same as the smallest in place turn angle NAO can do, robot transitions to moving towards it. 


### Moving towards the box

First the robot turns towards the object such that it is aproximately perpendicular to the object. Calculated using:
	
	atan2(y, x) 

where x, y represent the position of the object in the NAO camera frames.
Robot approaches box until it reaches a pickup distance, which is a predefined value from which robot is able to pick up the object it approached.

Distance is calculated using Euclidian formula:

	sqrt(x*x + y*y)

where x, y represent the position of the object in the NAO camera frames.

During the movement towards the box we always check whether the object is still seen and start again with the in-place rotations if we lost sight of it.

### Camera transformation to ground X,Y Plane

Since camera coordinate is not alligned with the ground coordinate frame a rotation transformation needed to be applied. According to the camera angles read from the NAO Webots model, we inversed the angles and used it in a rotational matrix. This matrix was then multiplied with the values received from the cameras(object position relative to the camera).
Rotation matrix using numpy:

	a11 = cos(rot_z)*cos(rot_y)
	a12 = cos(rot_z)*sin(rot_y)*sin(rot_x) - sin(rot_z)*cos(rot_x)
	a13 = cos(rot_z)*sin(rot_y)*cos(rot_x) + sin(rot_z)*sin(rot_x)
	a21 = sin(rot_z)*sin(rot_y)
	a22 = sin(rot_z)*sin(rot_y)*sin(rot_x) + cos(rot_z)*cos(rot_x)
	a23 = sin(rot_z)*sin(rot_y)*cos(rot_x) - cos(rot_z)*cos(rot_x)
	a31 = -sin(rot_y)
	a32 = cos(rot_y)*sin(rot_x)
	a33 = cos(rot_y)*cos(rot_x)
	rot_matrix = np.matrix([[a11, a12, a13],
                        [a21, a22, a23],
                        [a31, a32, a33]])



### Pick Up Maneuver

In order to simplify complex picking up maneuvers we should have performed in order to accurately pick up box or cone shaped object we designed a T-shaped Box(T-Box) this way we only need to be aproximately perpendicular to the box and at a distance of about 20-30 centimeters.

![image](https://user-images.githubusercontent.com/48418580/125595552-8e0e4591-7617-4bd9-b7fe-6d7239df2d5f.png) 

<img src="https://user-images.githubusercontent.com/48418580/125655013-12910a44-a7bc-41a1-ad00-2db76fe5da9d.png" width="450" height="350"> <img src="https://user-images.githubusercontent.com/48418580/125655049-380505d8-e4e3-411a-987a-6ca91459048d.png" width="450" height="350"> <img src="https://user-images.githubusercontent.com/48418580/125655078-15469035-f30b-4f13-89d2-08fdcf19b58b.png" width="450" height="350"> <img src="https://user-images.githubusercontent.com/48418580/125654726-8676e061-9cdd-44a6-91b8-fe28c50ec926.png" width="450" height="350">

<img src="https://user-images.githubusercontent.com/48418580/125654849-c0035d51-d617-4d8c-881a-4eaedcecf97f.png" width="750" height="550">

### Searching for Drop Off location

As soon as we Picked up the object we were constrained to only one camera, since Bottom camera of the robot was covered with the T-Box, we had to rely only on Top Camera. 
Similar to as without the T-Box, robot performed in place rotation until it detected the drop off location. 
Drop off location is a blue  box robot is able to recognize through camera stream.


### Moving towards the Drop Off location

As explained in top section Field of View of cameras are limited to a certain range, this is why as the robot was getting closer to the drop off location it dissapeared from the camera stream. 
As soon as that happened we marked the last distance measurement we got and performed a simplified version of Dead Reckoning.
For example if the last distance was aprox. 60 centimeters, we blindly performed 6 steps forward each for 10 centimeters.


### Drop Off and Back Away Maneuver

As soon as we blindly aproached are drop off area, similarly to the pick up maneuvers, object was dropped and task was completed.
If the robot tried to move after dropping of the object with colliding into it, it had to first back away/go backwards. 
That is why after the drop off robot immediately backs away for about 30-40cm. 

<img src="https://user-images.githubusercontent.com/48418580/125656383-f7e1fa3d-1985-4e52-9b02-8c7a0b3aff14.png" width="450" height="350"> <img src="https://user-images.githubusercontent.com/48418580/125656594-cc9e4ddc-6ad2-40c5-8e7c-8b85d2f05935.png" width="450" height="350">


## Problems we Encountered and Solved

As is normal for each project we encountered unexpected problems related to the simulation environment, robot itself, motions etc.
One of the major problems we solved is:
- We had to translate Choregraphe python output file to Webots motion file
- Standing position of the NAO in the Choregraphe is different from one in Webots simulation
- In-accurate object positioning using Camera
- Choregraphe motions were to fast in the Webots simulation
- Picking up the object

and couple of smaller ones:
- Adjusting the weight of the object
- Adjusting camera coordinate frame transformation
- Comparing Sonar/Bumper/Camera sensors on the NAO


## Motion Converter

We used Choregraphe to create desired motions and exported them in a Simplied form as a python file. 
Searching through internet we did not found any script that would converte python file to the webots .motion file.
We consider this script really useful for anybody creating NAO robot motion using Choregraphe and wanting to use it in the Webots simulation. 
Script was tested and confirmed working with all of our custom motions.

The script can be found under `/scripts/nao_to_webots_motion_converter.py`
