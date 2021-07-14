## NAO Robot Intro

The most important thing on the NAO for us was 2x Head camera we used for object recongition, while all other joints were relevant for motions.

![image](https://user-images.githubusercontent.com/48418580/125603897-e288db98-0732-488f-8171-77c382e7b307.png)

## Searching for Box

NAO spins on spot and after each in-place turn it checks whether is sees an object, if that's true it transitions to moving towards it.

## Box detection Setup

For box detection and its position relative to the camera on the NAO robot, Webots API is used. 
Detecting object position with monocular camera is quite unprecise, so we had to take that into an account. 
In case when object was seen ob both Bottom and Top Camera, Top camera was way more accurate. 
That's why we are innitialy checking whether object/drop off location is seen on the Top Camera and then as a fallback we check if the object/drop off location is seen on the Bottom camera. 
We needed to do that since we don't have any reference to determine when the Top Camera object detection is more accurate than Bottom camera and in our case the object will mostly be searched through the Top Camera.

We took aprox. measurement of the system to mathematically formulate what the robot is actually capable of seeing.
Image below denotes both camera Field of Views and gives a sense to what NAO is constrained to.

<img src="https://user-images.githubusercontent.com/48418580/125604669-9964c653-2583-44c4-a4e0-32ac0fad222f.png" width="450" height="450"> <img src="https://user-images.githubusercontent.com/48418580/125604723-97ef5800-9802-4174-a52b-7ec5fa39491a.png" width="450" height="450">

## Moving towards the box

First the robot turns towards the object such that it is aproximately perpendicular to the object. Calculated using:
	
	atan2(y, x) 

where x, y represent the position of the object in the NAO camera frames.
Once the orientation error is aprox. the same as the smallest in place turn angle NAO can do, we transition to forward movement.
Robot approaches box until it reaches a pickup distance, which is a predefined value from which robot is able to pick up the object it approached.

Distance is calculated using Euclidian formula:

	sqrt(x*x + y*y)

where x, y represent the position of the object in the NAO camera frames.

During the movement towards the box we always check whether the object is still seen and start again with the in-place rotations if we lost sight of it.

## Camera transformation to ground X,Y Plane

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

## Motion Converter

We used Choregraphe to create desired motions and exported them in a Simplied form as a python file. 
Since NAO in Webots moves using .motion files we needed to find a way/conversion such that output of Choregraphe can almost straight away used in the Webots simulation.
Therefore we created a script /scripts/nao_to_webots_motion_converter.py that can with a little bit of modification used with any Choregraphe output.
We also find this script of a general purpose since we could not find it anywhere online, so an additional bonus for the society.

## Pick Up Maneuver

In order to simplify complex picking up maneuvers we should have performed in order to accurately pick up boxed or coned shaped object we designed a T-shaped Box(T-Box) this way we only need to be aproximattely perpendicular to the box and at a distance of about 20-30 centimeters.

![image](https://user-images.githubusercontent.com/48418580/125595552-8e0e4591-7617-4bd9-b7fe-6d7239df2d5f.png)

## Searching for Drop Off location

As soon as we Picked up the object we were constrained to only one camera, since Bottom camera of the robot was covered with the T-Box, we had to rely only on Top Camera. 
Similar to as without the T-Box, robot performed in place rotation until it detected the drop off location. 
Drop off location is a blue plane area robot is able to recognize through camera stream.

## Moving towards the Drop Off location

As explained in top section Field of View of cameras are limited to a certain range, this is why as the robot was getting closer to the drop off location it dissapeared from the camera stream. 
As soon as that happened we marked the last distance measurement we got and performed a simplified version of Dead Reckoning.
For example if the last distance was aprox. 60 centimeters, we blindly performed 6 steps forward each for 10 centimeters.

## Drop Off Maneuver

As soon as we blindly aproached are drop off area, similarly to the pick up maneuvers, object was dropped and task was completed.
