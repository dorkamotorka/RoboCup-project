## Searching for Box

NAO spins on spot and after each in-place turn it checks whether is sees an object, if that's true it transitions to moving towards it.

## Box detection

For box detection and its position relative to the camera on the NAO robot, Webots API is used. 
Detecting object position with monocular camera is quite unprecise, so we had to take that into an account. In case when object was seen ob both Bottom and Top Camera, Top camera was way more accurate. That's why we are innitialy checking whether object is seen on the Top Camera and then as a fallback we check if the object is seen on the Bottom camera. We needed to do that since we don't have any reference to determine when the Top Camera object detection is more accurate than Bottom camera and in our case the object will mostly be searched through the Top Camera.

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

