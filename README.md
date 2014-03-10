JERKENGINE
====================================
A driving simulation engine v.1.0
Jerk (physics) - Rate of change of acceleration. Unit is m / s^3

AUTHORS
------------------------------------
    Anders Treptow
    Mickaël Fourgeaud
    Robin Gabre
    Max Witt


README
------------------------------------
The demo can be run from the scene "Assets/_scenes/tech_demo". 

Interesting game objects in the hierarchy are:
	camera_main 						- the main camera
	car_Peugeot 						- the car
	env_surfaces 						- The various surfaces
	env_airPlatforms/airTargets/AirTarget1-6/WindFX1-6 	- static air generating objects

CONTROLS
------------------------------------
   Controls for the car are as follows:
	UP:	Accelerate
	DOWN:   Decelerate/reverse
	LEFT:   Turn left
	RIGHT:  Turn right
	B:   	Break
	D: 	Shows rays used for collision detection
	R:	Reset the car rotation and add an up translation
	F1:   	Debug wheels


GRAPHICS
------------------------------------
In order to see the motion blur effect the project needs to run using Unity Pro since the velocity buffer uses render to texture.

The motion blur effect is located in the 'CameraMotionBlurEffect' component on camera_main (MainCamera). The effect can be switched on/off by selecting the 'Active' checkbox, or by disabling the component. The velocity buffer can be rendered by selecting the checkmark for 'Render Velocity Buffer'. The 'Blur Factor' is the amount of blurring that the screen will have. Also interesting to note is that the view of the camera can be toggled by selecting the 'Mode' on the Camera Tracker component (still on camera_main) to see the various effects from different views.

The rendering particles are located in the 'fx_speedEffect' prefab which is found in the scene by from camera_main/camera_fx/fx_speedEffect. The particle system in this prefab is controlled from the 'HighSpeedEffect' component located on the fx_speedEffect prefab. This scripts controls the amount of particle emission from the particle system by setting a minimum  and maximum amount of emission rate of particles. The velocity is displayed calculated from the 'Velocity Tracked Object' that is set in the same component but can be controlled manually by checking the checkbox for 'Manual Velocity'.

	GRAPHICS RELEVANT CODE
	------------------------------------
	Assets/Resources/Shaders
	Assets/Scripts/Graphics3.0
	Assets/Scripts/Movement/CameraTracker.cs
	Assets/Scripts/Graphics (deprecated)
	Assets/Scripts/Graphics2.0 (deprecated)


AIR & WIND
------------------------------------
Air resistance and wind objects use the same code and game objects. Everything you need can be found in the WindTarget prefab. The module consists of three objects: WindObject - the ball that acts as wind, WindFX - the object that generates WindObjects and governs their parameters, and WindTarget - A generic target which the WindFX aims at.

To create a stationary wind generator, simply drag the WindTarget into the scene. The parent object defines the target, towards which the wind will move. The child object WindFX is the actual generator and should be placed relative to the target to achieve the desired wind effect (below target for upwind, westward for a western wind). Adjust the parameters in the WindFX object and you're good to go. 

To create air resistance, use the WindFX object within the WindTarget and place it as a child to the car. Position the object just in front of the car and assign the car object as target gameobject in the WindFX script panel. The velocity of each generated wind ball will be compensated by the velocity of the target, so if the Wind Velocity parameter is set to 0, the WindObjects will appear static to the ground when the car moves.

Description of WindFX Properties:
  Script - Should be GenerateWindScript.cs
  Wind Object - Expects GameObject with Rigidbody and WindObjectScript.cs that will be spawned as wind (Should be the WindObject prefab)
  Target - Expects GameObject with Transform and Rigidbody. Can be a near-empty object, or complex object like a car.
  Wind Velocity - Float that defines the wind velocity, compensated by target velocity
  Wind Mass - Float that defines the mass of each WindObject spawned by this WindFX object
  Radius - Defines the size of the plane from which WindObject randomly will spawn
  Life Distance - How far in meters each WindObject will travel before they are destroyed
  Alive Wind Objects - How many WindObjects that may be simultaneously alive, spawned by this WindFX object


CAR PHYSICS - WHEELS
------------------------------------
To be able to use the tire model, it is required to have a gameobject, with a rigidbody applied to it, representing the car and to add the CarController and DriveTrain scripts to it. (Optionally you can add 2 AntiRollBar Scripts to stabilize the behavior of the car)
Next you have to add gameobject representing wheel(s) as child of the car gameobject and add the wheel scripts to it, but also an object representing the center of gravity of the car.
You can then add the wheel(s) to the Wheels array in the CarController script and to the PoweredWheels array in the DriveTrain script. 
It is also required to set all the parameters of each scripts depending of the car simulated.
It is possible to add a physics material to surface, the static friction coefficent will be used by the wheel script when the car is driving on it. 
You can take a look at the prefab Peugeot to see a working example.

	CAR PHYSICS - WHEELS RELEVANT CODE
	------------------------------------
	Assets/Scripts/Car/AntiRollBar.cs => This code comes from a unity example
	Assets/Scripts/Car/CarController.cs => This code comes from a unity example (only controls were added to it)
	Assets/Scripts/Car/Drivetrain.cs => This code comes from a unity example
	Assets/Scripts/Car/Wheel.cs => This code was developed during this project

AUDIO EFFECT
------------------------------------
To see what the script affects you need to target [car_Peugeot] in the hierarchy. The object have an AudioSource where you can change the sound. The effects will be seen in the Echo-filter component and the Reverb-filter component.

You cannot change the values from Unity since the script controls them on each update. The values on the Echo-filter component changes dynamically when moving the car, as well as the preset on the Reverb-filter component.
	
