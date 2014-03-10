     ____.             __    ___________              .__               
    |    | ___________|  | __\_   _____/ ____    ____ |__| ____   ____  
    |    |/ __ \_  __ \  |/ / |    __)_ /    \  / ___\|  |/    \_/ __ \ 
/\__|    \  ___/|  | \/    <  |        \   |  \/ /_/  >  |   |  \  ___/ 
\________|\___  >__|  |__|_ \/_______  /___|  /\___  /|__|___|  /\___  >
              \/           \/        \/     \//_____/         \/     \/ 
A driving simulation engine v.1.0

AUTHORS
------------------------------------
    Anders Treptow
    Mickaël Fourgeaud
    Robin Gabre
    Max Witt

README
------------------------------------
The demo can be run from the scene "Assets/_scenes/tech_demo". 

CONTROLS
------------------------------------
   Controls for the car are as follows:
	UP:		accelerate
	DOWN:   decelerate/reverse
	LEFT:   turn left
	RIGHT:  turn right
	B:   	break
	D: 		Shows rays used for collision detection
	R:	  	reset the car rotation and add an up translation
	F1:   	debug wheels

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
Air resistance and wind objects use the same code and game objects. Everything you need can be found in the WindTarget prefab.

To create a stationary wind generator, simply drag the WindTarget into the scene. The parent object defines the target, towards which the wind will move. The child object WindFX is the actual generator and should be placed relative to the target to 

CAR PHYSICS - WHEELS
------------------------------------
To be able to use the tire model, it is required to have a gameobject, with a rigidbody applied to it, representing the car and to add the CarController and DriveTrain scripts to it. (Optionally you can add 2 AntiRollBar Scripts to stabilize the behavior of the car)
Next you have to add gameobject representing wheel(s) as child of the car gameobject and add the wheel scripts to it, but also an object representing the center of gravity of the car.
You can then add the wheel(s) to the Wheels array in the CarController script and to the PoweredWheels array in the DriveTrain script. 
It is also required to set all the parameters of each scripts depending of the car simulated.
You can take a look at the prefab Peugeot to see a working example.

	CAR PHYSICS - WHEELS RELEVANT CODE
	Assets/Scripts/Car/AntiRollBar.cs => This code comes from a unity example
	Assets/Scripts/Car/CarController.cs => This code comes from a unity example (only controls were added to it)
	Assets/Scripts/Car/Drivetrain.cs => This code comes from a unity example
	Assets/Scripts/Car/Wheel.cs => This code was developed during this project
	