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
	UP:   accelerate
      DOWN:   deccelerate/reverse
      LEFT:   turn left
     RIGHT:   turn right
         B:   break
	F1:   debug wheels

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
	