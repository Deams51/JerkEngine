/* Robin Gabre
 * Third edition EchoScript
 */


using UnityEngine;
using System.Collections;

public class RayCast_Multi : MonoBehaviour {
	
	private AudioEchoFilter _echo;
	private AudioReverbFilter _rev;
	
	private RaycastHit hr, hl, hu, hf, hb;
	
	private Vector3 addLft = new Vector3 (-1.0f,1.0f,0.0f);
	private Vector3 addRght = new Vector3 (1.0f,1.0f,0.0f);
	private Vector3 addUp = new Vector3 (0.0f,1.5f,0.0f);
	private Vector3 addBck = new Vector3 (0.0f,1.0f,-2.0f);
	private Vector3 addFrnt = new Vector3 (0.0f,1.0f,2.0f);

	private float angle, wetMixValue, delayValue;
	private int counter;

	/* Method to check if any collision occurs. The reason is to
	 * be able to use the collision as a bool for if-statements.
	 */
	private bool IsColliding (RaycastHit _hit){ 
		if(_hit.collider != null){
			return true;
		}
		return false;
	}
	
	void Start () {	
	}
	
	void Awake()
	{
		_echo = GetComponent<AudioEchoFilter> ();
		_rev = GetComponent<AudioReverbFilter> ();
	}

	void Update () {
		/* Each update starts with setting the wetmix, delay and the counter to 0.
		 * WetMix is the variable for how much the echo will affect the output sound.
		 * 
		 * The variable counter keeps track of how many collision occur during the update.
		 */
		wetMixValue = 0.0f;
		delayValue = 0.0f;
		counter = 0;

		// Creating the vectors that works as the foundation for the raycasts.
		var lft = transform.TransformDirection (Vector3.left);
		var rght = transform.TransformDirection (Vector3.right);
		var up = transform.TransformDirection (Vector3.up);
		var frnt = transform.TransformDirection (Vector3.forward);
		var bck = transform.TransformDirection (Vector3.back);

		// DebugCode to visualize the raycasts in unity.
//		Debug.DrawRay (transform.position + addLft, lft * 40, Color.red);
//		Debug.DrawRay (transform.position  + addRght, rght * 40, Color.red);
//		Debug.DrawRay (transform.position + addUp, up * 40, Color.blue);
//		Debug.DrawRay (transform.position + addBck, bck * 40, Color.green);
//		Debug.DrawRay (transform.position + addFrnt, frnt * 40, Color.green);

		/* Raycasts with extra distance from the center of the object connected.
		 * This is to compensate for the rigidbody of the object. The information
		 * of the collision is stored into separate variables.
		 */
		Physics.Raycast (transform.position + addLft, lft, out hl, 40);
		Physics.Raycast (transform.position + addRght, rght, out hr, 40);
		Physics.Raycast (transform.position + addUp, up, out hu, 40);
		Physics.Raycast (transform.position + addBck, bck, out hb, 40);
		Physics.Raycast (transform.position + addFrnt, frnt, out hf, 40);

		/* If a collision occurs the angle between the normal vector from the surface hit
		 * and the raycast is calculated and adapted to values which is easier to work with.
		 * If the angle is smaller than 50 degrees less wetmix is added to the total wetMixValue.
		 * 
		 * The delay is calculated from the distance to the collision times two. Then divided
		 * by the speed of sound to get the time it takes for the sound to bounce.
		 */
		if(IsColliding(hf)){
				angle = Vector3.Angle(frnt,hf.normal) - 90.0f;
				angle = angle / 100;
				if(angle < 0.25f){
					angle = angle / 4;
				}
				else if (angle < 0.5f){
					angle = angle / 2;
				}
				wetMixValue = wetMixValue + (angle/3);
				
				delayValue = delayValue + ((hf.distance*2.0f)/342.0f)*1000.0f;
				counter++;
		}

		if(IsColliding(hb)){
				angle = Vector3.Angle(bck,hb.normal) - 90.0f;
				angle = angle / 100;
				if(angle < 0.25f){
					angle = angle / 4;
				}
				else if (angle < 0.5f){
					angle = angle / 2;
				}
				wetMixValue = wetMixValue + (angle/3);

			delayValue = delayValue + ((hb.distance*2.0f)/342.0f)*1000.0f;
			counter++;
		}

		if(IsColliding(hl)){
				angle = Vector3.Angle(lft,hl.normal) - 90.0f;
				angle = angle / 100;
				if(angle < 0.25f){
					angle = angle / 4;
				}
				else if (angle < 0.5f){
					angle = angle / 2;
				}
				wetMixValue = wetMixValue + (angle/3);

			delayValue = delayValue + ((hl.distance*2.0f)/342.0f)*1000.0f;
			counter++;
		}

		if(IsColliding(hr)){
				angle = Vector3.Angle(rght,hr.normal) - 90.0f;
				angle = angle / 100;
				if(angle < 0.25f){
					angle = angle / 4;
				}
				else if (angle < 0.5f){
					angle = angle / 2;
				}
				wetMixValue = wetMixValue + (angle/3);

			delayValue = delayValue + ((hr.distance*2.0f)/342.0f)*1000.0f;
			counter++;
		}

		if(IsColliding(hu)){
				angle = Vector3.Angle(up,hu.normal) - 90.0f;
				angle = angle / 100;
				if(angle < 0.25f){
					angle = angle / 4;
				}
				else if (angle < 0.5f){
					angle = angle / 2;
				}
				wetMixValue = wetMixValue + (angle/3);

			delayValue = delayValue + ((hu.distance*2.0f)/342.0f)*1000.0f;
			counter++;
		}

		/* If the amounts of collisions is greater than 2 but the mean value of the time 
		 * it takes for the sound to travel is less than 0.08s regular reverb is added
		 * instead. Else, the accumulated values of wetmix and delay is added to the echo.
		 */ 
		if(counter > 0){
			delayValue = delayValue/counter;
			if(delayValue < 80.0f){
				if(counter > 2){
					_rev.reverbPreset = AudioReverbPreset.Alley;
					_echo.wetMix = 0.0f;
				}
			}else{
				_echo.wetMix = wetMixValue;
				_echo.delay = (delayValue);
				_rev.reverbPreset = AudioReverbPreset.Off;
			}

		}else{
			_rev.reverbPreset = AudioReverbPreset.Off;
		}
	}
}