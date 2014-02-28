using UnityEngine;
using System.Collections;

public class RayCast_Duo : MonoBehaviour {

	private AudioEchoFilter _echo;
	private AudioReverbFilter _rev;

	private RaycastHit hr, hl;
	private bool IsColliding (RaycastHit _hit){ 
		{
//			if(_hit != null)
//			{
				if(_hit.collider != null)
					return true;
//			}
			return false;
		}
	}
	//public Color RayCastColor;
	
	// Use this for initialization
	void Start () {
		
	}

	void Awake()
	{
		_echo = GetComponent<AudioEchoFilter> ();
		_rev = GetComponent<AudioReverbFilter> ();
	}
	
	// Update is called once per frame
	void Update () {

		var lft = transform.TransformDirection (Vector3.left);
		var rght = transform.TransformDirection (Vector3.right);
		//GameObject obj = GameObject.FindGameObjectWithTag ("ReverbZone");
		
		//GameObject ray = null;// = GameObject.FindGameObjectWithTag ("LowRayCast");
		
		
		//AudioReverbZone arz = obj.GetComponent<AudioReverbZone> ();

		Debug.DrawRay(transform.position, lft * 50, Color.red);
		Debug.DrawRay(transform.position, rght * 50, Color.red);

		//Numbers based on how humans can hear echoes, otherwise it's just reverb!
//		bool l1 = Physics.Raycast (transform.position, lft, 17);
//		bool l2 = Physics.Raycast (transform.position, lft, 50);;
//		bool r1 = Physics.Raycast (transform.position, rght, 17);
//		bool r2 = Physics.Raycast (transform.position, rght, 50);

		RaycastHit hl, hr;
		Physics.Raycast (transform.position, rght, out hr, 50);
		Physics.Raycast (transform.position, lft, out hl, 50);
		 
		Debug.Log("Left: " + hl.distance + " - " + "Right: " + hr.distance);

		//if (hl. null) {
		//	float distance = Vector3.Distance(transform.position, hl.point);
		//	Debug.Log(distance);
		//}

		if (IsColliding(hl) && IsColliding(hr)) 
		{
			if(hl.distance > 17.0f && hr.distance > 17.0f)
			{
				_rev.reverbPreset = AudioReverbPreset.Off;
				_echo.enabled.Equals(true);
				_echo.wetMix = 1.0f;
				_echo.delay = (((hl.distance + hr.distance)/2)/340.0f) * 1000.0f;
			}
			else
			{
				_echo.wetMix = 0.0f;
				_rev.reverbPreset = AudioReverbPreset.Concerthall;
			}
		}
		else if (IsColliding(hl))
		{
			if(hl.distance > 17.0f){
				_rev.reverbPreset = AudioReverbPreset.Off;
				_echo.wetMix = 0.5f;
				_echo.delay = ((hl.distance)/340.0f) * 1000.0f;
			}
			else{
				_rev.reverbPreset = AudioReverbPreset.Auditorium;
				_echo.wetMix = 0.0f;
			}
		}
		else if (IsColliding(hr))
		{
			if(hr.distance > 17.0f){
				_rev.reverbPreset = AudioReverbPreset.Off;
				_echo.wetMix = 0.5f;
				_echo.delay = ((hr.distance)/340.0f) * 1000.0f;
			}
			else{
				_echo.wetMix = 0.0f;
				_rev.reverbPreset = AudioReverbPreset.Auditorium;
			}
		}
		else
		{
			//No effects.
			_echo.wetMix = 0.0f;
			_echo.dryMix = 1.0f;
			_echo.enabled.Equals(false);
			_rev.reverbPreset = AudioReverbPreset.Off;
		}
	}
}