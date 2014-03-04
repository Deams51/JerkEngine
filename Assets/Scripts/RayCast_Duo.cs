using UnityEngine;
using System.Collections;

public class RayCast_Duo : MonoBehaviour {

	private AudioEchoFilter _echo;
	private AudioReverbFilter _rev;

	private RaycastHit hr, hl, hu;
	private float fl = 0.0f;
	private float fr = 0.0f;
	private float fu = 0.0f;

	private Vector3 addLft = new Vector3 (-1.0f,1.0f,0.0f);
	private Vector3 addRght = new Vector3 (1.0f,1.0f,0.0f);
	private Vector3 addUp = new Vector3 (0.0f,1.5f,0.0f);

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
		var up = transform.TransformDirection (Vector3.up);

		Debug.DrawRay (transform.position + addLft, lft * 50, Color.red);
		Debug.DrawRay (transform.position  + addRght, rght * 50, Color.red);
		Debug.DrawRay (transform.position + addUp, up * 50, Color.blue);
	
		Physics.Raycast (transform.position + addLft, lft, out hl, 50);
		Physics.Raycast (transform.position + addRght, rght, out hr, 50);
		Physics.Raycast (transform.position + addUp, up, out hu, 50);
	

		Debug.Log("Left: " + hl.distance + " - " + "Right: " + hr.distance + " - " + "Up: " + hu.distance);



		if (IsColliding(hl) && IsColliding(hr))
		{
			if(hl.distance > 17.0f || hr.distance > 17.0f)
			{
				_rev.reverbPreset = AudioReverbPreset.Auditorium;
				_echo.wetMix = 1.0f;
				_echo.delay = (((hl.distance + hr.distance)/2)/340.0f) * 1000.0f;
			}
			else
			{
				_echo.wetMix = 0.0f;
				_rev.reverbPreset = AudioReverbPreset.Concerthall;
			}
		}
		else if (IsColliding(hl) && IsColliding(hu)) 
		{
			if(hl.distance > 17.0f || hu.distance > 17.0f)
			{
				_rev.reverbPreset = AudioReverbPreset.Off;
				_echo.wetMix = 1.0f;
				_echo.delay = (((hl.distance + hr.distance)/2)/340.0f) * 1000.0f;
			}
			else
			{
				_echo.wetMix = 0.0f;
				_rev.reverbPreset = AudioReverbPreset.Concerthall;
			}
		}
		else if (IsColliding(hr) && IsColliding(hu)) 
		{
			if(hr.distance > 17.0f || hu.distance > 17.0f)
			{
				_rev.reverbPreset = AudioReverbPreset.Off;
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
		else if (IsColliding(hu))
		{
			if(hu.distance > 17.0f){
				_rev.reverbPreset = AudioReverbPreset.Off;
				_echo.wetMix = 0.5f;
				_echo.delay = ((hu.distance)*340.0f) * 1000.0f;
			}
			else{
				_rev.reverbPreset = AudioReverbPreset.Auditorium;
				_echo.wetMix = 0.0f;
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





//if (IsColliding (hl)){
//	if (hl.distance > 17.0f) {
//		if (fl < 0.35f) {
//			fl = fl + 0.05f;
//			_echo.wetMix = _echo.wetMix + 0.05f;
//		}
//	}else{
//		_rev.reverbPreset = AudioReverbPreset.Auditorium;
//		if(fl > 0.0){
//			fl = fl - 0.05f;
//			_echo.wetMix = _echo.wetMix - 0.05f;
//		}
//	}
//}
//
//if (IsColliding (hr)){
//	if(hl.distance > 17.0f){
//		if(fr < 0.35f){
//			fr = fr + 0.05f;
//			_echo.wetMix = _echo.wetMix + 0.05f;
//		}
//	}else{
//		_rev.reverbPreset = AudioReverbPreset.Auditorium;
//		if(fr > 0.0){
//			fr = fr - 0.05f;
//			_echo.wetMix = _echo.wetMix - 0.05f;
//		}
//	}
//}
//
//if (IsColliding (hu)){
//	if(hl.distance > 10.0f){
//		if(fu < 0.3f){
//			fu = fu + 0.1f;
//			_echo.wetMix = _echo.wetMix + 0.1f;
//		}
//	}else{
//		_rev.reverbPreset = AudioReverbPreset.Auditorium;
//		if(fu > 0.0){
//			fu = 0.0f;
//			_echo.wetMix = _echo.wetMix - 0.35f;
//		}
//		
//	}
//}
//
//if (!(IsColliding (hl)))
//	if(_echo.wetMix > 0.0f)
//		if(fl > 0.0f)
//			if(fl > 0.0f)
//				fl = 0.0f;
//_echo.wetMix = _echo.wetMix - 0.35f;
//
//if (!(IsColliding (hr)))
//	if(_echo.wetMix > 0.0f)
//		if(fr > 0.0f)
//			fr = 0.0f;
//_echo.wetMix = _echo.wetMix - 0.35f;
//
//if (!(IsColliding (hu)))
//	if(_echo.wetMix > 0.0f)
//		if(fu > 0.0f)
//			fu = 0.0f;
//_echo.wetMix = _echo.wetMix - 0.3f;
//
//if (hl.distance == 0.0f && hr.distance == 0.0f && hu.distance == 0.0f)
//	_rev.reverbPreset = AudioReverbPreset.Off;
//_echo.wetMix = 0.0f;
//
//print (fl + " - " + fr + " - " + fu);