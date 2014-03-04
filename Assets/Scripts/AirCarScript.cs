using UnityEngine;
using System.Collections;

public class AirCarScript : MonoBehaviour {

	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
		float throttle = Input.GetAxis ("Vertical");
		float steering = Input.GetAxis ("Horizontal");

		if (throttle != 0) {
			throttle = throttle / Mathf.Abs(throttle);
			this.rigidbody.AddForce (transform.forward*10000000*Time.deltaTime*throttle); //tror inte man egentligen ska använda Time.deltaTime
		}

		if (steering != 0) {
			steering = steering / Mathf.Abs(steering);
			this.rigidbody.angularVelocity.Set(0,10000000*steering*Time.deltaTime,0);
		} else {
			this.rigidbody.angularVelocity.Set(0,0,0);
		}
	}
}
