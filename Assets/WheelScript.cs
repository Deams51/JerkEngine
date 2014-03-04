using UnityEngine;
using System.Collections;



public class WheelScript : MonoBehaviour {
	private float throttle = 0;

	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
		handleInput ();
	}

	void handleInput () {
		throttle = Input.GetAxis ("Vertical");

		this.rigidbody.AddRelativeForce (transform.forward * 10 * throttle);

	}
}

