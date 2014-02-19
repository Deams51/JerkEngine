using UnityEngine;
using System.Collections;

public class GenerateWindScript : MonoBehaviour 
{
	// Reference to the object that will be duplicated as wind. Should have a Time To Live (ref ttl)
	public GameObject WindObject;
	// Target for direction and relative velocity calculations. Should have a transform and 
	public GameObject Target;
	public float WindVelocity;
	public float frequency;
	private int _i = 0;

	private float time;

	private GameObject _bubbleInstance;

	// Use this for initialization
	void Start () 
	{
		time = 0;
	}
	
	// Update is called once per frame
	void Update () 
	{
		transform.LookAt (Target.transform.position);
		time += Time.deltaTime;
		if (time > (1.0/frequency)) {
			generateWind();
			time = 0;
		}


	}

	void generateWind() {
		_bubbleInstance = (GameObject)Instantiate (WindObject, transform.position, Quaternion.identity);
		_bubbleInstance.rigidbody.velocity = -(Target.rigidbody.velocity)+transform.forward * WindVelocity;
	}
}
