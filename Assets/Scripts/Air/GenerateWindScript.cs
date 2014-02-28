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
	public float radius;
	public float timeToLive;

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

		while (time > 1.0f/frequency) {
			generateWind();
			time -= 1.0f/frequency;
		}


	}

	// Clone the wind object, place it and kick it away
	void generateWind() {
		Vector3 newPosition = transform.position;
		float xr = Random.Range (-1.0f, 1.0f);
		float yr = Random.Range (-1.0f, 1.0f);
		newPosition.x += xr * xr * radius;
		newPosition.y += yr * yr * radius;

		_bubbleInstance = (GameObject)Instantiate (WindObject, newPosition, Quaternion.identity);
		_bubbleInstance.rigidbody.velocity = -(Target.rigidbody.velocity) + transform.forward * WindVelocity;
		_bubbleInstance.GetComponent<WindObjectScript> ().TimeToLive = timeToLive;
	}
}
