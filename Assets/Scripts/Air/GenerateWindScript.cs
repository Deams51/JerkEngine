using UnityEngine;
using System.Collections;

public class GenerateWindScript : MonoBehaviour 
{
	// Reference to the object that will be duplicated as wind. Should have a WindObjectScript as component.
	public GameObject WindObject;
	
	// Target for direction and relative velocity calculations. Should have a transform and rigidbody
	public GameObject Target;

	// Each WindGenerator set the mass and static velocity of their own WindObjects
	public float WindVelocity;
	public float WindMass;
	
	// Spread distance from this object (in a plane normal to the direction to the target) where WindObjects may spawn
	public float radius;
	
	// Travel distance until WindObjects are automatically destroyed
	public float lifeDistance;
	
	// Number of WindObjects that are alive at any given moment, dictates spawn frequency
	public float aliveWindObjects;
	
	private float time;
	private float ttl;
	
	private GameObject _bubbleInstance;
	
	// Use this for initialization
	void Start () 
	{
		time = 0;
	}
	
	// Update is called once per frame
	void Update () 
	{
		transform.LookAt(Target.transform.position);
		time += Time.deltaTime;
		
		
		// Spawns given number of balls (aliveWindObjects) during a ball lifetime
		while (time > ttl / aliveWindObjects)
		{
			generateWind();
			time -= ttl / aliveWindObjects;
		}
	}
	
	/// <summary>Clone the wind object, place it and kick it away. The object is placed on a quadratic plane with the size 
	/// radius in both directions, with the direction from this to the target transform as normal.
	void generateWind() {
		float xr = Random.Range (-1.0f, 1.0f);
		float yr = Random.Range (-1.0f, 1.0f);
		
		float x = Mathf.Abs (xr) * xr * radius;
		float y = Mathf.Abs (yr) * yr * radius;
		
		
		_bubbleInstance = (GameObject)Instantiate (WindObject, transform.position, Quaternion.identity);
		_bubbleInstance.transform.position += (Vector3.Normalize (transform.up) * y) + (Vector3.Normalize (transform.right) * x);
		_bubbleInstance.rigidbody.velocity = -(Target.rigidbody.velocity) + transform.forward * WindVelocity;
		_bubbleInstance.rigidbody.mass = this.WindMass;
		ttl = 1f;
		float v = Mathf.Abs (_bubbleInstance.rigidbody.velocity.magnitude);
		if(v > (this.lifeDistance/2)) {
			ttl = lifeDistance/v;
		}
		_bubbleInstance.GetComponent<WindObjectScript> ().TimeToLive = ttl;
	}
}
