using UnityEngine;
using System.Collections;


public class cameraPhysicTest : MonoBehaviour {

    public Transform target;
    public Quaternion rotat;
    public Vector3 distRelat;

    // Use this for initialization
	void Start () {
        rotat = target.rotation;
        distRelat = transform.position - target.position;
	}
	
	// Update is called once per frame
    void  Update ()
    {
        //transform.rotation = relativeRotat;
        transform.position = target.position + distRelat;
    }
}
