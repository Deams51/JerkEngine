using UnityEngine;
using System.Collections;


public class cameraPhysicTest : MonoBehaviour {

    public Transform target;
    public float smooth= 5.0f;
	
    // Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
    void  Update ()
    {
        transform.position = Vector3.Lerp (
        transform.position, target.position,
        Time.deltaTime * smooth);
    }
}
