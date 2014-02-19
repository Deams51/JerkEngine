using UnityEngine;
using System.Collections;

public class WindObjectScript : MonoBehaviour {
	public float TimeToLive;
	private float _age;
	// Use this for initialization
	void Start () {
		_age = 0;
	}
	
	// Update is called once per frame
	void Update () {
		_age += Time.deltaTime;

		if(_age >= TimeToLive) {
			Destroy(this.gameObject);
		}
	}
}
