using UnityEngine;
using System.Collections;

// This class simulates a single car's wheel with tire, brake and simple
// suspension (basically just a single, independant spring and damper).
public class Wheel : MonoBehaviour {

	public bool debug = true;
    public bool debugRay = false;
        
	// Wheel Specifications

    // Wheel radius in meters
    public float radius = 10.34f;
    // Wheel width in meters
    public float width = 2.00f;
	// Wheel suspension travel in meters
	public float suspensionTravel = 0.2f;
	// Damper strength in kg/s
	public float damping = 5000;
	// Wheel angular inertia in kg * m^2
	public float inertia = 2.2f;
	// Coeefficient of grip - this is simly multiplied to the resulting forces, 
	// so it is not quite realitic, but an easy way to quickly change handling characteritics
	public float grip = 1.0f;
	// Maximal braking torque (in Nm)
	public float brakeFrictionTorque = 4000;
	// Maximal handbrake torque (in Nm)
	public float handbrakeFrictionTorque = 0;
	// Base friction torque (in Nm)
	public float frictionTorque = 10;
	// Maximal steering angle (in degrees)
	public float maxSteeringAngle = 28f;
	// Graphical wheel representation (to be rotated accordingly)
	public GameObject model;
	// Fraction of the car's mass carried by this wheel
	public float massFraction = 0.25f;
	// Pacejka coefficients
	public float[] a={1.0f,-60f,1688f,4140f,6.026f,0f,-0.3589f,1f,0f,-6.111f/1000f,-3.244f/100f,0f,0f,0f,0f};
	public float[] b={1.0f,-60f,1588f,0f,229f,0f,0f,0f,-10f,0f,0f};

	// inputs
	// engine torque applied to this wheel
	public float driveTorque = 0;
	// engine braking and other drivetrain friction torques applied to this wheel
	public float driveFrictionTorque = 0;
	// brake input
	public float brake = 0;
	// handbrake input
	public float handbrake = 0;
	// steering input
	public float steering = 0;
	// drivetrain inertia as currently connected to this wheel
	public float drivetrainInertia = 0;
	// suspension force externally applied (by anti-roll bars)
	public float suspensionForceInput = 0;
	
	// output
	public float angularVelocity;
	public float slipRatio;
	public float slipVelo;
	public float compression;
	
	// state
	float fullCompressionSpringForce;
	Vector3 wheelVelo;
	Vector3 localVelo;
	Vector3 groundNormal;
	float rotation;
	float normalForce;
	Vector3 suspensionForce;
	Vector3 roadForce;
	Vector3 up, right, forward;
	Quaternion localRotation = Quaternion.identity;
	Quaternion inverseLocalRotation = Quaternion.identity;	
	float slipAngle;
	int lastSkid = -1;
	
	// cached values
	Rigidbody body;
	float maxSlip;
	float maxAngle;
	float oldAngle;	
	Skidmarks skid;

    RaycastHit hit;
    public bool onGround;
    public bool steeringWheel;
    public float Fx;
    public float Fy;

    public Vector3 FBase;
    public Vector3 F;


    // Debug variables
    public bool debugSuspensions = false;
    public bool debugRoadForce = false;
    public bool debugWheelsXAxis = false;
    public bool debugWheelsVelo = false;
    public bool debugCollisions = false;
	
	float CalcLongitudinalForce(float Fz,float slip)
	{
		Fz*=0.001f;//convert to kN
		slip*=100f; //covert to %
		float uP=b[1]*Fz+b[2];
		float D=uP*Fz;	
		float B=((b[3]*Fz+b[4])*Mathf.Exp(-b[5]*Fz))/(b[0]*uP);
		float S=slip+b[9]*Fz+b[10];
		float E=b[6]*Fz*Fz+b[7]*Fz+b[8];
		float Fx=D*Mathf.Sin(b[0]*Mathf.Atan(S*B+E*(Mathf.Atan(S*B)-S*B)));
		return Fx;
	}
	
	float CalcLateralForce(float Fz,float slipAngle)
	{
		Fz*=0.001f;//convert to kN
		slipAngle*=(360f/(2*Mathf.PI)); //convert angle to deg
		float uP=a[1]*Fz+a[2];
		float D=uP*Fz;
		float B=(a[3]*Mathf.Sin(2*Mathf.Atan(Fz/a[4])))/(a[0]*uP*Fz);
		float S=slipAngle+a[9]*Fz+a[10];
		float E=a[6]*Fz+a[7];
		float Sv=a[12]*Fz+a[13];
		float Fy=D*Mathf.Sin(a[0]*Mathf.Atan(S*B+E*(Mathf.Atan(S*B)-S*B)))+Sv;
        return Fy;
	}
	
	float CalcLongitudinalForceUnit(float Fz,float slip)
	{
		return CalcLongitudinalForce(Fz,slip*maxSlip);
	}
	
	float CalcLateralForceUnit(float Fz,float slipAngle)
	{
		return CalcLongitudinalForce(Fz,slipAngle*maxAngle);
	}

	Vector3 CombinedForce(float Fz,float slip,float slipAngle)
	{
		float unitSlip = slip/maxSlip;
		float unitAngle = slipAngle/maxAngle;
		float p = Mathf.Sqrt(unitSlip*unitSlip + unitAngle*unitAngle);
		if(p > Mathf.Epsilon)
		{
			if (slip < -0.8f)
				return -localVelo.normalized * (Mathf.Abs(unitAngle/p * CalcLateralForceUnit(Fz,p)) + Mathf.Abs(unitSlip/p * CalcLongitudinalForceUnit(Fz,p)));
			else
			{
				Vector3 forward = new Vector3( 0, -groundNormal.z, groundNormal.y);
				return Vector3.right * unitAngle/p * CalcLateralForceUnit(Fz,p) + forward * unitSlip/p * CalcLongitudinalForceUnit(Fz,p);
			}
		}
		else
			return Vector3.zero;
	}

	void InitSlipMaxima()
	{
		const float stepSize = 0.001f;
		const float testNormalForce = 4000f;
		float force = 0;
		for (float slip = stepSize;;slip += stepSize)
		{
			float newForce = CalcLongitudinalForce(testNormalForce,slip);
			if (force<newForce)
				force = newForce;
			else {
				maxSlip = slip-stepSize;
				break;
			}
		}
		force = 0;
		for (float slipAngle = stepSize;;slipAngle += stepSize)
		{
			float newForce = CalcLateralForce(testNormalForce,slipAngle);
			if (force<newForce)
				force = newForce;
			else {
				maxAngle = slipAngle-stepSize;
				break;
			}
		}
	}
	
	void Start () {
		Transform trs = transform;
		while (trs != null && trs.rigidbody == null)
			trs = trs.parent;
		if (trs != null)
			body = trs.rigidbody;

		InitSlipMaxima ();
		skid = FindObjectOfType(typeof(Skidmarks)) as Skidmarks;
		fullCompressionSpringForce = body.mass * massFraction * 2.0f * -Physics.gravity.y;


        // Tests
        deflectionsInit(n);
	}
	
	Vector3 SuspensionForce () {
		float springForce = compression * fullCompressionSpringForce;
		normalForce = springForce;
		
		float damperForce = Vector3.Dot(localVelo, groundNormal) * damping;

		return (springForce - damperForce + suspensionForceInput) * up;
	}
	
	float SlipRatio ()
	{
		const float fullSlipVelo = 4.0f;

		float wheelRoadVelo = Vector3.Dot (wheelVelo, forward);
		if (wheelRoadVelo == 0)
			return 0;
		
		float absRoadVelo = Mathf.Abs (wheelRoadVelo);
		float damping = Mathf.Clamp01( absRoadVelo / fullSlipVelo );
		
		float wheelTireVelo = angularVelocity * radius;
		return (wheelTireVelo - wheelRoadVelo) / absRoadVelo * damping;
	}

	float SlipAngle ()
	{
		const float fullAngleVelo = 2.0f;
		
		Vector3 wheelMotionDirection = localVelo;
		wheelMotionDirection.y = 0;

		if (wheelMotionDirection.sqrMagnitude < Mathf.Epsilon)
			return 0;
				
		float sinSlipAngle = wheelMotionDirection.normalized.x;
		Mathf.Clamp(sinSlipAngle, -1, 1); // To avoid precision errors.

		float damping = Mathf.Clamp01( localVelo.magnitude / fullAngleVelo );
		
		return -Mathf.Asin(sinSlipAngle) * damping * damping;
	}
	
	Vector3 RoadForce () {
		int slipRes=(int)((100.0f-Mathf.Abs(angularVelocity))/(10.0f));
		if (slipRes < 1)
			slipRes = 1;
		float invSlipRes = (1.0f/(float)slipRes);
		
		float totalInertia = inertia + drivetrainInertia;
		float driveAngularDelta = driveTorque * Time.deltaTime * invSlipRes / totalInertia;
		float totalFrictionTorque = brakeFrictionTorque * brake + handbrakeFrictionTorque * handbrake + frictionTorque + driveFrictionTorque;
		float frictionAngularDelta = totalFrictionTorque * Time.deltaTime * invSlipRes / totalInertia;

		Vector3 totalForce = Vector3.zero;
		float newAngle = maxSteeringAngle * steering;
		for (int i=0; i<slipRes; i++)
		{
			//float f = i * 1.0f/(float)slipRes;
			//localRotation = Quaternion.Euler (0, oldAngle + (newAngle - oldAngle) * f, 0); 		
			//inverseLocalRotation = Quaternion.Inverse(localRotation);
			//forward = transform.TransformDirection (localRotation * Vector3.forward);
			//right = transform.TransformDirection (localRotation * Vector3.right);
			
			//slipRatio = SlipRatio ();
			//slipAngle = SlipAngle ();
			//Vector3 force = invSlipRes * grip * CombinedForce (normalForce, slipRatio, slipAngle);
			//Vector3 worldForce = transform.TransformDirection (localRotation * force);
			//angularVelocity -= (force.z * radius * Time.deltaTime) / totalInertia;
			angularVelocity += driveAngularDelta;
			if (Mathf.Abs(angularVelocity) > frictionAngularDelta)
				angularVelocity -= frictionAngularDelta * Mathf.Sign(angularVelocity);
			else
				angularVelocity = 0;
				
			//wheelVelo += worldForce* (1/body.mass) * Time.fixedDeltaTime * invSlipRes;
			//totalForce += worldForce;
		}

		//float longitunalSlipVelo = Mathf.Abs(angularVelocity * radius - Vector3.Dot (wheelVelo, forward));	
		//float lateralSlipVelo = Vector3.Dot (wheelVelo, right);
		//slipVelo = Mathf.Sqrt(longitunalSlipVelo * longitunalSlipVelo + lateralSlipVelo * lateralSlipVelo);
		
		//oldAngle = newAngle;
		return totalForce;
	}
	
    // Check if a hitpoint is inside the cylinder
    bool inCylinder(Vector3 hitPoint, Vector3 center, float radius, float width)
    {
        // Seems like the orientation is always on x axis
        Vector3 orientation = new Vector3(1.0f,0.0f,0.0f); 
        Vector3 pointRel = transform.InverseTransformPoint(hitPoint); // center is now origin
        // dist on direction = pointRel scalar orientation
        float distRelDir = Vector3.Dot(pointRel, orientation) / orientation.magnitude; // Division shouldn't be necessary since orientation is normalized.
        // In case vector is outside the tire
        if (distRelDir > width / 2 || distRelDir < -width / 2)
            return false;
        return true;
    }

    public struct rayElem
    {
        Vector3 center;
        Vector3 direction;
        float length;
        public rayElem(Vector3 c, Vector3 d, float l)
        {
            center = c;
            direction = d;
            length = l;
        }
        public Vector3 c()
        {
            return center;
        }
        public Vector3 d()
        {
            return direction;
        }
        public float l()
        {
            return length;
        }
    }
    rayElem[] listRays(Vector3 center, float radius, float width, int rPrecision, int wPrecision)
    {
        int numberRays = rPrecision * wPrecision;
        int cpt = 0;
        float widthInc = width / wPrecision;
        float rInc = 180.0f / (float)rPrecision;
        bool showRays = false;
        rayElem[] tabRays = new rayElem[numberRays];
        Quaternion rotate = Quaternion.Euler(0, maxSteeringAngle * steering, 0);
        //Quaternion rotate = this.model.transform.;

        if (debugRay)
        {
            debugRay = false;
            showRays = true;
        }
        for (int w = 0; w < wPrecision; w++)
        {
            for (float a = 0.0f; a < 180.0f; a += rInc)
            {
                Vector3 translation = new Vector3((-width/2) + w*widthInc, -suspensionTravel, 0);
                Vector3 direction = new Vector3(0, -radius * Mathf.Sin(a * Mathf.PI / 180), radius * Mathf.Cos(a * Mathf.PI / 180));

                tabRays[cpt] = new rayElem(transform.TransformPoint( translation), transform.TransformDirection(rotate * direction), radius);
                //Logger("cos : " + -radius * Mathf.Cos(a * Mathf.PI / 180) + " sin : " + radius * Mathf.Sin(a * Mathf.PI / 180));
                if (showRays)
                {
                    Debug.DrawRay(tabRays[cpt].c(), tabRays[cpt].d(), Color.blue, 10.0f);
                }
                cpt++;
            }
        }
        return tabRays;
    }

    public void checkOnGround()
    {
        Vector3 up = transform.up;
        Vector3 centerW = model.transform.position;// transform.position + ((-up) * suspensionTravel);
        onGround = false;
        float dist2 = radius;
        
        hit = new RaycastHit();
        rayElem[] tab = listRays(centerW, radius, width, 26, 4);
        Debug.DrawRay(centerW, up);

        foreach (rayElem r in tab)
        {
            RaycastHit[] hts = Physics.RaycastAll(r.c(), r.d(), r.l());
            foreach (RaycastHit h in hts)
            {
                if (!h.collider.isTrigger && h.collider.name != "Collider" && h.distance < dist2)
                {
                    hit = h;
                    onGround = true;
                    dist2 = h.distance;
                    //Debug.DrawLine(hitAlternate.point, centerW + new Vector3(hitAlternate.point.x - pos.x, 0, 0), Color.green, 10.0f);

                }
            }
        }
    }

    void FixedUpdate()
    {
/*
    }

    public void PhysUpdate()
    {
*/
        Vector3 pos = transform.position;
        up = transform.up;

        checkOnGround();


        Vector3 centerW = pos +((-up) * suspensionTravel);

		if (onGround)
        {
            if (hit.collider.sharedMaterial != null)
            {
                muslx = hit.collider.sharedMaterial.staticFriction;
                musly = hit.collider.sharedMaterial.staticFriction;
            }
            else
            {
                muslx = 1.0f;
                musly = 1.0f;
            }
            Vector3 groundNormalRCM = transform.InverseTransformDirection(inverseLocalRotation * hit.normal);
            //Debug.DrawLine(hit.point, centerW, Color.green, 10.0f);
            if (debugCollisions) Debug.DrawRay(hit.point, groundNormalRCM /*centerW*/ , Color.red, 10.0f);
            //Logger("Colliding with : " + hitAlternate.collider.name);
            groundNormal = groundNormalRCM; // groundNormalRCM; // transform.InverseTransformDirection(inverseLocalRotation * hit.normal);

            //Logger("RCM - dist : " + (hitAlternate.distance + suspensionTravel - radius) + " normal : " + groundNormalRCM
             //+ "\nRC - dist : " + (hit.distance - radius) + " normal : " + groundNormal);

            compression = 1.0f - ((hit.distance + suspensionTravel - radius) / suspensionTravel);
			
            wheelVelo = body.GetPointVelocity (pos);
			localVelo = transform.InverseTransformDirection (inverseLocalRotation * wheelVelo);
			suspensionForce = SuspensionForce ();
            roadForce = RoadForce();

            // TESTS
            FCalc(angularVelocity, steering * maxSteeringAngle, normalForce, n);
            body.AddForceAtPosition(new Vector3(0.0f, suspensionForce.y, 0.0f) + F, pos);
            //body.AddRelativeTorque(Torque);
            if (debugRoadForce) Debug.DrawLine(model.transform.position, model.transform.position + F, Color.blue);
            if (debugSuspensions) Debug.DrawLine(model.transform.position, model.transform.position + (suspensionForce / 5000), Color.blue);
            FBase = roadForce;
            //body.AddForceAtPosition (suspensionForce + roadForce, pos);
		}
		else
		{
			//Logger("not on ground");
			deflectionsInit(n);
			compression = 0.0f;
			suspensionForce = Vector3.zero;
			roadForce = Vector3.zero;
			float totalInertia = inertia + drivetrainInertia;
			float driveAngularDelta = driveTorque * Time.deltaTime / totalInertia;
			float totalFrictionTorque = brakeFrictionTorque * brake + handbrakeFrictionTorque * handbrake + frictionTorque + driveFrictionTorque;
			float frictionAngularDelta = totalFrictionTorque * Time.deltaTime / totalInertia;
			angularVelocity += driveAngularDelta;
			if (Mathf.Abs(angularVelocity) > frictionAngularDelta)
				angularVelocity -= frictionAngularDelta * Mathf.Sign(angularVelocity);
			else
				angularVelocity = 0;
			slipRatio = 0;
			slipVelo = 0;
		}

		if (skid != null && Mathf.Abs(slipRatio) > 0.2)
            lastSkid = skid.AddSkidMark(hit.point, hit.normal, Mathf.Abs(slipRatio) - 0.2f, lastSkid);
		else
			lastSkid = -1;

		compression = Mathf.Clamp01 (compression);
		rotation += angularVelocity * Time.deltaTime;
		if (model != null)
		{
			model.transform.localPosition = Vector3.up * (compression - 1.0f) * suspensionTravel;
			model.transform.localRotation = Quaternion.Euler (Mathf.Rad2Deg * rotation, maxSteeringAngle * steering, 0);
		}

	}

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.D))
        {
            debugRay = true;
        }
        if (Input.GetKeyDown(KeyCode.F))
        {
            body.AddForce(Physics.gravity * 5000);
        }

        if (Input.GetKey(KeyCode.UpArrow))
        {
            //angularVelocity += 1;
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            //angularVelocity -= 1;
        }
    }
	
	void Logger(string log)
	{
        if (debug) Debug.Log(this.name + ": " + log + "\n");
	}
    /*
    void AverageLumpedModel()
    {
        Vector3 F = new Vector3();
        Vector3 Fn = new Vector3();

        Fn =   ;// integral de 0 à L (fn(bizarre)dbizarre)
        F = (t0 * zt + t1 * Vzt + t2 * Vr) * Fn;
    }*/





    // tire model at : http://code.eng.buffalo.edu/dat/sites/tire/tire.html
    // Not working yet
    void BuffaloTire(float Fa)
    {
        float Fz = massFraction * body.mass * Physics.gravity.magnitude;
        /* mu :  nominal coefficient of friction and has a value of 0.85 for normal road conditions,
         *       0.3 for wet road conditions, and 0.1 for icy road conditions. */
        float mu0 = 0.85f; 

                    float Fzt = 810f;   // Given by table
                    float Tw = 6f;      // Given by table
                    float Tp = 24f;     // Given by table
                float ap0 = ( 0.0768f *  Mathf.Sqrt(Fz*Fzt) ) / ( Tw * (Tp + 5) );
                float A0 = 914.02f;     // Given by table
                float A1 = 12.9f;       // Given by table
                float A2 = 2028.24f;    // Given by table
            float Ks = (2.0f/(ap0*ap0) * (A0 + (A1*Fz) - ( (A1/A2) * Fz * Fz )));
                    
                    Vector3 wheelVelocity = body.rigidbody.GetPointVelocity(transform.position);
                    Vector3 wheelVeloInW = transform.InverseTransformDirection(wheelVelocity);
                float u = wheelVeloInW.z; // velocity of the wheel on xW axis
                float v = wheelVeloInW.x; // velocity of the wheel on orth direction (lateral)
            float alpha = Mathf.Atan(v / u); // Angle between VxW axis and vW the velocity of the wheel
                if (u == 0) alpha = 0;
            
            float S;
            if (u > radius*angularVelocity) S = (u - radius*angularVelocity)/u;
            else  S = (radius*angularVelocity - u)/radius*angularVelocity;

                    float CSFZ = 18.7f; // Given by table
                float Kc =(2.0f/(ap0*ap0)) *Fz * (CSFZ) ;
            float kpc = Kc + (Ks - Kc)*Mathf.Sqrt(Mathf.Pow(Mathf.Sin(alpha),2) + S*S*Mathf.Pow(Mathf.Cos(alpha),2));

                float C1 = 1.0f; // Given
                float C2 = 0.34f; // Given
                float C3 = 0.57f; // Given
                float C4 = 0.32f; // Given
                    float Ka = 0.05f; // Given
                    float ap = ap0 * (1 - (Ka * (Fx/Fz))); // Using old version of Fx  
                //Debug.Log("ap = " + ap + " ap0 = " + ap0 + " Fx = " + Fx + " Fz = " + Fz);
                float teta = ( (Mathf.PI*ap*ap) / ( 8.0f * mu0 * Fz) ) * Mathf.Sqrt( (Ks*Ks * Mathf.Tan(alpha) * Mathf.Tan(alpha)) + ( Kc*Kc*(S/(1-S))) );
                float teta2 = teta * teta;
                float teta3 = teta2 * teta;


            float fteta =( (C1*teta3) + (C2*teta2) + ((4/Mathf.PI)*teta) ) / ( (C1*teta3) + (C3*teta2) + (C4*teta) + 1);
           // Debug.Log("teta = " + teta + " mu0 = " + mu0 + " ap =" + ap + " Fz = " + Fz + " Ks = " + Ks + " Kc = " + Kc + " S = " + S + " alpha = " + alpha + "tan(alpha) = " + Mathf.Tan(alpha));
            float tan2 = Mathf.Tan(alpha) * Mathf.Tan(alpha);
            float temp = Mathf.Sqrt((Ks*Ks*tan2) + (kpc*kpc*S*S));
                
                float Kmu = 0.124f; // Givern
            float mu = mu0*(1 - (Kmu * Mathf.Sqrt(Mathf.Sin(alpha)*Mathf.Sin(alpha) + S*S*Mathf.Cos(alpha)*Mathf.Cos(alpha))));

        Fx = (fteta*kpc*S*mu*Fz) / temp ;
        Fy = ( fteta*Ks*Mathf.Tan(alpha)*mu*Fz) / temp ;
        
        //Debug.Log("fteta = " + fteta + " ks = " + Ks + " alpha = " + alpha + " mu = " + mu + " Fz = " + Fz + " temp = " + temp);
    }


    // OTHER MODEL : LUMPED


    public struct defl_struct
    {
        // Deflection on x
        public float x;
        // Deflection on y
        public float y;
        // 
        // Update current deflection
        public void update(Vector2 vWheel, float radius, float vAng, float delta)
        {
            x += ((radius * vAng) - (vWheel.x)) * delta;
            y += (vWheel.y) * delta;
        }

        // Constructor
        public defl_struct(float x_, float y_)
        {
            x = x_;
            y = y_;
        }
    }

    public defl_struct[] deflections;
    public float L = 0.02f;
    public float W = 0.01f;
    public int n = 100;
    public float wr;
    public float wr_old = 0.0f;
    public Vector2 vWheel;
    public Vector2 vWheel_old = Vector2.zero;
    public float teta1x = 0.0f;
    public float teta2x = 0.0f;
    public float teta1y = 0.0f;
    public float teta2y = 0.0f;
    public float torque;
    public Vector3 Torque;
    public float vrxDebug;
    public float vryDebug;
    public float Fz;
    public float defX;
    public float defY;
    public bool tests = false;
    public float muslx = 0.8f; // 0.8 for dry asphalt, 0.6 for wet asphalt, 0.2 for snow, 0.1 for ice?
    public float musly = 0.8f; // 0.8 for dry asphalt, 0.6 for wet asphalt, 0.2 for snow, 0.1 for ice?
    public float mugvr = 1f;


    // Creating deflections arrays for n bristles
    // At the start no bristle is deflected
    // 2D array : bristle deflection on x (deflections[i][0]) and y (deflections[i][1])
    public void deflectionsInit(int n)
    {
        deflections = new defl_struct[n];
        for (int i=0; i < n; i++)
        {
            deflections[i] = new defl_struct(0.0f, 0.0f);
        }
    }
    public void updateDeflection(Vector2 vWheel, float vAng, float delta, int n)
    {
        float wr = (-radius * vAng);
        int nbr_bristles = Mathf.Abs((int)(wr * delta * n / L)) + 1;
        float subDelta = (delta/n) + (delta / n) * (int)(nbr_bristles/n);

//        float subDelta = delta / n;

        for (int i = 1; i < n; i++)
        {
            deflections[i].x = ((radius * vAng) - (vWheel.x)) * subDelta*i;
            deflections[i].y = (vWheel.y) * subDelta*i;
            if (tests)
                Debug.Log("deflection i : " + i + " = (" + deflections[i].x + "; " + deflections[i].y
                                + ")"); 
        }
    }
    /*
    public void updateDeflection(Vector2 vWheel, float vAng, float delta, int n)
    {
        // replacing missing data due to delta
        // number of bristles concerned
        float wr = (-radius * vAng);
        int nbr_bristles = Mathf.Abs((int)(wr * delta * n / L)) + 1;
        float wr_delta = (wr - wr_old) / nbr_bristles;
        Vector2 vWheel_delta = (vWheel - vWheel_old) / nbr_bristles;
        float subDelta = delta / nbr_bristles;
        //if (nbr_bristles > n) Debug.Log("nbr_bristles > n ==> nbr_bristles = " + nbr_bristles + " n = " + n);

        float a = 0.5f;
        int b = 2*nbr_bristles%n;
        int c = 2 * nbr_bristles;
        float wr_old2 = 2 * wr_old;
        Vector2 vWheel_old2 = 2 * vWheel_old;

        for (int i = 0; i < Mathf.Min(n, nbr_bristles); i++)
        {
            float sumWr = a * i * (wr_delta * (i - b + c + 1) + wr_old);
            float sumVwheelX = a * i * (vWheel_delta.x * (i - b + c + 1) + vWheel_old2.x);
            float sumVwheelY = a * i * (vWheel_delta.y * (i - b + c + 1) + vWheel_old2.y);
            deflections[i].x = (sumWr + sumVwheelX)*subDelta;
            deflections[i].y = (sumVwheelY)*subDelta;
            if (i==(nbr_bristles % n) / 2)
            {
                defX = deflections[i].x;
                defY = deflections[i].y;
            }
            if(tests)
                Debug.Log("deflection i : " + i + " = (" + deflections[i].x + "; " + deflections[i].y
                                + "), variation de (" + (sumWr + sumVwheelX) * subDelta + "; " + a * i * (vWheel_delta.y * (i - b + c + 1) + vWheel_old2.y) + ") en delta = " + subDelta + "secondes");
        }
        for (int i = nbr_bristles%n; i < n; i++)
        {
            deflections[i].update(vWheel, radius, vAng, delta);
        }
        wr_old = wr;
        vWheel_old = vWheel;
    }
    /*
    public void updateDeflection(Vector2 vWheel, float vAng, float delta, int n)
    {
        defl_struct newBristle, temp;

        // replacing missing data due to delta
        // number of bristles concerned
        float wr = (-radius * vAng);
        int nbr_bristles = Mathf.Abs((int)(wr * delta * n / L)) + 1;
        float wr_delta = (wr - wr_old) / nbr_bristles;
        Vector2 vWheel_delta = (vWheel - vWheel_old) / nbr_bristles;
        float subDelta = delta / nbr_bristles;
        //if (nbr_bristles > n) Debug.Log("nbr_bristles > n ==> nbr_bristles = " + nbr_bristles + " n = " + n);
        for (int i = 0; i < nbr_bristles; i++)
        {
            newBristle = new defl_struct(0.0f, 0.0f);
            temp = deflections[0];
            deflections[0] = newBristle;

            //Debug.Log("delta = " + delta);        
            for (int j = 1; j < n; j++)
            {
                temp.update(vWheel_old + (i + 1) * vWheel_delta, radius, wr_old + (i + 1) * wr_delta, subDelta);
                newBristle = deflections[j];
                deflections[j] = temp;
                temp = newBristle;
                if (tests && i == nbr_bristles - 1 && j == (int)(n / 2))
                    Debug.Log("deflection j-1 : " + j + " = (" + deflections[j - 1].x + "; " + deflections[j - 1].y
                        + "), variation de (" + ((-radius * (wr_old + ((i + 1) * wr_delta))) + ((vWheel_old + ((i + 1) * vWheel_delta)).x)) * subDelta + "; " + ((vWheel_old + (i + 1) * vWheel_delta).y) * subDelta + ") en " + subDelta + "secondes");
            }
        }

        wr_old = wr;
        vWheel_old = vWheel;
    }
    */
    // normalized normal pressure distribution
    public float nnpd()
    {
        return 1.0f/(float)n;
    }

    public float zx(int i)
    {
        if (i < 0) return 0.0f;
        return deflections[i].x;
    }

    public float zy(int i)
    {
        if (i < 0) return 0.0f;
        return deflections[i].y;
    }

    // Phi for a bristle i
    public Vector2 Phi(int i, Vector2 vWheel, float vAng, float alpha, float Fz, int n)
    {
        float phix; // result
        float phiy; // result
        float teta0x = muslx * (3.16f * 100000.0f * Fz);
        float teta0y = musly * ((-0.18f * Fz * Fz) + (1.88f * Fz) + 1.84f) * 100000.0f;
        this.Fz = Fz;

        float muS = -0.054f * Fz + 1.887f; // Given by tables
        float muC = -0.022f * Fz + 0.734f; // Given by tables
        float Vs = -3.5f; // Given by tables        
        float d = 0.6f; // Given by tables

        float Fc = muC * Fz; // Coulomb friction force
        float Fs = muS * Fz; // maximum static friction force

        float vrx = (radius * vAng) + (vWheel.x);
        vrxDebug = vrx;
        float vry = vWheel.y;
        vryDebug = vry;
        float vrAbs = Mathf.Sqrt(vrx * vrx + vry * vry);

        float gvr = mugvr * ( Fc + ((Fs - Fc) * Mathf.Exp(-Mathf.Pow(Mathf.Abs(vrAbs / Vs), d))));

        float dzx = vrx - ((teta0x * Mathf.Abs(vrAbs) / gvr) * zx(i)) - (radius * Mathf.Abs(vAng) * ((n - 1) / L) * (zx(i) - zx(i - 1)));
        float dzy = vry - ((teta0y * Mathf.Abs(vrAbs) / gvr) * zy(i)) - (radius * Mathf.Abs(vAng) * ((n - 1) / L) * (zy(i) - zy(i - 1)));

        float part1 = (nnpd() * L * W) / Fz;
        // phiX calculus
        float part2x = (teta0x * zx(i)) + (teta1x * dzx) + (teta2x * vrx);
        
        // phiY calculus
        float part3y = (teta0y * zy(i)) + (teta1y * dzy) + (teta2y * vry);
        
        
        phix = part1 * part2x;
        phiy = part1 * part3y;
        //Debug.Log("vrx = " + vrx + " vrabs = " + vrAbs + " zx(i) = " +  zx(i) + " radius = " + radius + " vAngAbs = " + vAng + " L = " + L  
        //   + "\nphi = " + phix + " part1 = " + part1 + " part2 = " + part2 + " dz = " + dzx + " gvr = " + gvr );
        //Debug.Log("phiY = " + phiy);
        return new Vector2(phix,phiy);
    }

    public Vector2 integral(Vector2 vWheel, float vAng, float alpha, float Fz, int n)
    {
        float resx = 0.0f;
        float resy = 0.0f;
        float resT = 0.0f;
        float sumx = 0.0f;
        float sumy = 0.0f;
        float sumT = 0.0f;

        float coef = L/(2*n);
        Vector2 phiInit = Phi(0, vWheel, vAng, alpha, Fz, n);
        sumx += phiInit.x;
        sumy += phiInit.y;
        sumT += phiInit.y*((L/2)-zy(0)) ;
        for (int i = 1; i < n - 1; i++)
        {
            Vector2 phi = Phi(i, vWheel, vAng, alpha, Fz, n);
            sumx += (2 * phi.x);
            sumy += (2 * phi.y);
            sumT += (2 * phi.y*((L / 2) - zy(i)));
        }
        Vector2 phiLast = Phi(n - 1, vWheel, vAng, alpha, Fz, n);
        sumx += phiLast.x;
        sumy += phiLast.y;
        sumT += phiLast.y * ((L / 2) - zy(n-1));

        resx = coef * sumx;
        resy = coef * sumy;
        resT = coef * sumT;

        return new Vector3(resx,resy,resT);
    }
    public float alpha;
    public float Xcoeff;
    public float Ycoeff;
    public void FCalc(float vAng, float steering, float Fz, int n)
    {
        Quaternion toLocal;
        Quaternion toWheel;
        //Fz = 4000.0f;
        
        // velocity in woorld coordinate
        Vector3 vWheelInWorld = body.rigidbody.GetPointVelocity(model.transform.position);
        float wr = vAng * radius;
        
        if (!steeringWheel)
        {
            steering = 0;
        }
        
        toLocal = Quaternion.Euler(0, -steering, 0);
        toWheel = Quaternion.Euler(0, steering, 0);
        
        // velocity in wheel coordinate
        Vector3 vWheelLocal = toLocal * transform.InverseTransformDirection(vWheelInWorld);

        // Transform into equations coord system and into ground velocity
        Vector2 vWheel = new Vector2(vWheelLocal.z, vWheelLocal.x);
        if (debugWheelsVelo) Debug.DrawRay(model.transform.position + transform.up * 3, vWheelInWorld, Color.green);

        alpha = Vector2.Angle(new Vector2(vWheelInWorld.z, vWheelInWorld.x),
            new Vector2(transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, 1.0f)).z, 
                transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, 1.0f)).x));
        Vector3 cross = Vector3.Cross(vWheelInWorld, transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, 1.0f)));
        if (cross.z > 0) alpha -= 360;
        
        if (steeringWheel)
        {
            wr = vWheel.x;
            angularVelocity = wr / radius;
            vAng = wr / radius;
        }
        this.wr = wr;
        this.vWheel = vWheel;
        updateDeflection(vWheel, vAng, Time.fixedDeltaTime, n);
        Vector3 resIntegral = integral(vWheel, vAng, alpha, Fz, n);
        Fx = W * resIntegral.x;
        Fy = W * resIntegral.y;
        //frictionTorque = W * resIntegral.z;
        //Torque = transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, torque));
        Vector3 FLocal = new Vector3(Ycoeff*Fy, 0.0f,Xcoeff*Fx);
        if (steeringWheel)
        {
            //FLocal.z=0.0f;
        }
        F = transform.TransformDirection(toWheel * FLocal);
        if (debugWheelsXAxis) Debug.DrawRay(model.transform.position + up * 3, transform.TransformDirection(toWheel * new Vector3(0.0f,0.0f,1.0f)));
    }
}
