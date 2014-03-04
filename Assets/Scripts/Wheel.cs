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
	Vector3 up, right, forward;
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
    public bool drivingWheel;
    public float Fx;
    public float Fy;

    public Vector3 FBase;
    public Vector3 F = Vector3.zero;

    // New variables

    defl_struct[] deflections;
    public float L = 0.2f;
    public float W = 0.1f;
    public int n = 100;
    float wr;
    Vector2 vWheel;
    public float teta1x = 0.0f;
    public float teta2x = 0.0f;
    public float teta1y = 0.0f;
    public float teta2y = 0.0f;
    public float vrxDebug;
    public float vryDebug;
    public float muslx = 0.8f; // 0.8 for dry asphalt, 0.6 for wet asphalt, 0.2 for snow, 0.1 for ice?
    public float musly = 0.8f; // 0.8 for dry asphalt, 0.6 for wet asphalt, 0.2 for snow, 0.1 for ice?
    public float mugvr = 1.0f;
    float alpha;
    public float Xcoeff;
    public float Ycoeff;

    // Debug variables
    public bool debugSuspensions = false;
    public bool debugRoadForce = false;
    public bool debugWheelsXAxis = false;
    public bool debugWheelsVelo = false;
    public bool debugCollisions = false;

    public float FxDebug;

	void Start () {
		Transform trs = transform;
		while (trs != null && trs.rigidbody == null)
			trs = trs.parent;
		if (trs != null)
			body = trs.rigidbody;

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
	
	
	void RoadForce () {
		int slipRes=(int)((100.0f-Mathf.Abs(angularVelocity))/(10.0f));
		if (slipRes < 1)
			slipRes = 1;
		float invSlipRes = (1.0f/(float)slipRes);
		
		float totalInertia = inertia + drivetrainInertia;
		float driveAngularDelta = driveTorque * Time.deltaTime * invSlipRes / totalInertia;
		float totalFrictionTorque = brakeFrictionTorque * brake + handbrakeFrictionTorque * handbrake + frictionTorque + driveFrictionTorque;
		float frictionAngularDelta = totalFrictionTorque * Time.deltaTime * invSlipRes / totalInertia;

        for (int i=0; i<slipRes; i++)
		{
			angularVelocity += driveAngularDelta;
			if (Mathf.Abs(angularVelocity) > frictionAngularDelta)
				angularVelocity -= frictionAngularDelta * Mathf.Sign(angularVelocity);
			else
				angularVelocity = 0;
		}
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
        float rInc = 220.0f / (float)rPrecision;
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
            for (float a = 0.0f; a < 220.0f; a += rInc)
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

    void checkOnGround()
    {
        Vector3 up = transform.up;
        Vector3 centerW = model.transform.position;// transform.position + ((-up) * suspensionTravel);
        onGround = false;
        float dist2 = radius;
        
        hit = new RaycastHit();
        rayElem[] tab = listRays(centerW, radius, width, 10, 4);

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
                }
            }
        }
    }

    void FixedUpdate()
    {
        Vector3 pos = transform.position;
        up = transform.up;

        checkOnGround();

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
            RoadForce();

            // TESTS
            FCalc(angularVelocity, steering * maxSteeringAngle, normalForce, n);
            //Debug.Log("F = " + F + " Suspension : " + suspensionForce.y);
            body.AddForceAtPosition(new Vector3(0.0f, suspensionForce.y, 0.0f) + F, pos);
            //body.AddRelativeTorque(Torque);
            if (debugRoadForce) Debug.DrawLine(model.transform.position, model.transform.position + F, Color.blue);
            if (debugSuspensions) Debug.DrawLine(model.transform.position, model.transform.position + (suspensionForce / 5000), Color.blue);
            //body.AddForceAtPosition (suspensionForce + roadForce, pos);
		}
		else
		{
			//Logger("not on ground");
			deflectionsInit(n);
			compression = 0.0f;
			suspensionForce = Vector3.zero;
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
    }
	
	void Logger(string log)
	{
        if (debug) Debug.Log(this.name + ": " + log + "\n");
	}

    // OTHER MODEL : LUMPED


    struct defl_struct
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

    // Creating deflections arrays for n bristles
    // At the start no bristle is deflected
    // 2D array : bristle deflection on x (deflections[i][0]) and y (deflections[i][1])
    void deflectionsInit(int n)
    {
        deflections = new defl_struct[n];
        for (int i=0; i < n; i++)
        {
            deflections[i] = new defl_struct(0.0f, 0.0f);
        }
    }
    void updateDeflection(Vector2 vWheel, float vAng, float delta, int n)
    {
        float wr = (-radius * vAng);
        int nbr_bristles = Mathf.Abs((int)(wr * delta * n / L)) + 1;
        float subDelta = (delta/n) + (delta / n) * (int)(nbr_bristles/n);

//        float subDelta = delta / n;

        for (int i = 1; i < n; i++)
        {
            deflections[i].x = ((radius * vAng) - (vWheel.x)) * subDelta*i;
            deflections[i].y = (vWheel.y) * subDelta*i;
        }
    }

    // normalized normal pressure distribution
    float nnpd()
    {
        return 1.0f; ///(float)n;
    }

    float zx(int i)
    {
        if (i < 0) return 0.0f;
        return deflections[i].x;
    }

    float zy(int i)
    {
        if (i < 0) return 0.0f;
        return deflections[i].y;
    }

    // Phi for a bristle i
    Vector2 Phi(int i, Vector2 vWheel, float vAng, float alpha, float Fz, int n)
    {
        float teta0x = muslx * (3.16f * 100000.0f * Fz);
        float teta0y = musly * ((-0.18f * Fz * Fz) + (1.88f * Fz) + 1.84f) * 100000.0f;
        
        float muS = -0.054f * Fz + 1.887f; // Given by tables
        float muC = -0.022f * Fz + 0.734f; // Given by tables
        float Vs = -3.5f; // Given by tables        
        float d = 0.6f; // Given by tables

        float vrx = (radius * vAng) - (vWheel.x);
        vrxDebug = vrx;
        float vry = vWheel.y;
        vryDebug = vry;
        float vrAbs = Mathf.Sqrt(vrx * vrx + vry * vry);

        float gy = 0.007f*Fz*Fz - 0.028f*Fz + 0.98f;
        float beta = 0.0f;
        if (vrx != 0 ) beta = Mathf.Abs(vry/vrx);

        float gvr = (1.0f + ((gy - 1.0f) / (Mathf.PI * 0.5f)) * beta) * (muC + ((muS - muC) * Mathf.Exp(-Mathf.Pow(Mathf.Abs(vrAbs / Vs), d))));
        //Debug.Log("gvr = " + gvr + " gy = " + gy + " beta = " + beta);
        float dzx = vrx - ((teta0x * Mathf.Abs(vrAbs) / gvr) * zx(i)) - (radius * Mathf.Abs(vAng) * ((n - 1) / L) * (zx(i) - zx(i - 1)));
        //Debug.Log("dzx = " + dzx + " gvr = " + gvr);
        float dzy = vry - ((teta0y * Mathf.Abs(vrAbs) / gvr) * zy(i)) - (radius * Mathf.Abs(vAng) * ((n - 1) / L) * (zy(i) - zy(i - 1)));

        float part1 = nnpd() / Fz;
        // phiX calculus
        float part2x = (teta0x * zx(i)) + (teta1x * dzx) + (teta2x * vrx);
        //Debug.Log("Part2x = " + part2x + " teta0x = " + teta0x + " zx(i) = " + zx(i) + " teta1x = " + teta1x + " dzx = " + dzx + " teta2x = " + teta2x + " vrx = " + vrx );
        // phiY calculus
        float part2y = (teta0y * zy(i)) + (teta1y * dzy) + (teta2y * vry);
        
        //Debug.Log("vrx = " + vrx + " vrabs = " + vrAbs + " zx(i) = " +  zx(i) + " radius = " + radius + " vAngAbs = " + vAng + " L = " + L  
        //   + "\nphi = " + phix + " part1 = " + part1 + " part2x = " + part2x + " part2y = " + part2y + " dz = " + dzx + " gvr = " + gvr );
        //Debug.Log("phiY = " + phiy);
        return new Vector2(part1 * part2x, part1 * part2y);
    }

    Vector3 integral(Vector2 vWheel, float vAng, float alpha, float Fz, int n)
    {
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

        return coef * new Vector3(sumx, sumy, sumT);
    }

    void FCalc(float vAng, float steering, float Fz, int n)
    {
        // velocity in woorld coordinate
        Vector3 vWheelInWorld = body.rigidbody.GetPointVelocity(model.transform.position);
        float wr = vAng * radius;

        Quaternion toLocal = Quaternion.Euler(0, -steering, 0);
        Quaternion toWheel = Quaternion.Euler(0, steering, 0);
        
        // velocity in wheel coordinate
        Vector3 vWheelLocal = toLocal * transform.InverseTransformDirection(vWheelInWorld);

        // Transform into equations coord system and into ground velocity
        Vector2 vWheel = new Vector2(vWheelLocal.z, vWheelLocal.x);
        if (debugWheelsVelo) Debug.DrawRay(model.transform.position + transform.up * 3, vWheelInWorld, Color.green);

        // Keep the angle between [0;360]
        alpha = Vector2.Angle(new Vector2(vWheelInWorld.z, vWheelInWorld.x),
            new Vector2(transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, 1.0f)).z, 
                transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, 1.0f)).x));
        Vector3 cross = Vector3.Cross(vWheelInWorld, transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, 1.0f)));
        if (cross.z > 0) alpha -= 360;
        
        if (!drivingWheel)
        {
            wr = vWheel.x;
            angularVelocity = wr / radius;
            vAng = wr / radius;
        }

        updateDeflection(vWheel, vAng, Time.fixedDeltaTime, n);
        Vector3 resIntegral = integral(vWheel, vAng, alpha, Fz, n);
        Fx = W * resIntegral.x;
        Fy = W * resIntegral.y;
        driveTorque = W * resIntegral.z;
        if (Fx < 0) Fx = 100 * Fx;
        Vector3 FLocal = new Vector3(Ycoeff*Fy, 0.0f, Xcoeff*Fx);
        FxDebug = Fx;
        F = transform.TransformDirection(toWheel * FLocal);
        if (debugWheelsXAxis) Debug.DrawRay(model.transform.position + up * 3, transform.TransformDirection(toWheel * new Vector3(0.0f,0.0f,1.0f)));
    }
}
