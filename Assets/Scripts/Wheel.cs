using UnityEngine;
using System.Collections;

// This class simulates a single car's wheel with tire, brake and simple
// suspension (basically just a single, independant spring and damper).
public class Wheel : MonoBehaviour {        
	// Wheel Specifications

    // Wheel radius in meters
    public float radius = 0.34f;
    // Wheel width in meters
    public float width = 0.2f;
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
    // If this wheel connected to the engine
    public bool drivingWheel;

	// Inputs
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

    // Array of bristles deflections
    defl_struct[] deflections;
    public float LengthContactPatch = 0.2f;
    public float WidthContactPatch = 0.1f;
    // The number of bristles used in the model
    public int n = 100;
    // Parameters for Tire model
    public float teta1x = 0.0f;
    public float teta2x = 0.0f;
    public float teta1y = 0.0f;
    public float teta2y = 0.0f;
    public float muslx = 1.0f; // 1.0 for dry asphalt, 0.6 for wet asphalt, 0.2 for snow, 0.1 for ice?
    public float musly = 1.0f; // 1.0 for dry asphalt, 0.6 for wet asphalt, 0.2 for snow, 0.1 for ice?
    public float mugvr = 1.0f;
    public float Xcoeff;
    public float Ycoeff;

    // Debug variables
    // Shows the result of every collisions
    public bool debugCollisions = false;
    // Shows the suspension force
    public bool debugSuspensions = false;
    // Shows the road force
    public bool debugRoadForce = false;
    // Shows the wheel x axis
    public bool debugWheelsXAxis = false;
    // Shows the wheel velo
    public bool debugWheelsVelo = false;
    // Shows all the rays tested for collisions
    public bool debugRay = false;
	
	// Output
	public float angularVelocity;
    public float compression;
    // If in contact with ground
    public bool onGround;
    // Forces from the road on the tire
    public Vector3 RoadForce = Vector3.zero;
	
	
	// cached values
	Rigidbody body;
	float maxAngle;
    float oldAngle;
    float fullCompressionSpringForce;
    Vector3 wheelVelo;
    Vector3 localVelo;
    Vector3 groundNormal;
    float rotation;
    float normalForce;
    Vector3 suspensionForce;
    Vector3 up, right, forward;
    Quaternion inverseLocalRotation = Quaternion.identity;
    RaycastHit hit;

    // This structure represents a ray.
    // It is used for the detection collisions system
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
	
    // Calculates the suspension force depending on the current compression and the full compression calculated at the init
	Vector3 SuspensionForce () {
		float springForce = compression * fullCompressionSpringForce;
		normalForce = springForce;
		float damperForce = Vector3.Dot(localVelo, groundNormal) * damping;
		return (springForce - damperForce + suspensionForceInput) * up;
	}
	
	// Calculate the new wheel velocity based on the drivetrain state
	void UpdateWheelVelocity () {
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
	
    // Collision related

    // Check if a hitpoint is inside the cylinder defined by center, radius and width
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

    // Creates an array of rays centered at center and equally distributed between [-width/2;width/2] and [-110°;110°] depending of wPrecision and rPrecision
    rayElem[] listRays(Vector3 center, float radius, float width, int rPrecision, int wPrecision)
    {
        int numberRays = rPrecision * wPrecision;
        int cpt = 0;
        float widthInc = width / wPrecision;
        float rInc = 220.0f / (float)rPrecision;
        bool showRays = false;
        rayElem[] tabRays = new rayElem[numberRays];
        Quaternion rotate = Quaternion.Euler(0, maxSteeringAngle * steering, 0);

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
                if (showRays)
                {
                    Debug.DrawRay(tabRays[cpt].c(), tabRays[cpt].d(), Color.blue, 10.0f);
                }
                cpt++;
            }
        }
        return tabRays;
    }

    // Check if the wheel is touching the ground (ie. an object that is not the car) 
    void checkOnGround()
    {
        Vector3 centerW = model.transform.position;
        onGround = false;
        float distance = radius;
        
        hit = new RaycastHit();
        rayElem[] tab = listRays(centerW, radius, width, 10, 4);

        foreach (rayElem r in tab)
        {
            RaycastHit[] hts = Physics.RaycastAll(r.c(), r.d(), r.l());
            foreach (RaycastHit h in hts)
            {
                if (!h.collider.isTrigger && h.collider.name != "Collider" && h.distance < distance)
                {
                    hit = h;
                    onGround = true;
                    distance = h.distance;
                }
            }
        }
    }
    

    // Tire model related 

    struct defl_struct
    {
        // Deflection on x
        public float x;
        // Deflection on y
        public float y;
        // 
        // Update current deflection of current bristle
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

    // Goal :
    //  Calculates the deflections that happened between now - step and now. 
    // In :
    //  Vector2 vWheel : Wheel horizontal velocity
    //  float vAng : Wheel angular velocity
    //  float delta : time step used
    //  int n : number of bristles used
    void updateDeflection(Vector2 vWheel, float vAng, float delta, int n)
    {
        float wr = (-radius * vAng);
        int nbr_bristles = Mathf.Abs((int)(wr * delta * n / LengthContactPatch)) + 1;
        float subDelta = (delta/n) + (delta / n) * (int)(nbr_bristles/n);

        for (int i = 1; i < n; i++)
        {
            deflections[i].x = ((radius * vAng) - (vWheel.x)) * subDelta*i;
            deflections[i].y = (vWheel.y) * subDelta*i;
        }
    }

    // Return the pressusre at a point of the tire
    // Since normalized normal pressure distribution is used, it's a constant.
    // But it could be modified to use a trapezoidal distribution for example.
    float nnpd()
    {
        return 1.0f;
    }

    // Returns deflection on longitudinal axis for the bristle i
    float zx(int i)
    {
        // To simplify discrete calculus of the road force
        // Every bristle before 0 is not in contact with the ground and so there is no deflection
        if (i < 0) return 0.0f;
        return deflections[i].x;
    }

    // Returns deflection on lateral axis for the bristle i
    float zy(int i)
    {
        // To simplify discrete calculus of the road force
        // Every bristle before 0 is not in contact with the ground and so there is no deflection
        if (i < 0) return 0.0f;
        return deflections[i].y;
    }

    // Goal :
    //  Calculates the value of Ph for the bristle i. 
    // In :
    //  int i : index of current bristle
    //  Vector2 vWheel : Wheel horizontal velocity
    //  float vAng : Wheel angular velocity
    //  float alpha : current steering angle for this wheel
    //  float Fz : vertical force applied to this wheel
    //  int n : number of bristles used
    Vector2 Phi(int i, Vector2 vWheel, float vAng, float alpha, float Fz, int n)
    {
        float teta0x = muslx * (3.16f * 100000.0f * Fz); // Given by tables
        float teta0y = musly * ((-0.18f * Fz * Fz) + (1.88f * Fz) + 1.84f) * 100000.0f; // Given by tables
        
        float muS = -0.054f * Fz + 1.887f; // Given by tables
        float muC = -0.022f * Fz + 0.734f; // Given by tables
        float Vs = -3.5f; // Given by tables        
        float d = 0.6f; // Given by tables

        float vrx = (radius * vAng) - (vWheel.x);
        float vry = vWheel.y;
        float vrAbs = Mathf.Sqrt(vrx * vrx + vry * vry);

        float gy = 0.007f * Fz * Fz - 0.028f * Fz + 0.98f;  // Given by tables
        float beta = 0.0f;
        if (vrx != 0 ) beta = Mathf.Abs(vry/vrx);

        float gvr = (1.0f + ((gy - 1.0f) / (Mathf.PI * 0.5f)) * beta) * (muC + ((muS - muC) * Mathf.Exp(-Mathf.Pow(Mathf.Abs(vrAbs / Vs), d))));
        float dzx = vrx - ((teta0x * Mathf.Abs(vrAbs) / gvr) * zx(i)) - (radius * Mathf.Abs(vAng) * ((n - 1) / LengthContactPatch) * (zx(i) - zx(i - 1)));
        float dzy = vry - ((teta0y * Mathf.Abs(vrAbs) / gvr) * zy(i)) - (radius * Mathf.Abs(vAng) * ((n - 1) / LengthContactPatch) * (zy(i) - zy(i - 1)));

        float part1 = nnpd() / Fz;
        // phiX calculus
        float part2x = (teta0x * zx(i)) + (teta1x * dzx) + (teta2x * vrx);
        // phiY calculus
        float part2y = (teta0y * zy(i)) + (teta1y * dzy) + (teta2y * vry);
        
        return new Vector2(part1 * part2x, part1 * part2y);
    }

    // Calculates the integral using rectangle rule
    // Allows easy computation due to the bristles deflections array
    Vector3 integral(Vector2 vWheel, float vAng, float alpha, float Fz, int n)
    {
        float sumx = 0.0f;
        float sumy = 0.0f;
        float sumT = 0.0f;
        float coef = LengthContactPatch/(2*n);

        Vector2 phiInit = Phi(0, vWheel, vAng, alpha, Fz, n);
        sumx += phiInit.x;
        sumy += phiInit.y;
        sumT += phiInit.y*((LengthContactPatch/2)-zy(0)) ;
        for (int i = 1; i < n - 1; i++)
        {
            Vector2 phi = Phi(i, vWheel, vAng, alpha, Fz, n);
            sumx += (2 * phi.x);
            sumy += (2 * phi.y);
            sumT += (2 * phi.y*((LengthContactPatch / 2) - zy(i)));
        }
        Vector2 phiLast = Phi(n - 1, vWheel, vAng, alpha, Fz, n);
        sumx += phiLast.x;
        sumy += phiLast.y;
        sumT += phiLast.y * ((LengthContactPatch / 2) - zy(n-1));

        return coef * new Vector3(sumx, sumy, sumT);
    }

    // Calculates the force applied on the wheel by the road
    void FCalc(float vAng, float steering, float Fz, int n)
    {
        // velocity in world coordinate
        Vector3 vWheelInWorld = body.rigidbody.GetPointVelocity(model.transform.position);
        float wr = vAng * radius;

        Quaternion toLocal = Quaternion.Euler(0, -steering, 0);
        Quaternion toWheel = Quaternion.Euler(0, steering, 0);
        
        // velocity in wheel coordinate
        Vector3 vWheelLocal = toLocal * transform.InverseTransformDirection(vWheelInWorld);

        // Transform into equations coord system and into ground velocity
        Vector2 vWheel = new Vector2(vWheelLocal.z, vWheelLocal.x);
        if (debugWheelsVelo) Debug.DrawRay(model.transform.position + transform.up * 3, vWheelInWorld, Color.green);

        // Used to keep the angle between [0;360] since unity works on [-180;180]
        float alpha = Vector2.Angle(new Vector2(vWheelInWorld.z, vWheelInWorld.x),
            new Vector2(transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, 1.0f)).z, 
                transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, 1.0f)).x));
        Vector3 cross = Vector3.Cross(vWheelInWorld, transform.TransformDirection(toWheel * new Vector3(0.0f, 0.0f, 1.0f)));
        if (cross.z > 0) alpha -= 360;
        
        if (drivingWheel)
        {
            UpdateWheelVelocity();
        }
        else
        {
            // If not driving wheel then angular velocity is depending of the speed of the wheel only
            wr = vWheel.x;
            angularVelocity = wr / radius;
            vAng = wr / radius;
        }

        updateDeflection(vWheel, vAng, Time.fixedDeltaTime, n);
        Vector3 resIntegral = integral(vWheel, vAng, alpha, Fz, n);
        float Fx = WidthContactPatch * resIntegral.x;
        float Fy = WidthContactPatch * resIntegral.y;
        driveTorque = WidthContactPatch * resIntegral.z;
        if (Fx < 0) Fx = 10 * Fx;
        Vector3 FLocal = new Vector3(Ycoeff*Fy, 0.0f, Xcoeff*Fx);
        RoadForce = transform.TransformDirection(toWheel * FLocal);
        if (debugWheelsXAxis) Debug.DrawRay(model.transform.position + transform.up * 3, transform.TransformDirection(toWheel * new Vector3(0.0f,0.0f,1.0f)));
    }

    // Unity related functions

    void Start()
    {
        Transform trs = transform;

        while (trs != null && trs.rigidbody == null)
            trs = trs.parent;

        if (trs != null)
            body = trs.rigidbody;

        fullCompressionSpringForce = body.mass * massFraction * 2.0f * -Physics.gravity.y;

        // Initializing deflections ot the tire
        deflectionsInit(n);
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

    void FixedUpdate()
    {
        Vector3 pos = transform.position;

        checkOnGround();

        if (onGround)
        {
            // setting the custom frictions coefficent if the material has one
            // ohterwise it uses default one
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
            groundNormal = transform.InverseTransformDirection(inverseLocalRotation * hit.normal);
            compression = 1.0f - ((hit.distance + suspensionTravel - radius) / suspensionTravel);
            wheelVelo = body.GetPointVelocity(pos);
            localVelo = transform.InverseTransformDirection(inverseLocalRotation * wheelVelo);
            suspensionForce = SuspensionForce();
            FCalc(angularVelocity, steering * maxSteeringAngle, normalForce, n);
            body.AddForceAtPosition(suspensionForce + RoadForce, pos);

            // Debug
            if (debugCollisions) Debug.DrawRay(hit.point, groundNormal, Color.red, 10.0f);
            if (debugRoadForce) Debug.DrawLine(model.transform.position, model.transform.position + RoadForce, Color.blue);
            if (debugSuspensions) Debug.DrawLine(model.transform.position, model.transform.position + (suspensionForce / 2500), Color.blue);
        }
        else
        {
            // Reset the deflections and compression since wheel doesn't touch the ground
            // and update the wheel angular velocity 
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
        }

        compression = Mathf.Clamp01(compression);
        rotation += angularVelocity * Time.fixedDeltaTime;
        if (model != null)
        {
            model.transform.localPosition = Vector3.up * (compression - 1.0f) * suspensionTravel;
            model.transform.localRotation = Quaternion.Euler(Mathf.Rad2Deg * rotation, maxSteeringAngle * steering, 0);
        }

    }
}
