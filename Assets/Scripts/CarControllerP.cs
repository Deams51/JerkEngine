/*
 *  To do :
 *      - Looking at Rolling resistance Calculation
 *      - Vehicule body side slip angle calculation
 * 
 * 
 */

using UnityEngine;
using System.Collections;

public class CarControllerP : MonoBehaviour {

    public WheelP fL;
    public WheelP fR;
    public WheelP rL;
    public WheelP rR;
    WheelP[] wheels;
    Vector3 windForce;
    Vector3 Fgravitation;
    Vector3 lastVelocity;
    public float g = 9.81f;
    public float massCoG;
    public float bR; // float bR => distance between 2 wheels on rear axle 
    public float bF; // float bF => distance between 2 wheels on front axle
    public float lF; // longitudinal distance between front wheel and CoG
    public float lR; // longitudinal distance between rear wheel and CoG
    public Vector3 totalForce;
    public float FR; // Rolling resistance force

    public float angularVelo = 0f; // Angular velocity of the wheel
    
    //debug
    public float aY;
    public float aX;
    public float dW;

    Transform CoG;   

	// Use this for initialization
	void Start () 
    {
        CoG = transform.Find("CoG");
        if (CoG == null) Debug.LogError("Couldn't find CoG in transform.Find(\"CoG\")");

        wheels = new WheelP[4];
        if (fL == null) Debug.LogError("fL not assigned to wheel.");
        wheels[0] = fL;
        fL.Init("FL", CoG);
        if (fR == null) Debug.LogError("fR not assigned to wheel.");
        wheels[1] = fR;
        fR.Init("FR", CoG);
        if (rL == null) Debug.LogError("rL not assigned to wheel.");
        wheels[2] = rL;
        rL.Init("RL", CoG);
        if (rR == null) Debug.LogError("rR not assigned to wheel.");
        wheels[3] = rR;
        rR.Init("RR", CoG);
         
        if (massCoG == 0.0f) massCoG = this.rigidbody.mass;

        // Calculating lF, lR, bF, bR
        Vector3 fLPoscog = CoG.InverseTransformPoint(fL.transform.position);
        Vector3 fRPoscog = CoG.InverseTransformPoint(fR.transform.position);
        Vector3 rLPoscog = CoG.InverseTransformPoint(rL.transform.position);
        Vector3 rRPoscog = CoG.InverseTransformPoint(rR.transform.position);

        lF = Mathf.Abs(fLPoscog.z);
        lR = Mathf.Abs(rLPoscog.z);

        bF = Mathf.Abs(fLPoscog.x) + Mathf.Abs(fRPoscog.x);
        bR = Mathf.Abs(rLPoscog.x) + Mathf.Abs(rRPoscog.x);    
	}
	
	// Update is called once per frame
	void Update () {
        if(Input.GetKeyDown(KeyCode.Z))
        {
            angularVelo += 1;
        }

	}

    void FixedUpdate()
    {
        // Getting data
        // For Suspensions
        // Wheel ground contact forces
        Vector3 acceleration = (rigidbody.velocity - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = rigidbody.velocity;
        // Vertical distance between CoG and ground
        // So in fact it's the y coordinate of the mcog pos in rear wheel coordinates + radius. 
        float hCoG = Mathf.Abs(rL.transform.InverseTransformPoint(CoG.transform.position).y) + rL.wheelRadius();

        // For calcRadii
        // Getting current steering angle
        var wheel = GameObject.Find("WheelFL");
        Wheel wh = (Wheel)wheel.GetComponent("Wheel");
        dW = wh.maxSteeringAngle * wh.steering;

        // For calcAlpha
        //float vPhi => Angular speed of the CoG x axis to real World x axis 
        //float beta => Angle between vCoG and xCoG
        Vector3 angVeloCoG = CoG.InverseTransformDirection(rigidbody.angularVelocity);
        Vector3 temp = angVeloCoG.normalized * rigidbody.angularVelocity.magnitude;
        float vPhi = temp.y;
        Vector3 temp2 = CoG.InverseTransformDirection(rigidbody.velocity);
        Vector3 temp2mod = new Vector3(temp2.x, 0.0f, temp2.z);
        float beta = 90 - Vector3.Angle(new Vector3(1.0f, 0.0f, 0.0f), temp2mod);
        // ??? Might only have to use 2 coordinates not the height
        float vCoG = this.rigidbody.velocity.magnitude;



        // Wheels update
        foreach (WheelP w in wheels)
        {
            // Calculating casters
            w.CalcCasters();
            w.Suspension(acceleration, massCoG, hCoG, bF, bR, lF, lR, -Physics.gravity.y);
            w.calcRadii(lR, lF, bR, bF, dW);
            w.calcAlpha(vPhi, beta, dW, vCoG);
            Vector3 test = w.v;
            Debug.DrawRay(w.transform.position, test);
            // For test purpose only
            //angularVelo = wh.angularVelocity;
            w.CalcSlip(angularVelo);
            // Friction force
            w.FrictionForces(dW, dW);
        }

    
        WindForce();
        RollingResistance();
        TotalForce();
        totalForce = new Vector3(totalForce.x, totalForce.z, totalForce.y);
        Debug.DrawRay(transform.position, totalForce, Color.red);
        totalForce = transform.TransformDirection(totalForce);
        //rigidbody.AddForce(totalForce);
        Debug.DrawRay(transform.position, totalForce, Color.blue);
    }


    // To do
    public void WindForce()
    {
        windForce = new Vector3();
        windForce.x = 0;
        windForce.y = 0;
        windForce.z = 0;
    }

    // Gravitation forces in the undercarriage coordinates system
    public void gravitation(float chi_road, float phi_road)
    {
        float cosChi = Mathf.Cos(chi_road);
        float sinChi = Mathf.Sin(chi_road);
        float cosPhi = Mathf.Cos(phi_road);
        float sinPhi = Mathf.Sin(phi_road);

        Matrix4x4 rot = new Matrix4x4();
        rot.m00 = cosChi;
        rot.m01 = sinChi * sinPhi;
        rot.m02 = sinChi * cosPhi;
        rot.m03 = 0;
        rot.m10 = 0;
        rot.m11 = cosPhi;
        rot.m12 = -sinPhi;
        rot.m13 = 0;
        rot.m20 = -sinChi;
        rot.m21 = cosChi * sinPhi;
        rot.m22 = cosChi * cosPhi;
        rot.m23 = 0;
        rot.m30 = 0;
        rot.m31 = 0;
        rot.m32 = 0;
        rot.m33 = 1;

        Fgravitation = rot * new Vector3(0,0,massCoG*Physics.gravity.y);
    }

    void RollingResistance()
    {
        FR = 0f;
    }

    // Total force applied to the chassis in the IN system
    void TotalForce()
    {
        float chiRoad = 0f, psi = 0f, phiRoad = 0f;
        //gravitation(chiRoad, phiRoad);
        Vector3 ForceInUndercarriage = new Vector3();
        //ForceInUndercarriage.x = fL.Fx + fR.Fx + rL.Fx + rR.Fx + windForce.x + Fgravitation.x + FR;
        ForceInUndercarriage.y = fL.Fy + fR.Fy + rL.Fy + rR.Fy + windForce.y + Fgravitation.y;
        ForceInUndercarriage.z = fL.FZC + fR.FZC + rL.FZC + rR.FZC + windForce.z + Fgravitation.z;
        
        //Debug.Log("fgravitation = " + Fgravitation + " forceinunder = " + ForceInUndercarriage);

        Matrix4x4 T_U_In = TRotZYX(psi, -chiRoad, -phiRoad);
        totalForce = T_U_In * ForceInUndercarriage;
    }

    Vector3 worldToCoG(Vector3 vW)
    {
        if (CoG == null) Debug.LogError("Couldn't find CoG in transform.Find(\"CoG\")");
        return CoG.position - vW;
    }

    Matrix4x4 TRotZYX(float psy, float chi, float phi)
    {
        Matrix4x4 res = new Matrix4x4();
        float cosPsy = Mathf.Cos(psy);
        float sinPsy = Mathf.Sin(psy);
        float cosChi = Mathf.Cos(chi);
        float sinChi = Mathf.Sin(chi);
        float cosPhi = Mathf.Cos(phi);
        float sinPhi = Mathf.Sin(phi);

        res.m00 = cosPsy*cosChi;
        res.m01 = (-sinPsy*cosPhi)-(cosPsy*sinChi*sinPhi);
        res.m02 = sinPsy*sinPhi + cosPsy*sinChi*cosPhi;
        res.m03 = 0;
        res.m10 = sinPsy*cosChi;
        res.m11 = cosPsy*cosPhi + sinPsy*sinChi*sinPhi;
        res.m12 = (-cosPsy*sinPhi)-(sinPsy*sinChi*cosPhi);
        res.m13 = 0;
        res.m20 = sinChi;
        res.m21 = cosChi*sinPhi;
        res.m22 = cosChi*cosPhi;
        res.m23 = 0;
        res.m30 = 0;
        res.m31 = 0;
        res.m32 = 0;
        res.m33 = 1;
        return res;
    }

    // chi_road and phi_road in case the road is not level
    Matrix4x4 T_Cog_In(float psy, float chi, float phi, float chi_road, float phi_road)
    {
        return TRotZYX(psy, chi - chi_road, phi - phi_road);
    }

    Matrix4x4 T_In_CoG(float psy, float chi, float phi, float chi_road, float phi_road)
    {
        return TRotZYX(psy, chi_road - chi, phi_road - phi);
    }

    Matrix4x4 T_U_In(float psy, float chi_road, float phi_road)
    {
        return TRotZYX(psy, -chi_road, -phi_road);
    }
}
