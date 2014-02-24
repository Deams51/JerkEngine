/*
 *  To Check :
 *  - Effect of the camber angle on the tire side slip angle
 * 
 * 
 */
using UnityEngine;
using System.Collections;
public class WheelP : MonoBehaviour {

    // Dynamic caster
    public float nS;
    // Caster
    public float nL;
    // Distance between wheel contact point and car's CoG, called radii
    public float r;
    // Angle between wheel contact point and CoG axis
    public float angleChassis;
    // wheel speed
    public Vector3 v;
    // Angle between xW and vW
    public float alpha;
    // Longitudinal slip
    public float sL;
    // Side slip
    public float sS;
    // Res slip
    public float sRes;
    // Friction coefficient
    public float muRes;
    // Force on Z axis
    public float Fz = 0f;
    // Friction force FL
    public float FL = 0f;
    // Friction force FLx
    public float Fx = 0f;
    // Friction force FLy
    public float Fy = 0f;
    // Friction force FS
    public float FS = 0f;
    public float FZC = 0f;
    public string position;
    public Transform CoG;
    // PARAMETERS

    // Suspension related parameters
    public static float kW;
    public static float kU;
    public static float dW;
    public static float dU;
    // Radius
    public float radius;
    // WIdth
    public float width;
    // Spring stiffness : stiffness of the tire used to calculate the real radius of the wheel
    // Between 2000 and 6000 N/cm for commercial tires. 
    public float kT = 40000f;
    // Attenuation factor kS mostly between 0.9-0.95
    public float kS = 0.9f;

    //debug 
    public float vcog;
    public float vphi;
    public float beta;
    public float dw;

    // Use this for initialization
	void Start () {
	}
	
	// Update is called once per frame
	void Update () {
	}


    public void Init(string pos, Transform cog)
    {
        position = pos;
        CoG = cog;
    }

    public float wheelRadius()
    {
        // Can also be determined using the distance betwen W and surface of contact inside the W.
        // wheelRadius = sqrt(r0^2 - l^2)
        kT = 6000;
        return radius - (Fz / kT);
    }

    /* Input :
     *  float a => Tire side slip angle (in rad?)
     *  float fz => the force acting vertically at the wheel ground contact point
     *  float fy => the lateral wheel force
     * Goal :
     *  Calculate the casters nL and nS used to calculate wheel ground contact point to CG vector
     */
    public void CalcCasters()
    {
        //Caster parameters
        float fz0 = 5000.0f; // in Newton
        float cPress = 230000.0f; // in N.m
        float l0 = -0.03f; // in m
        float l1 = 0.12f; //in m
        float toChange = 1.0f; // ??? supposed to be tan(alpha), but alpha is only computed later
        nL = 0.5f * (l0 + (l1 * (Fz / fz0)));
        nS = (3.0f * nL * toChange) + (Fy / cPress);
        //Debug.Log("ns = " + nS + " FY = " + Fy);
    }


    public void Suspension(Vector3 acceleration, float massCoG, float hCoG, float bF, float bR, float lF, float lR, float g)
    {
        Vector3 acc = CoG.InverseTransformDirection(acceleration);
        float aY = -acc.x; // Inverse of vehicle acceleration on Y == lateral
        float aX = -acc.z; // Inverse of vehicle acceleration on X == front 

        float l = lF + lR;
        float temp = 0.0f, temp2 = 0.0f;

        switch (position[0])
        {
            case 'F':
                temp = ((lR / l) * g) - ((hCoG / l) * aX);
                temp2 = (hCoG * aY) / (bF * g);
                break;

            case 'R':
                temp = ((lF / l) * g) + ((hCoG / l) * aX);
                temp2 = (hCoG * aY) / (bR * g);
                break;
        }

        switch (position[1])
        {
            case 'L':
                FZC = massCoG * temp * (0.5f - temp2);
                Fz = FZC;
                break;
            case 'R':
                FZC = massCoG * temp * (0.5f + temp2);
                Fz = FZC;
                break;
        }

        // Suspension spring forces

        // suspension damping forces
    }


    /* Goal : 
     *  Calculus of distance between each wheel contact point and CoG
     * Input :
     *  float lR => distance from CoG to rear wheel axle
     *  float lF => distance from CoG to front wheel axle
     *  float bR => distance between 2 wheels on rear axle 
     *  float bF => distance between 2 wheels on front axle
     *  float dW => steering angle
     */
    public void calcRadii(float lR, float lF, float bR, float bF, float dW)
    {
        float part1 = 0.0f;
        float part2 = 0.0f;
        float sin = Mathf.Sin(dW);
        float cos = Mathf.Cos(dW);

        //FL
        switch (position)
        {
            case "FL":
                part1 = lF - (nL * cos) + (nS * sin);
                part2 = (bF / 2) - (nS * cos) - (nL * sin);
                break;

            case "FR":
                part1 = lF - (nL * cos) + (nS * sin);
                part2 = (bF / 2) + (nS * cos) - (nL * sin);
                break;

            case "RL":
                part1 = lR + nL;
                part2 = (bR / 2) - nS;
                break;

            case "RR":
                part1 = lR + nL;
                part2 = (bR / 2) + nS;
                break;
        }
        r = Mathf.Sqrt(part1 * part1 + part2 * part2);
        angleChassis = Mathf.Atan(part2 / part1);
        //Debug.Log("r = " + r + " ar = " + ar);
    }


    /* Goal :
     *  Calculus of the angle alpha of each wheels
     * Input :
     *  float vPhi => Angular speed of the CoG x axis to real World x axis 
     *  float beta => Angle between vCoG and xCoG
     */
    public void calcAlpha(float vPhi, float beta, float dW, float vCoG)
    {
        // Longitudinal CoG coordinate direction
        Vector3 eX = transform.forward;
        // Lateral CoG direction
        // ??? Might have to use (-right)
        Vector3 eY = transform.right;
        float vCB = vCoG * beta;
        float part1 = 0.0f;
        float part2 = 0.0f;

        vcog = vCoG;
        vphi = vPhi;
        this.beta = beta;
        this.dw = dW;

        float epsilon = 0.01f;
        if (Mathf.Abs(vCoG) < epsilon)
        {
            alpha = 0.0f;
        }
        else
        {

            switch (position)
            {
                case "FL":
                    part1 = (vCoG - (vPhi * r * Mathf.Sin(angleChassis)));
                    part2 = (vCB + (vPhi * r * Mathf.Cos(angleChassis)));
                    break;
                case "FR":
                    part1 = (vCoG + (vPhi * r * Mathf.Cos(angleChassis)));
                    part2 = (vCB + (vPhi * r * Mathf.Sin(angleChassis)));
                    break;
                case "RL":
                    part1 = (vCoG - (vPhi * r * Mathf.Cos(angleChassis)));
                    part2 = (vCB - (vPhi * r * Mathf.Sin(angleChassis)));
                    break;
                case "RR":
                    part1 = (vCoG + (vPhi * r * Mathf.Sin(angleChassis)));
                    part2 = (vCB - (vPhi * r * Mathf.Cos(angleChassis)));
                    break;
            }
            v = part1 * eX + part2 * eY;
            alpha = dW - Mathf.Atan(part2 / part1);
        }

        //Debug.Log(position +" : Alpha = " + alpha + " vPhi = " + vPhi + " beta = " + beta + " dw = " + dW + " vCoG = " + vCoG + " vcb = " + vCB + " r = " + r + " ar = " + ar);
    }

    public void CalcSlip(float angularVelo)
    {
        float vr = angularVelo;
        float vw = v.magnitude;

        float temp = vr * Mathf.Cos(alpha);
        //Debug.Log("temp = " + temp + " vw = " + vw + " vr = " + vr);
        // case driving
        if (temp > vw)
        {
            sL = (temp - vw) / temp;
            sS = Mathf.Tan(alpha);
            //Debug.Log("driving : sL = " + sL + " sS = " + sS + " alpha = " + alpha + " temp = " + temp + " vw = " + vw);
        }
        else if (vr == 0)
        {
            sL = 0;
            sS = 1;
        }
        // case braking
        else
        {
            sL = (temp - vw) / vw;
            sS = (vr * Mathf.Sin(alpha)) / vw;
            //Debug.Log("braking : sL = " + sL + " sS = " + sS + " alpha = " + alpha + " vr = " + vr + " vw = " + vw);
        }
        sRes = Mathf.Sqrt((sL * sL) + (sS * sS));
    }

    // dwR and dWL are the angles dW for each side
    public void FrictionForces(float dWR, float dWL)
    {
        // Calc muRes
        // Parameters road dependent
        // Ex for asphalt
        float c1 = 1.2801f;
        float c2 = 23.99f;
        float c3 = 0.52f;
        //Simplified version
        muRes = ((c1 * (1 - Mathf.Exp(-c2 * sRes))) - c3 * sRes);
        // Check about reduction factor in case of extreme driving situations
        FL = ((muRes * (sL / sRes) * Mathf.Cos(alpha)) + (muRes * kS * (sS / sRes) * Mathf.Sin(alpha))) * Fz;
        FS = ((muRes * kS * (sS / sRes) * Mathf.Cos(alpha)) - (muRes * (sL / sRes) * Mathf.Sin(alpha))) * Fz;

        switch (position)
        {
            case "FL":
                Fx = FL * Mathf.Cos(dWL) - FS * Mathf.Sin(dWL);
                Fy = FS * Mathf.Cos(dWL) - FL * Mathf.Sin(dWL);
                break;

            case "FR":
                Fx = FL * Mathf.Cos(dWR) - FS * Mathf.Sin(dWR);
                Fy = FS * Mathf.Cos(dWR) - FL * Mathf.Sin(dWR);
                break;

            case "RL":
                Fx = FL;
                Fy = FS;
                break;

            case "RR":
                Fx = FL;
                Fy = FS;
                break;
        }
        //Debug.Log("Fx = " + Fx + " Fy = " + Fy);
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
        // quickfix
        float steering = 0f;
        bool debugRay = false;
        // end


        int numberRays = rPrecision * wPrecision;
        int cpt = 0;
        float widthInc = width / wPrecision;
        float rInc = 180.0f / (float)rPrecision;
        bool showRays = false;
        rayElem[] tabRays = new rayElem[numberRays];
        Quaternion rotate = Quaternion.Euler(0, steering, 0);
        //Quaternion rotate = this.model.transform.;

        //if (debugRay)
        {
            debugRay = false;
            showRays = true;
        }
        for (int w = 0; w < wPrecision; w++)
        {
            for (float a = 0.0f; a < 180.0f; a += rInc)
            {
                Vector3 translation = new Vector3((-width / 2) + w * widthInc, 0, 0);
                Vector3 direction = new Vector3(0, -radius * Mathf.Sin(a * Mathf.PI / 180), radius * Mathf.Cos(a * Mathf.PI / 180));

                tabRays[cpt] = new rayElem(transform.TransformPoint(translation), transform.TransformDirection(rotate * direction), radius);
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




    public bool checkOnGround()
    {
        Vector3 centerW = transform.position;
        bool onGround = false;
        float dist2 = radius;

        RaycastHit hit = new RaycastHit();
        rayElem[] tab = listRays(centerW, radius, width, 50, 10);

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
        return onGround;
    }
}
