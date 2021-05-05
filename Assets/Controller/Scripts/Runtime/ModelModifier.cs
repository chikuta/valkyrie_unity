using UnityEngine;
using System.Linq;
using System.Collections.Generic;

namespace Controller
{
    public class ModelModifier : MonoBehaviour
    {
        void Start()
        {
            // get articulation chain
            var chain = this.GetComponentsInChildren<ArticulationBody>();
            foreach (var body in chain)
            {
                if (body.mass < 1.0e-6)
                {
                    body.mass = 1.0e-5f;
                }

                Debug.LogFormat("{0} {1}", body.name, body.inertiaTensor);

                if (body.inertiaTensor.x <= 1.0e-6f && body.inertiaTensor.y <= 1.0e-6f && body.inertiaTensor.z <= 1.0e-6f)
                {
                    Debug.LogFormat("Change intertiaTensor {0}", body.name);
                    body.inertiaTensor = new Vector3(1.0e-5f, 1.0e-5f, 1.0e-5f);
                }
            }
        }
    };
}
