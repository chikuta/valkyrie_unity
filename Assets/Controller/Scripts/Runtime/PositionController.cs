using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System;
using System.Linq;
using System.Collections.Generic;
using RosSharp.Urdf;
using RosMessageTypes;
using SensorMsgs = RosMessageTypes.Sensor;

namespace Controller
{

class PositionController : MonoBehaviour
{
    [SerializeField] string JointStateTopicName = "/unity/joint_states";
    [SerializeField] string JointStateFrameId = "base_link";
    [SerializeField] string JointCommandTopicName = "/unity/joint_command";
    [SerializeField] ROSConnection rosConnection = null;
    [SerializeField] float stiffness = 0;
    [SerializeField] float damping = 0;
    [SerializeField] float speed = 5.0f;    // Unit deg/s
    [SerializeField] float torque = 100.0f; // Unity Nm or N
    [SerializeField] float acceleration = 0;
    [SerializeField] float forceLimit = 0;
    private List<ArticulationBody> articulationChain = null;
    private Dictionary<string, ArticulationBody> jointDict = null;
    private SensorMsgs.MJointState jointCommand = null;
    private SensorMsgs.MJointState actualStates = null;
    private int jointCount = 0;
    private float minJointMass = 0.05f;

    void Start()
    {
        articulationChain = new List<ArticulationBody>();
        this.GetComponentsInChildren<ArticulationBody>(articulationChain);

        // create joint states for joint_state publisher
        actualStates = new SensorMsgs.MJointState();
        jointCount = articulationChain.Count(joint => joint.jointType != ArticulationJointType.FixedJoint);
        actualStates.header.frame_id = JointStateFrameId;
        actualStates.name = new string[jointCount];
        actualStates.position = new double[jointCount];
        actualStates.velocity = new double[jointCount];
        actualStates.effort = new double[jointCount];

        // create ros_joint_name - articulation_joint dictionary
        jointDict = new Dictionary<string, ArticulationBody>();
        foreach (var joint in articulationChain)
        {
            if (joint.jointType != ArticulationJointType.FixedJoint)
            {
                var urdf_joint = joint.gameObject.GetComponent<UrdfJoint>();
                if (urdf_joint)
                {
                    // create dictionary
                    Debug.LogFormat("Register ros joint [{0}] to dictionary.", urdf_joint.jointName);
                    jointDict.Add(urdf_joint.jointName, joint);
                }
            }
        }

        // set initial value to each joint drives
        foreach (var joint in articulationChain)
        {
            // set initial value to drive
            Debug.LogFormat("Init drive [{0}].", joint.name);
            int defDynamicVal = 10;
            joint.jointFriction = defDynamicVal;
            joint.angularDamping = defDynamicVal;
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.target = 0;
            joint.xDrive = drive;
            joint.mass = Math.Max(minJointMass, joint.mass);
        }

        // create ros connector
        ROSConnection.instance.Subscribe<SensorMsgs.MJointState>(JointCommandTopicName, ReceiveJointCommand);
    }

    void Update()
    {
        if (rosConnection != null)
        {
            PublishJointState();
        }
    }

    void FixedUpdate()
    {
        if (jointCommand != null)
        {
            SetJointCommand();
        }
    }

    void PublishJointState()
    {
        int idx = 0;
        foreach (var elem in jointDict)
        {
            if (elem.Value.jointPosition.dofCount > 0)
            {
                actualStates.name[idx] = elem.Key;
                actualStates.position[idx] = elem.Value.jointPosition[0];
                actualStates.velocity[idx] = elem.Value.jointVelocity[0];
                actualStates.effort[idx] = elem.Value.jointForce[0];
            }
            else
            {
                actualStates.name[idx] = elem.Key;
                actualStates.position[idx] = 0.0;
                actualStates.velocity[idx] = 0.0;
                actualStates.effort[idx] = 0.0;
            }
            ++idx;
        }

        actualStates.header.Update();
        rosConnection.Send(JointStateTopicName, actualStates);
    }

    void SetJointCommand()
    {
        // create joint_name - position map
        for (int idx = 0; idx < jointCommand.name.Count(); ++idx)
        {
            string name = jointCommand.name[idx];
            if (jointDict.ContainsKey(name))
            {
                ArticulationDrive drive = jointDict[name].xDrive;
                float target_pos = (float)(jointCommand.position[idx] * 180.0f / Math.PI);
                float direction = (target_pos - drive.target) > 0.0f ? 1.0f : -1.0f;
                float delta_pos = direction * Time.fixedDeltaTime * speed;
                float next_target_pos = drive.target + delta_pos;
                drive.target = next_target_pos;

                if (InRange(drive.target, next_target_pos, target_pos))
                {
                    drive.target = target_pos;
                }
                else if (InRange(drive.upperLimit, drive.lowerLimit, next_target_pos))
                {
                    drive.target = next_target_pos;
                }
                else
                {
                    Debug.LogFormat("Error {0}", name);
                }

                jointDict[name].xDrive = drive;
            }
            else
            {
                Debug.LogFormat("joint [{0}] is not found.", name);
            }
        }
    }

    void ReceiveJointCommand(SensorMsgs.MJointState msg)
    {
        jointCommand = msg;
    }

    private bool InRange(float a, float b, float val)
    {
        float max = Math.Max(a, b);
        float min = Math.Min(a, b);
        return (max >= val && min <= val);
    }
};

}