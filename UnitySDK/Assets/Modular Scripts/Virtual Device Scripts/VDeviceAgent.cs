using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Linq;
using Mujoco;

public class VDeviceAgent : Agent
{
    [SerializeField]
    VDeviceObservationSource observations;

    [SerializeField]
    Transform target;

    [SerializeField]
    Transform result;

    [SerializeField]
    VDeviceMuscle muscle;


    private void Start()
    {
        muscle.OnAgentInitialize(null);
    }

    override public void CollectObservations(VectorSensor sensor)
    {
        observations.FeedObservationsToSensor(sensor);
    }
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float[] vectorAction = actionBuffers.ContinuousActions.ToArray();

        muscle.ApplyActions(vectorAction, 0f);

        AddReward(Quaternion.Angle(target.rotation, result.rotation));
    }
}
