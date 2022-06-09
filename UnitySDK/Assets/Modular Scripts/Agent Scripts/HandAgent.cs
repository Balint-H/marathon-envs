using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Mujoco;
using Unity.MLAgents.Actuators;
using System.Linq;
using Unity.MLAgents.Sensors;

public class HandAgent : Agent
{
    [SerializeField]
    MjActuator thumb;

    [SerializeField]
    MjActuator wrist;

    [SerializeField]
    CameraSensor cam;
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        thumb.Control = actionBuffers.ContinuousActions.First();
        wrist.Control = actionBuffers.ContinuousActions.Last();
    }
}
