using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using System;

public class VDeviceMuscle : Muscles
{
    public override int ActionSpaceSize => 1;

    [SerializeField]
    MjActuator actuator;

    float[] nextActions;

    public override void ApplyActions(float[] actions, float actionTimeDelta)
    {
        nextActions = actions;
    }

    public override float[] GetActionsFromState()
    {
        return new float[] { actuator.Control };
    }

    public override void OnAgentInitialize(DReConAgent agent)
    {
        MjScene.Instance.ctrlCallback += UpdateTorque;
    }

    private unsafe void UpdateTorque(object sender, MjStepArgs e)
    {
        e.data->ctrl[actuator.MujocoId] = nextActions[0];
        actuator.Control = nextActions[0];
    }
}
