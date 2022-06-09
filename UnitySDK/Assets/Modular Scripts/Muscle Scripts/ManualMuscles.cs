using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MotorUpdate;
using Mujoco;
using System.Linq;
using System;

public class ManualMuscles : Muscles
{
    [SerializeField]
    private List<ActuatorPositionTarget> actuatorGameObjects;

    [SerializeField]
    MotorUpdateRule updateRule;

    [SerializeField]
    bool useActState;


    public override int ActionSpaceSize => actuatorGameObjects.Count;


    unsafe public override void ApplyActions(float[] actions, float actionTimeDelta)
    {
        foreach((var motor, var action) in actuatorGameObjects.Zip(actions, Tuple.Create))
        {
            motor.actuator.Control = action;
        }
    }

    public override float[] GetActionsFromState()
    {
        return actuatorGameObjects.Select(a => a.actuator.Control).ToArray();
    }

    private void Awake()
    {

        foreach (var a in actuatorGameObjects)
        {
            a.state = useActState?  new MjActuatorState(a.actuator) : new MjHingeJointState(a.actuator.Joint as MjHingeJoint);
        }

        MjScene.Instance.ctrlCallback += UpdateTorques;

    }

    private void Start()
    {

    }

    private unsafe void UpdateTorques(object Sender, MjStepArgs e)
    {
        //MujocoLib.mj_kinematics(MjScene.Instance.Model, MjScene.Instance.Data);
        float[] curActions = actuatorGameObjects.Select(a => updateRule.GetTorque(a.state, new StaticState((useActState? 1f : Mathf.Deg2Rad ) * a.target, 0f, 0f))).ToArray();
        foreach ((var motor, var action) in actuatorGameObjects.Zip(curActions, Tuple.Create))
        {
            e.data->ctrl[motor.actuator.MujocoId] = action;
            motor.actuator.Control = action;
        }
    }

    [Serializable]
    class ActuatorPositionTarget
    {
        [SerializeField]
        public MjActuator actuator;

        [SerializeField]
        public float target;

        public IState state;
    }

}
