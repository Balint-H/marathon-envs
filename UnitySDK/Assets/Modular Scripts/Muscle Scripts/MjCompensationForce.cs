using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;
using MotorUpdate;

namespace Mujoco
{
    public class MjCompensationForce : Muscles
    {
        [SerializeField]
        protected Transform actuatorRoot;

        [SerializeField]
        protected List<MjActuator> actuatorSubset;


        protected IReadOnlyList<Tuple<MjActuator, MjHingeJoint>> activeActRefPairs;
        protected IReadOnlyList<Tuple<MjActuator, MjHingeJoint>> passiveActRefPairs;

        private IReadOnlyList<MjActuator> actuators;
        public virtual IReadOnlyList<MjActuator> Actuators { get => actuatorRoot.GetComponentsInChildren<MjActuator>().ToList(); }

        [SerializeField]
        MotorUpdateRule updateRule;

        [SerializeField]
        Transform kinematicRef;

        [SerializeField, Range(0.0f, 1.0f)]
        float compensation;

        [SerializeField]
        bool trackPosition;
        [SerializeField]
        bool trackVelocity;

        [SerializeField]
        bool updateAlone;

        [SerializeField, Min(0f)]
        float maxForce;

        private bool IsSubsetDefined { get => (actuatorSubset != null && actuatorSubset.Count > 0); }

        public override int ActionSpaceSize => IsSubsetDefined && kinematicRef && trackPosition ? actuatorSubset.Count : actuatorRoot.GetComponentsInChildren<MjActuator>().ToList().Count;

        float[] nextActions;

        unsafe private void UpdateTorque(object sender, MjStepArgs e)
        {
            foreach ((var action, (var actuator, var reference)) in nextActions.Zip(activeActRefPairs, Tuple.Create))
            {

                var curState = new float[] { (float)e.data->qpos[actuator.Joint.QposAddress],
                                             (float)e.data->qvel[actuator.Joint.DofAddress],
                                             (float)e.data->qacc[actuator.Joint.DofAddress]};
                var targetState = trackPosition ? new float[] { (float)e.data->qpos[reference.QposAddress]+action,
                                                                    trackVelocity? (float)e.data->qvel[reference.DofAddress] : 0f} : new float[] { action, 0f };
                float torque = updateRule.GetTorque(curState, targetState);
                torque *= actuator.CommonParams.Gear[0];
                torque += compensation * (float)e.data->qfrc_bias[actuator.Joint.DofAddress];
                e.data->qfrc_applied[actuator.Joint.DofAddress] = Mathf.Clamp(torque, -maxForce, maxForce);
            }

            foreach ((var actuator, var reference) in passiveActRefPairs)
            {

                var curState = new float[] { (float)e.data->qpos[actuator.Joint.QposAddress],
                                             (float)e.data->qvel[actuator.Joint.DofAddress],
                                             (float)e.data->qacc[actuator.Joint.DofAddress]};
                var targetState = new float[] { (float)e.data->qpos[reference.QposAddress],
                                                 trackVelocity? (float)e.data->qvel[reference.DofAddress] : 0f};
                float torque = updateRule.GetTorque(curState, targetState);
                
                torque *= actuator.CommonParams.Gear[0];
                torque += compensation * (float)e.data->qfrc_bias[actuator.Joint.DofAddress];
                e.data->qfrc_applied[actuator.Joint.DofAddress] = Mathf.Clamp(torque, -maxForce, maxForce);
            }
        }

        unsafe public override void ApplyActions(float[] actions, float actionTimeDelta)
        {
            nextActions = actions;

        }

        public override float[] GetActionsFromState()
        {
            if (trackPosition) return Enumerable.Repeat(0f, ActionSpaceSize).ToArray();
            if (kinematicRef) return activeActRefPairs.Select(a => Mathf.Deg2Rad * a.Item2.Configuration).ToArray();
            return activeActRefPairs.Select(a => a.Item1.Control).ToArray();
        }

        public override void OnAgentInitialize(DReConAgent agent)
        {
            MjScene.Instance.ctrlCallback += UpdateTorque;
            actuators = Actuators;
            nextActions = Enumerable.Repeat(0f, ActionSpaceSize).ToArray();
            if (IsSubsetDefined && kinematicRef && trackPosition)
            {
                var passiveActs = actuators.Where(a => !actuatorSubset.Contains(a));
                activeActRefPairs = actuatorSubset.Select(a => Tuple.Create(a, FindReference(a))).ToList();
                passiveActRefPairs = passiveActs.Select(a => Tuple.Create(a, FindReference(a))).ToList();
                return;
            }

            activeActRefPairs = actuators.Select(a => Tuple.Create(a, FindReference(a))).ToList();
            passiveActRefPairs = new List<Tuple<MjActuator, MjHingeJoint>>();
            if(agent)agent.onBeginHandler += resetActions;
        }

        private void resetActions(object sender, AgentEventArgs e)
        {
            nextActions = GetActionsFromState();
        }

        private void OnDisable()
        {

            if (MjScene.InstanceExists) MjScene.Instance.ctrlCallback -= UpdateTorque;
        }

        private MjHingeJoint FindReference(MjActuator act)
        {
            return kinematicRef ? kinematicRef.GetComponentsInChildren<MjHingeJoint>().First(rj => rj.name.Contains(act.Joint.name)) : null;
        }

        private void Awake()
        {
            if (updateAlone)
            {
                OnAgentInitialize(null);
            }
        }

    }
}
