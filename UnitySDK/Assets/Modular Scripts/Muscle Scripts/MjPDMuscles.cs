using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;

namespace Mujoco
{
    public class MjPDMuscles : MjPositionMuscles
    {
        [SerializeField]
        public float kv;

        [SerializeField]
        public Transform kinematicRef;

        [SerializeField]
        public bool useBaseline;

        IReadOnlyList<PDActuator> pdActuators;

        public override IReadOnlyList<MjActuator> Actuators { get => actuatorRoot.GetComponentsInChildren<MjActuator>(); }


        public override void ApplyActions(float[] actions, float actionTimeDelta)
        {
            foreach ((var action, var pd) in actions.Zip(pdActuators, Tuple.Create))
            {
                pd.ApplyAction(action);
            }
        }

        public override float[] GetActionsFromState()
        {
            return pdActuators.Select(pd => pd.CurrentAction).ToArray();
        }

        public override void OnAgentInitialize(DReConAgent agent)
        {
            base.OnAgentInitialize(null);
            pdActuators = actuators.Select(act => new PDActuator(act, FindReference(act), useBaseline)).ToList();
        }

        private MjHingeJoint FindReference(MjActuator act)
        {
            return kinematicRef?.GetComponentsInChildren<MjHingeJoint>().First(rj => rj.name.Contains(act.Joint.name));
        }



        private class PDActuator
        {
            BiasedScale bs;
            MjActuator actuator;
            MjHingeJoint referenceJoint;

            bool useBaseline;
            bool isVelocity;

            public PDActuator(MjActuator act, MjHingeJoint referenceJoint, bool useBaseline=true)
            {
                bs = BiasedScale.ForActuator(act);
                actuator = act;
                this.referenceJoint = referenceJoint;
                isVelocity = IsVelocity(act);
                this.useBaseline = useBaseline;
            }

            public float CurrentAction { get => useBaseline ? (actuator.Control - ReferenceValue) / bs : actuator.Control / bs; }

            float ReferencePosition { get => referenceJoint.Configuration*Mathf.Deg2Rad*actuator.CommonParams.Gear[0]; }
            float ReferenceVelocity { get => referenceJoint.Velocity*Mathf.Deg2Rad*actuator.CommonParams.Gear[0]; }

            float ReferenceValue { get => isVelocity ? ReferenceVelocity : ReferencePosition; }

            public void ApplyAction(float a)
            {
                actuator.Control = useBaseline ? bs * a + ReferenceValue : bs*a;
            }
        }
    }
}