using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;

namespace Mujoco
{
    public class MjPositionMuscles : MjMuscles
    {

        protected IReadOnlyList<BiasedScale> biasedScales;

        [SerializeField]
        protected bool biasActions;

        [SerializeField]
        public float kp;

        public override IReadOnlyList<MjActuator> Actuators { get => actuatorRoot.GetComponentsInChildren<MjActuator>().Where(act => !IsVelocity(act)).ToList(); }

        

        public override void ApplyActions(float[] actions, float actionTimeDelta)
        {
            foreach ((var action, (var actuator, var bs)) in actions.Zip(actuators.Zip(biasedScales, Tuple.Create), Tuple.Create))
            {
                actuator.Control =  bs*action;
            }
        }

        public override float[] GetActionsFromState()
        {
            return actuators.Zip(biasedScales, (a, bs) => a.Control/bs).ToArray();
        }

        public override void OnAgentInitialize(DReConAgent agent)
        {
            base.OnAgentInitialize(agent);

            biasedScales = actuators.Select(a => BiasedScale.ForActuator(a, biasActions)).ToList();
        }

        protected static bool IsVelocity(MjActuator act) => act.CustomParams.BiasPrm[2] != 0;

        protected struct BiasedScale
        {
            public readonly float scale;
            public readonly float bias;

            public BiasedScale(float scale, float bias)
            {
                this.scale = scale;
                this.bias = bias;
            }

            public static BiasedScale FromRange(float min, float max, bool biasActions= false)
            {
                
                float bias = (max + min) / 2f;

                float scale = max-bias;

                return new BiasedScale(scale, biasActions? bias : 0f);
            }

            public static BiasedScale ForActuator(MjActuator act,  bool biasActions = false)
            {
                return BiasedScale.FromRange(act.CommonParams.CtrlRange.x, act.CommonParams.CtrlRange.y, biasActions);
            }


            public static float operator *(BiasedScale bs, float a) => bs.scale * a + bs.bias;
            public static float operator /(float a, BiasedScale bs) => (a - bs.bias)/bs.scale;
        }
    }
}