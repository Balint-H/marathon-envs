using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Unity.Mathematics;
using Mujoco;

namespace MotorUpdate
{
    public abstract class MotorUpdateRule : ScriptableObject
    {
        // Start is called before the first frame update
        [SerializeField]
        protected float[] gains;

        

        public virtual float GetTorque(float[] curState, float[] targetState)
        {
            float res = 0;
            for (int i = 0; i < gains.Length; i++)
            {
                res -= gains[i] * (curState[i] - targetState[i]);
            }
            return res;
        }

        public virtual float GetTorque(IState curState, IState targetState)
        {
            return GetTorque(curState.stateVector, targetState.stateVector);
        }



    }
    #region Queryable state Adapters
    public interface IState
    {
        public float Acceleration { get; }
        public float Velocity { get; }
        public float Position { get; }

        public float[] stateVector { get; }

        public string Name { get; }

        public GameObject gameObject { get; }
       
    }

   

    public class MjActuatorState : IState
    {
        readonly private MjActuator mjActuator;

        public MjActuatorState(MjActuator mjActuator)
        {
            this.mjActuator = mjActuator;
        }

        public float Velocity => mjActuator.GetVelocity();

        public float Position => mjActuator.GetLength();

        public float Acceleration => mjActuator.GetAcceleration(); 

        public string Name => mjActuator.name;



        public GameObject gameObject { get => mjActuator.gameObject; }

        public float[] stateVector => new float[] { Position, Velocity, Acceleration};
    }

    public class MjHingeJointState : IState
    {
        readonly private MjHingeJoint joint;

        public MjHingeJointState(MjHingeJoint joint)
        {
            this.joint = joint;
        }

        public float Velocity => joint.GetVelocityRad();

        public float Position => joint.GetPositionRad();

        public float Acceleration => joint.GetAccelerationRad();

        public string Name => joint.name;

        public GameObject gameObject { get => joint.gameObject; }

        public float[] stateVector => new float[] { Position, Velocity, Acceleration };
    }

    public class MjPositionState : IState
    {
        readonly private MjActuator mjActuator;

        public MjPositionState(MjActuator mjActuator)
        {
            this.mjActuator = mjActuator;
        }

        public float Velocity => 0f;

        public float Position => mjActuator.Length;

        public float Acceleration => 0f;

        public string Name => mjActuator.name;



        public GameObject gameObject { get => mjActuator.gameObject; }

        public float[] stateVector => new float[] { Position, Velocity, Acceleration };
    }

    public struct StaticState : IState
    {
        readonly float position;
        readonly float velocity;
        readonly float acceleration;

        public StaticState(float position, float velocity, float acceleration)
        {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        public float Acceleration => acceleration;

        public float Velocity => velocity;

        public float Position => position;

        public string Name => "ManualState";

        public GameObject gameObject => null;

        public float[] stateVector => new float[] { position, velocity, acceleration };
    }
    #endregion
}