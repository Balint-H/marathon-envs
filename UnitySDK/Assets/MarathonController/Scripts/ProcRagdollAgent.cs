using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using ManyWorlds;
using UnityEngine.Assertions;

using System;

public class ProcRagdollAgent : Agent
{
    [Header("Settings")]
    public float FixedDeltaTime = 1f / 60f;
    public float SmoothBeta = 0.2f;
    public bool ReproduceDReCon;

    [Header("Camera")]

    public bool RequestCamera;
    public bool CameraFollowMe;
    public Transform CameraTarget;

    [Header("... debug")]
    public bool SkipRewardSmoothing;
    public bool debugCopyMocap;
    public bool ignorActions;
    public bool dontResetOnZeroReward;
    public bool dontSnapMocapToRagdoll;
    public bool DebugPauseOnReset;
    public bool dontResetWhenOutOfBounds;



    List<Rigidbody> _mocapBodyParts;
    SpawnableEnv _spawnableEnv;
    Observations2Learn _dReConObservations;
    Rewards2Learn _dReConRewards;
    Muscles _ragDollSettings;
    List<ArticulationBody> _motors;
    MarathonTestBedController _debugController;
    InputController _inputController;
    SensorObservations _sensorObservations;
    DecisionRequester _decisionRequester;
    IAnimationController _mocapAnimatorController;


    bool _hasLazyInitialized;
    float[] _smoothedActions;
    float[] _mocapTargets;

    [Space(16)]
    [SerializeField]
    bool _hasAwake = false;
    MapAnim2Ragdoll _mocapControllerArtanim;

    void Awake()
    {
        if (RequestCamera && CameraTarget != null)
        {
            // Will follow the last object to be spawned
            var camera = FindObjectOfType<Camera>();
            if (camera != null)
            {
                var follow = camera.GetComponent<SmoothFollow>();
                if (follow != null)
                    follow.target = CameraTarget;
            }
        }
        _hasAwake = true;
    }
    void Update()
    {
        if (debugCopyMocap)
        {
            EndEpisode();
        }

        Assert.IsTrue(_hasLazyInitialized);

        // hadle mocap going out of bounds
        bool isOutOfBounds = !_spawnableEnv.IsPointWithinBoundsInWorldSpace(_mocapControllerArtanim.transform.position+new Vector3(0f, .1f, 0f));
        bool reset = isOutOfBounds && dontResetWhenOutOfBounds == false;
        if (reset)
        {
            _mocapControllerArtanim.transform.position = _spawnableEnv.transform.position;
            EndEpisode();
        }
    }
    override public void CollectObservations(VectorSensor sensor)
    {
        Assert.IsTrue(_hasLazyInitialized);

        float timeDelta = Time.fixedDeltaTime * _decisionRequester.DecisionPeriod;
        _dReConObservations.OnStep(timeDelta);

        if (ReproduceDReCon)
        {
            AddDReConObservations(sensor);
            return;
        }

        sensor.AddObservation(_dReConObservations.MocapCOMVelocity);
        sensor.AddObservation(_dReConObservations.RagDollCOMVelocity);
        sensor.AddObservation(_dReConObservations.RagDollCOMVelocity - _dReConObservations.MocapCOMVelocity);
        sensor.AddObservation(_dReConObservations.InputDesiredHorizontalVelocity);
        sensor.AddObservation(_dReConObservations.InputJump);
        sensor.AddObservation(_dReConObservations.InputBackflip);
        sensor.AddObservation(_dReConObservations.HorizontalVelocityDifference);
        // foreach (var stat in _dReConObservations.MocapBodyStats)
        // {
        //     sensor.AddObservation(stat.Position);
        //     sensor.AddObservation(stat.Velocity);
        // }
        foreach (var stat in _dReConObservations.RagDollBodyStats)
        {
            sensor.AddObservation(stat.Position);
            sensor.AddObservation(stat.Velocity);
        }
        foreach (var stat in _dReConObservations.BodyPartDifferenceStats)
        {
            sensor.AddObservation(stat.Position);
            sensor.AddObservation(stat.Velocity);
        }
        sensor.AddObservation(_dReConObservations.PreviousActions);

        // add sensors (feet etc)
        sensor.AddObservation(_sensorObservations.SensorIsInTouch);
    }
    void AddDReConObservations(VectorSensor sensor)
    {
        sensor.AddObservation(_dReConObservations.MocapCOMVelocity);
        sensor.AddObservation(_dReConObservations.RagDollCOMVelocity);
        sensor.AddObservation(_dReConObservations.RagDollCOMVelocity - _dReConObservations.MocapCOMVelocity);
        sensor.AddObservation(_dReConObservations.InputDesiredHorizontalVelocity);
        sensor.AddObservation(_dReConObservations.InputJump);
        sensor.AddObservation(_dReConObservations.InputBackflip);
        sensor.AddObservation(_dReConObservations.HorizontalVelocityDifference);
        // foreach (var stat in _dReConObservations.MocapBodyStats)
        // {
        //     sensor.AddObservation(stat.Position);
        //     sensor.AddObservation(stat.Velocity);
        // }
        foreach (var stat in _dReConObservations.RagDollBodyStats)
        {
            sensor.AddObservation(stat.Position);
            sensor.AddObservation(stat.Velocity);
        }
        foreach (var stat in _dReConObservations.BodyPartDifferenceStats)
        {
            sensor.AddObservation(stat.Position);
            sensor.AddObservation(stat.Velocity);
        }
        sensor.AddObservation(_dReConObservations.PreviousActions);
    }

    //adapted from previous function (Collect Observations)
    public int calculateDreConObservationsize()
    {
        int size = 0;

        size +=
        3  //sensor.AddObservation(_dReConObservations.MocapCOMVelocity);
        + 3 //sensor.AddObservation(_dReConObservations.RagDollCOMVelocity);
        + 3 //sensor.AddObservation(_dReConObservations.RagDollCOMVelocity - _dReConObservations.MocapCOMVelocity);
        + 2 //sensor.AddObservation(_dReConObservations.InputDesiredHorizontalVelocity);
        + 1 //sensor.AddObservation(_dReConObservations.InputJump);
        + 1 //sensor.AddObservation(_dReConObservations.InputBackflip);
        + 2;//sensor.AddObservation(_dReConObservations.HorizontalVelocityDifference);


        Observations2Learn _checkDrecon = GetComponent<Observations2Learn>();


        //foreach (var stat in _dReConObservations.RagDollBodyStats)

        foreach (var collider in _checkDrecon.EstimateBodyPartsForObservation())

        {
            size +=
             3 //sensor.AddObservation(stat.Position);
             + 3; //sensor.AddObservation(stat.Velocity);
        }
        //foreach (var stat in _dReConObservations.BodyPartDifferenceStats)
        foreach (var collider in _checkDrecon.EstimateBodyPartsForObservation())

        {
            size +=
            +3 // sensor.AddObservation(stat.Position);
            + 3; // sensor.AddObservation(stat.Velocity);
        }

        //action size and sensor size are calculated separately, we do not use:
        //sensor.AddObservation(_dReConObservations.PreviousActions);
        //sensor.AddObservation(_sensorObservations.SensorIsInTouch);

        return size;
    }






    public override void OnActionReceived(float[] vectorAction)
    {
        Assert.IsTrue(_hasLazyInitialized);

        float timeDelta = Time.fixedDeltaTime;
        if (!_decisionRequester.TakeActionsBetweenDecisions)
            timeDelta = timeDelta*_decisionRequester.DecisionPeriod;
        _dReConRewards.OnStep(timeDelta);

        bool shouldDebug = _debugController != null;
        bool dontUpdateMotor = false;
        if (_debugController != null)
        {
            dontUpdateMotor = _debugController.DontUpdateMotor;
            dontUpdateMotor &= _debugController.isActiveAndEnabled;
            dontUpdateMotor &= _debugController.gameObject.activeInHierarchy;
            shouldDebug &= _debugController.isActiveAndEnabled;
            shouldDebug &= _debugController.gameObject.activeInHierarchy;
        }
        if (shouldDebug)
        {
            vectorAction = GetDebugActions(vectorAction);
        }

        if (!SkipRewardSmoothing)
            vectorAction = SmoothActions(vectorAction);
        int i = 0;
        foreach (var m in _motors)
        {
            if (m.isRoot)
                continue;
            if (dontUpdateMotor)
                continue;
            Vector3 targetNormalizedRotation = Vector3.zero;
			if (m.jointType != ArticulationJointType.SphericalJoint)
                continue;
            if (m.twistLock == ArticulationDofLock.LimitedMotion)
                targetNormalizedRotation.x = vectorAction[i++];
            if (m.swingYLock == ArticulationDofLock.LimitedMotion)
                targetNormalizedRotation.y = vectorAction[i++];
            if (m.swingZLock == ArticulationDofLock.LimitedMotion)
                targetNormalizedRotation.z = vectorAction[i++];
            if (!ignorActions)
            {
                UpdateMotor(m, targetNormalizedRotation);
            }
        }
        _dReConObservations.PreviousActions = vectorAction;

        AddReward(_dReConRewards.Reward);

        if (ReproduceDReCon)
        {
            // DReCon Logic
            if (_dReConRewards.HeadHeightDistance > 0.5f || _dReConRewards.Reward <= 0f)
            {
                if (!dontResetOnZeroReward)
                    EndEpisode();
            }
            else if (_dReConRewards.Reward <= 0.1f && !dontSnapMocapToRagdoll)
            {
                Transform ragDollCom = _dReConObservations.GetRagDollCOM();
                Vector3 snapPosition = ragDollCom.position;
                // snapPosition.y = 0f;
                var snapDistance = _mocapControllerArtanim.SnapTo(snapPosition);
                // AddReward(-.5f);
            }
        }
        else
        {
            // Our Logic
            bool terminate = false;
            // terminate = terminate || _dReConRewards.PositionReward < 1E-5f;
            // terminate = terminate || _dReConRewards.ComVelocityReward < 1E-20f;
            // // terminate = terminate || _dReConRewards.ComDirectionReward < .01f;
            // // terminate = terminate || _dReConRewards.PointsVelocityReward < 1E-5f;
            // terminate = terminate || _dReConRewards.LocalPoseReward < 1E-5f;
            // // terminate = terminate || _dReConRewards.HeadHeightDistance > 0.5f;
            terminate = terminate || _dReConRewards.PositionReward < .01f;
            // terminate = terminate || _dReConRewards.ComVelocityReward < .01f;
            terminate = terminate || _dReConRewards.ComDirectionReward < .01f;
            if (_dReConRewards.PointsVelocityReward > 0f) // HACK
                terminate = terminate || _dReConRewards.PointsVelocityReward < .01f;
            terminate = terminate || _dReConRewards.LocalPoseReward < .01f;
            if (dontResetOnZeroReward)
                terminate = false;
            // terminate = false; // HACK disable for now
            // if (terminate && StepCount > 9)
            if (terminate)
            {
                EndEpisode();
            }
            else if (!dontSnapMocapToRagdoll)
            {
                Transform ragDollCom = _dReConObservations.GetRagDollCOM();
                Vector3 snapPosition = ragDollCom.position;
                // snapPosition.y = 0f;
                var snapDistance = _mocapControllerArtanim.SnapTo(snapPosition);
                // AddReward(-.5f);
            }            
        }
    }
    float[] GetDebugActions(float[] vectorAction)
    {
        var debugActions = new List<float>();
        foreach (var m in _motors)
        {
            if (m.isRoot)
                continue;
            DebugMotor debugMotor = m.GetComponent<DebugMotor>();
            if (debugMotor == null)
            {
                debugMotor = m.gameObject.AddComponent<DebugMotor>();
            }
            // clip to -1/+1
            debugMotor.Actions = new Vector3(
                Mathf.Clamp(debugMotor.Actions.x, -1f, 1f),
                Mathf.Clamp(debugMotor.Actions.y, -1f, 1f),
                Mathf.Clamp(debugMotor.Actions.z, -1f, 1f)
            );
            Vector3 targetNormalizedRotation = debugMotor.Actions;

			if (m.jointType != ArticulationJointType.SphericalJoint)
                continue;
            if (m.twistLock == ArticulationDofLock.LimitedMotion)
                debugActions.Add(targetNormalizedRotation.x);
            if (m.swingYLock == ArticulationDofLock.LimitedMotion)
                debugActions.Add(targetNormalizedRotation.y);
            if (m.swingZLock == ArticulationDofLock.LimitedMotion)
                debugActions.Add(targetNormalizedRotation.z);
        }

        debugActions = debugActions.Select(x => Mathf.Clamp(x, -1f, 1f)).ToList();
        if (_debugController.ApplyRandomActions)
        {
            debugActions = debugActions
                .Select(x => UnityEngine.Random.Range(-_debugController.RandomRange, _debugController.RandomRange))
                .ToList();
        }

        _debugController.Actions = debugActions.ToArray();
        return debugActions.ToArray();
    }

    float[] SmoothActions(float[] vectorAction)
    {
        // yt =β at +(1−β)yt−1
        var smoothedActions = vectorAction
            .Zip(_dReConObservations.PreviousActions, (a, y) => SmoothBeta * a + (1f - SmoothBeta) * y)
            .ToArray();
        return smoothedActions;
    }
    float[] GetActionsFromRagdollState()
    {
        var vectorActions = new List<float>();
        foreach (var m in _motors)
        {
            if (m.isRoot)
                continue;
            int i = 0;
			if (m.jointType != ArticulationJointType.SphericalJoint)
                continue;
            if (m.twistLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = m.xDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var deg = m.jointPosition[i++] * Mathf.Rad2Deg;
                var target = (deg - midpoint) / scale;
                vectorActions.Add(target);
            }
            if (m.swingYLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = m.yDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var deg = m.jointPosition[i++] * Mathf.Rad2Deg;
                var target = (deg - midpoint) / scale;
                vectorActions.Add(target);
            }
            if (m.swingZLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = m.zDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var deg = m.jointPosition[i++] * Mathf.Rad2Deg;
                var target = (deg - midpoint) / scale;
                vectorActions.Add(target);
            }
        }
        return vectorActions.ToArray();
    }
    public override void Initialize()
    {
        Assert.IsTrue(_hasAwake);
        Assert.IsFalse(_hasLazyInitialized);
        _hasLazyInitialized = true;

        _decisionRequester = GetComponent<DecisionRequester>();
        _debugController = FindObjectOfType<MarathonTestBedController>();
        Time.fixedDeltaTime = FixedDeltaTime;
        _spawnableEnv = GetComponentInParent<SpawnableEnv>();

        if (_debugController != null)
        {
            dontResetOnZeroReward = true;
            dontSnapMocapToRagdoll = true;
        }

        _mocapControllerArtanim = _spawnableEnv.GetComponentInChildren<MapAnim2Ragdoll>();
        _mocapBodyParts = _mocapControllerArtanim.GetRigidBodies();

        _dReConObservations = GetComponent<Observations2Learn>();
        _dReConRewards = GetComponent<Rewards2Learn>();

        _ragDollSettings = GetComponent<Muscles>();
        _inputController = _spawnableEnv.GetComponentInChildren<InputController>();
        _sensorObservations = GetComponent<SensorObservations>();


        _motors = GetComponentsInChildren<ArticulationBody>()
            .Where(x => x.jointType == ArticulationJointType.SphericalJoint)
            .Where(x => !x.isRoot)
            .Distinct()
            .ToList();
        var individualMotors = new List<float>();
        _dReConObservations.PreviousActions = GetActionsFromRagdollState();

        _mocapAnimatorController = _mocapControllerArtanim.GetComponent<IAnimationController>();



        _mocapControllerArtanim.OnAgentInitialize();
        _dReConObservations.OnAgentInitialize();
        _dReConRewards.OnAgentInitialize(ReproduceDReCon);
        _mocapAnimatorController.OnAgentInitialize();

        _hasLazyInitialized = true;
    }
    public override void OnEpisodeBegin()
    {
        Assert.IsTrue(_hasAwake);
        _smoothedActions = null;
        debugCopyMocap = false;

        _mocapAnimatorController.OnReset();
        Vector3 resetVelocity = Vector3.zero;
        if (_inputController != null)
        {
            // resets to source anim
            _inputController.OnReset();
            var angle = Vector3.SignedAngle(Vector3.forward, _inputController.HorizontalDirection, Vector3.up);
            var rotation = Quaternion.Euler(0f, angle, 0f);
            _mocapControllerArtanim.OnReset(rotation);
            resetVelocity = Vector3.zero;
            _mocapControllerArtanim.CopyStatesTo(this.gameObject);
        }
        else
        {
            // source anim is continious
            var rotation = _mocapControllerArtanim.transform.rotation;
            _mocapControllerArtanim.OnReset(rotation);
            resetVelocity = _mocapAnimatorController.GetDesiredVelocity();
            _mocapControllerArtanim.CopyStatesTo(this.gameObject);
            _mocapControllerArtanim.CopyVelocityTo(this.gameObject, resetVelocity);
        }

        float timeDelta = float.Epsilon;
        _dReConObservations.OnReset();
        _dReConRewards.OnReset();
        _dReConObservations.OnStep(timeDelta);
        _dReConRewards.OnStep(timeDelta);
#if UNITY_EDITOR		
		if (DebugPauseOnReset)
		{
	        UnityEditor.EditorApplication.isPaused = true;
		}
#endif	        
        if (_debugController != null && _debugController.isActiveAndEnabled)
        {
            _debugController.OnAgentEpisodeBegin();
        }
        _dReConObservations.PreviousActions = GetActionsFromRagdollState();
    }

    float[] GetMocapTargets()
    {
        if (_mocapTargets == null)
        {
            _mocapTargets = _motors
                .Where(x => !x.isRoot)
                .SelectMany(x => {
                    List<float> list = new List<float>();
        			if (x.jointType != ArticulationJointType.SphericalJoint)
                        return list.ToArray();
                    if (x.twistLock == ArticulationDofLock.LimitedMotion)
                        list.Add(0f);
                    if (x.swingYLock == ArticulationDofLock.LimitedMotion)
                        list.Add(0f);
                    if (x.swingZLock == ArticulationDofLock.LimitedMotion)
                        list.Add(0f);
                    return list.ToArray();
                })
                .ToArray();
        }
        int i = 0;
        foreach (var joint in _motors)
        {
            if (joint.isRoot)
                continue;
            Rigidbody mocapBody = _mocapBodyParts.First(x => x.name == joint.name);
            Vector3 targetRotationInJointSpace = -(Quaternion.Inverse(joint.anchorRotation) * Quaternion.Inverse(mocapBody.transform.localRotation) * joint.parentAnchorRotation).eulerAngles;
            targetRotationInJointSpace = new Vector3(
                Mathf.DeltaAngle(0, targetRotationInJointSpace.x),
                Mathf.DeltaAngle(0, targetRotationInJointSpace.y),
                Mathf.DeltaAngle(0, targetRotationInJointSpace.z));
			if (joint.jointType != ArticulationJointType.SphericalJoint)
                continue;
            if (joint.twistLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.xDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var target = (targetRotationInJointSpace.x - midpoint) / scale;
                _mocapTargets[i] = target;
                i++;
            }
            if (joint.swingYLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.yDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var target = (targetRotationInJointSpace.y - midpoint) / scale;
                _mocapTargets[i] = target;
                i++;
            }
            if (joint.swingZLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.zDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var target = (targetRotationInJointSpace.z - midpoint) / scale;
                _mocapTargets[i] = target;
                i++;
            }
        }
        return _mocapTargets;
    }

    void UpdateMotor(ArticulationBody joint, Vector3 targetNormalizedRotation)
    {
        Vector3 power = Vector3.zero;
        try
        {
            power = _ragDollSettings.MusclePowers.First(x => x.Muscle == joint.name).PowerVector;

        }
        catch (Exception e)
        {
            Debug.Log("there is no muscle for joint " + joint.name);

        }

        // For a physically realistic simulation - ,  
        var m = joint.mass;
        var d = _ragDollSettings.DampingRatio; // d should be 0..1.
        var n = _ragDollSettings.NaturalFrequency; // n should be in the range 1..20
        var k = Mathf.Pow(n,2) * m;
        var c = d * (2 * Mathf.Sqrt(k * m));
        var stiffness = k;
        var damping = c;

        if (joint.twistLock == ArticulationDofLock.LimitedMotion)
        {
            var drive = joint.xDrive;
            var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
            var midpoint = drive.lowerLimit + scale;
            var target = midpoint + (targetNormalizedRotation.x * scale);
            drive.target = target;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = power.x * _ragDollSettings.ForceScale;
            joint.xDrive = drive;
        }

        if (joint.swingYLock == ArticulationDofLock.LimitedMotion)
        {
            var drive = joint.yDrive;
            var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
            var midpoint = drive.lowerLimit + scale;
            var target = midpoint + (targetNormalizedRotation.y * scale);
            drive.target = target;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = power.y * _ragDollSettings.ForceScale;
            joint.yDrive = drive;
        }

        if (joint.swingZLock == ArticulationDofLock.LimitedMotion)
        {
            var drive = joint.zDrive;
            var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
            var midpoint = drive.lowerLimit + scale;
            var target = midpoint + (targetNormalizedRotation.z * scale);
            drive.target = target;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = power.z * _ragDollSettings.ForceScale;
            joint.zDrive = drive;
        }
    }

    void FixedUpdate()
    {
        if (debugCopyMocap)
        {
            EndEpisode();
        }
    }
    void OnDrawGizmos()
    {
        if (_dReConRewards == null || _inputController == null)
            return;
        var comTransform = _dReConRewards._ragDollBodyStats.transform;
        var vector = new Vector3(_inputController.MovementVector.x, 0f, _inputController.MovementVector.y);
        var pos = new Vector3(comTransform.position.x, 0.001f, comTransform.position.z);
        DrawArrow(pos, vector, Color.black);
    }
    void DrawArrow(Vector3 start, Vector3 vector, Color color)
    {
        float headSize = 0.25f;
        float headAngle = 20.0f;
        Gizmos.color = color;
        Gizmos.DrawRay(start, vector);
        if (vector != Vector3.zero)
        {
            Vector3 right = Quaternion.LookRotation(vector) * Quaternion.Euler(0, 180 + headAngle, 0) * new Vector3(0, 0, 1);
            Vector3 left = Quaternion.LookRotation(vector) * Quaternion.Euler(0, 180 - headAngle, 0) * new Vector3(0, 0, 1);
            Gizmos.DrawRay(start + vector, right * headSize);
            Gizmos.DrawRay(start + vector, left * headSize);
        }
    }
}
