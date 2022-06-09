using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class AutoInput : MonoBehaviour, IMMInput
{

    [SerializeField]
    List<Transform> landmarks;

    [SerializeField]
    LandmarkSelection landmarkSelectionMode; 

    int landmarkIdx;
    protected Vector3 hitPoint;
    protected bool targetIsActive;
    protected DampedTrajectory trajectory;
    [SerializeField]
    protected TrackCircle circle;

    [SerializeField]
    public Transform fauxRootInWorld;

    [SerializeField]
    float distThreshold;

    [SerializeField]
    WaitSettings waitSettings;

    private Vector2 analogueDirection;

    public  bool TargetIsActive
    {
        get => targetIsActive;
        set
        {
            if (targetIsActive != value)
            {
                if (value == true)
                {
                    circle.enabled = true;
                    circle.transform.position = new Vector3(hitPoint.x, 0.05f, hitPoint.z);
                    targetIsActive = true;
                }
                else
                {
                    targetIsActive = false;
                    float waitTime = waitSettings.SampleWaitTime();
                    if (waitTime == 0f)
                    {
                        hitPoint = GetNextPoint();

                        TargetIsActive = true;
                    }
                    else
                    {
                        StartCoroutine(SetNewTarget(waitTime));
                    }

                    
                   
                }
            }

        }
    }

    IEnumerator SetNewTarget(float waitTime)
    {
        yield return new WaitForSeconds(waitTime);

        hitPoint = GetNextPoint();

                        TargetIsActive = true;

    }

    public IEnumerable<Vector2> CurrentTrajectoryAndDirection
    {
        get
        {
            return trajectory.CurrentSamples.Select(v=>new Vector2(v.y, -v.x));
        }
    }

    public IEnumerable<Vector2> CurrentTrajectory
    {
        get
        {
            return trajectory.CurrentTrajectory;
        }
    }

    private void Awake()
    {
        trajectory = new DampedTrajectory();
        targetIsActive = true;
        TargetIsActive = false;
    }

    private void Update()
    {
        if (TargetIsActive)
        {
            Vector3 differenceVector = fauxRootInWorld.position - hitPoint;
            if (differenceVector.magnitude < 1.4)
            {
                analogueDirection = Vector2.zero;
                TargetIsActive = false;
            }
            else
            {
                Vector2 projectedDirection = new Vector3(differenceVector.x, differenceVector.z);

                analogueDirection = Vector2.ClampMagnitude(-projectedDirection, 1);
            }
        }

        trajectory.TargetVelocity = analogueDirection;
        trajectory.UpdateStep(Time.deltaTime);
    }

    private Vector3 GetNextPoint()
    {
        switch(landmarkSelectionMode)
        {
            case LandmarkSelection.Random: return GetRandomPoint();
            case LandmarkSelection.OrderedLandmark: return GetNextLandmark();
            case LandmarkSelection.RandomLandmark: return GetRandomLandmark();
            default: return GetNextLandmark();
        }
    }

    private Vector3 GetNextLandmark()
    {
        landmarkIdx = (landmarkIdx + 1) % landmarks.Count;
        return landmarks[landmarkIdx].position;
    }

    private Vector3 GetRandomLandmark()
    {
        landmarkIdx = Random.Range(0, landmarks.Count);
        return landmarks[landmarkIdx].position;
    }

    private Vector3 GetRandomPoint()
    {
        landmarkIdx = -1;
        var bb = GeometryUtility.CalculateBounds(landmarks.Select(t => t.position).ToArray(), Matrix4x4.identity);
        var newPoint = new Vector3(Random.Range(bb.min.x, bb.max.x), Random.Range(bb.min.y, bb.max.y), Random.Range(bb.min.z, bb.max.z));
        return (fauxRootInWorld.position.Horizontal3D() + new Vector3(0, newPoint.y, 0) - newPoint).magnitude > distThreshold? newPoint : GetRandomPoint();
    }

    public float Eignv { get => trajectory.Eignv; set => trajectory.Eignv = value; }
    public Vector2 AnalogueDirection { get => analogueDirection;}

    protected class DampedTrajectory
    {

        readonly IEnumerable<float> timeSamples = MMUtility.LinSpace(1f / 3f, 1, 3);

        Vector2 localVelocity;
        Vector2 localAcceleration;

        Vector2 targetVelocity;
        public Vector2 TargetVelocity { get => targetVelocity; set => targetVelocity = value; }

        private float eignv = -3;
        public float Eignv { get => eignv; set => eignv = value; }

        Vector2 c1;
        Vector2 c2;
        Vector2 c3;

        public IEnumerable<Vector2> CurrentSamples
        {
            get
            {
                c1 = localVelocity - targetVelocity;
                c2 = localAcceleration - c1 * eignv;
                c3 = c2 - c1 * eignv;
                return TrajectoryHorizon.Concat(DirectionHorizon);
            }
        }

        public IEnumerable<Vector2> CurrentTrajectory
        {
            get
            {
                c1 = localVelocity - targetVelocity;
                c2 = localAcceleration - c1 * eignv;
                c3 = c2 - c1 * eignv;
                return TrajectoryHorizon;
            }
        }

        public void UpdateStep(float dt)
        {
            c1 = localVelocity - targetVelocity;
            c2 = localAcceleration - c1 * eignv;
            localVelocity = VelocityStep(dt);
            localAcceleration = AccelerationStep(dt);
        }

        Vector2 VelocityStep(float dt)
        {
            return targetVelocity + (c1 + c2 * dt) * Mathf.Exp(eignv * dt);
        }

        Vector2 AccelerationStep(float dt)
        {
            return (localAcceleration + c2 * eignv * dt) * (float)System.Math.Exp((eignv * dt)); ;
        }

        Vector2 PositionStep(float dt)
        {
            return targetVelocity * dt + (c3 + (c1 * eignv + c2 * eignv * dt - c2) * (float)System.Math.Exp((eignv * dt))) / (eignv * eignv);
        }

        IEnumerable<Vector2> VelocityHorizon
        {
            get
            {
                return timeSamples.Select(t => VelocityStep(t));
            }
        }

        IEnumerable<Vector2> AccelerationHorizon
        {
            get
            {
                return timeSamples.Select(t => AccelerationStep(t));
            }
        }

        IEnumerable<Vector2> TrajectoryHorizon
        {
            get
            {
                return timeSamples.Select(t => PositionStep(t));
            }
        }

        IEnumerable<Vector2> DirectionHorizon
        {
            get
            {
                return timeSamples.Select(t => VelocityStep(t).normalized);
            }
        }
    }

    [System.Serializable]
    struct WaitSettings
    {
        [SerializeField, Range(0,1)]
        float waitProbability;

        [SerializeField, Tooltip("In seconds")]
        Vector2 waitRange;

        public float SampleWaitTime()
        {
            if (waitProbability == 0f) return 0;
            float a = Random.value;
            if (a < (1f - waitProbability)) return 0f;
            else
            {
                float m = (waitRange.y - waitRange.x) / waitProbability;
                float b = waitRange.y - m;
                return m * a + b;
            }
        }
    }



}

public enum LandmarkSelection
{
    Random,
    OrderedLandmark,
    RandomLandmark,
}

public interface IMMInput
{
    public float Eignv { get; set; }
    public IEnumerable<Vector2> CurrentTrajectoryAndDirection { get; }
}