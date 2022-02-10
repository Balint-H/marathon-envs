using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Target : DelayableEventHandler
{
    // Start is called before the first frame update
    [SerializeField]
    Collider projectile;

    [SerializeField]
    bool shouldTrackHeight;

    [SerializeField]
    float targetHeight;

    public bool h;

    Vector3 lastPosition;

    [SerializeField]
    Vector3 idealPosition;

    [SerializeField]
    Transform pivot;
    Matrix4x4 pivotMatrix;

    [SerializeField]
    bool shouldMoveTarget;

    [SerializeField]
    Transform currentTarget;

    [SerializeField]
    float radiusRange = 0.5f;

    [SerializeField]
    float angleRange = 0.1f;

    [SerializeField]
    float heightRange = 0.125f;

    public override EventHandler Handler => (sender, args) => WrappedReset();


    private void Awake()
    {
        pivotMatrix = pivot.localToWorldMatrix;
        h = false;
    }



    void Start()
    {
        lastPosition = projectile.transform.position;
    }

    private void FixedUpdate()
    {
        if (shouldTrackHeight)
        {
            
            var curPosition = projectile.transform.position;


            if (curPosition.y <= targetHeight && lastPosition.y > targetHeight) idealPosition = curPosition;

            lastPosition = projectile.transform.position;
        }


    }

    void WrappedReset()
    {
        if (IsWaiting) return;
        if (framesToWait != 0)
        {
            StartCoroutine(DelayedExecution(this, EventArgs.Empty));
            return;
        }

        ResetTarget();
    }

    void ResetTarget()
    {
        h = false;
        if (!shouldMoveTarget) return;
        Vector3 relTargetPos = pivotMatrix.inverse.MultiplyPoint3x4(idealPosition);
        float meanRadius = relTargetPos.Horizontal3D().magnitude;
        float meanAngle = Mathf.Deg2Rad * Vector3.Angle(Vector3.right, relTargetPos.Horizontal3D());
        float meanHeight = relTargetPos.y;

        

        float sampledRadius = UnityEngine.Random.Range(meanRadius - radiusRange, meanRadius + radiusRange);
        float sampledAngle = UnityEngine.Random.Range(meanAngle - angleRange, meanAngle + angleRange);
        float sampledHeight = UnityEngine.Random.Range(meanHeight - heightRange, meanHeight + heightRange);


        Vector3 sampledPosition = pivotMatrix.MultiplyPoint3x4(new Vector3(Mathf.Cos(sampledAngle) * sampledRadius, sampledHeight, Mathf.Sin(sampledAngle) * sampledRadius));

        currentTarget.position = sampledPosition;
    }

    private void OnCollisionEnter(Collision collision)
    {
        h = true;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    protected override IEnumerator DelayedExecution(object sender, EventArgs args)
    {
        IsWaiting = true;
        yield return WaitFrames();
        ResetTarget();
        IsWaiting = false;
    }
}
