﻿using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using UnityEngine;
using UnityEngine.Assertions;
using ManyWorlds;

public class DReConRewardStats : MonoBehaviour
{
    [Header("Settings")]

    public MonoBehaviour ObjectToTrack;

    [Header("Stats")]
    public Vector3 CenterOfMassVelocity;
    public float CenterOfMassVelocityMagnitude;

    // [Header("debug")]
    // public Vector3 debugA;
    // public Vector3 debugB;
    // public Vector3 debugC;

    [HideInInspector]
    public Vector3 LastCenterOfMassInWorldSpace;
    [HideInInspector]
    public bool LastIsSet;

    SpawnableEnv _spawnableEnv;
    List<Collider> _colliders;
    List<Rigidbody> _rigidbodyParts;
    List<ArticulationBody> _articulationBodyParts;
    List<GameObject> _bodyParts;
    GameObject _root;
    List<GameObject> _trackRotations;
    public List<Quaternion> Rotations;
    public Vector3[] Points;
    Vector3[] _lastPoints;
    public Vector3[] PointVelocity;

    [Header("Stats")]
    public List<string> ColliderNames;
    public List<string> RotationNames;
    public List<string> BodyPartNames;
    bool _hasLazyInitialized;

    public void OnAgentInitialize(Transform defaultTransform, DReConRewardStats orderToCopy = null)
    {
        Assert.IsFalse(_hasLazyInitialized);
        _hasLazyInitialized = true;
                
        _spawnableEnv = GetComponentInParent<SpawnableEnv>();
        _articulationBodyParts = ObjectToTrack
            .GetComponentsInChildren<ArticulationBody>()
            .Distinct()
            .ToList();
        _rigidbodyParts = ObjectToTrack
            .GetComponentsInChildren<Rigidbody>()
            .Distinct()
            .ToList();
        if (_rigidbodyParts?.Count>0)
            _bodyParts = _rigidbodyParts.Select(x=>x.gameObject).ToList();
        else
            _bodyParts = _articulationBodyParts.Select(x=>x.gameObject).ToList();
        _trackRotations = _bodyParts
            .SelectMany(x=>x.GetComponentsInChildren<Transform>())
            .Select(x=>x.gameObject)
            .Distinct()
            .Where(x=>x.GetComponent<Rigidbody>() != null || x.GetComponent<ArticulationBody>() != null)
            .ToList();
        _colliders = _bodyParts
            .SelectMany(x=>x.GetComponentsInChildren<Collider>())
            .Where(x=>x.enabled)
            .Where(x=>!x.name.Contains("senor"))
            .Distinct()
            .ToList();
        if (orderToCopy != null)
        {
            _bodyParts = orderToCopy._bodyParts
                .Select(x=>_bodyParts.First(y=>y.name == x.name))
                .ToList();
            _trackRotations = orderToCopy._trackRotations
                .Select(x=>_trackRotations.First(y=>y.name == x.name))
                .ToList();
            _colliders = orderToCopy._colliders
                .Select(x=>_colliders.First(y=>y.name == x.name))
                .ToList();
        }
        Points = Enumerable.Range(0,_colliders.Count * 6)
            .Select(x=>Vector3.zero)
            .ToArray();
        _lastPoints = Enumerable.Range(0,_colliders.Count * 6)
            .Select(x=>Vector3.zero)
            .ToArray();            
        PointVelocity = Enumerable.Range(0,_colliders.Count * 6)
            .Select(x=>Vector3.zero)
            .ToArray();
        Rotations = Enumerable.Range(0,_trackRotations.Count)
            .Select(x=>Quaternion.identity)
            .ToList();
        if (_root == null)
        {
            _root = _bodyParts.First(x=>x.name== "articulation:Hips");
        }        
        transform.position = defaultTransform.position;
        transform.rotation = defaultTransform.rotation;
        ColliderNames = _colliders
            .Select(x=>x.name)
            .ToList();
        RotationNames = _trackRotations
            .Select(x=>x.name)
            .ToList();
        BodyPartNames = _bodyParts
            .Select(x=>x.name)
            .ToList();
    }
    public void OnReset()
    {
        Assert.IsTrue(_hasLazyInitialized);
        ResetStatus();
        LastIsSet = false;
    }
    public void ResetStatus()
    {
        CenterOfMassVelocity = Vector3.zero;
        CenterOfMassVelocityMagnitude = 0f;
        LastCenterOfMassInWorldSpace = transform.position;
        GetAllPoints(Points);
        Array.Copy(Points, 0, _lastPoints, 0, Points.Length);
        for (int i = 0; i < Points.Length; i++)
        {
            PointVelocity[i] = Vector3.zero;
        }
        for (int i = 0; i < _trackRotations.Count; i++)
        {
            Quaternion localRotation = _trackRotations[i].transform.localRotation;
            if (_trackRotations[i].gameObject == _root)
                localRotation = Quaternion.Inverse(transform.rotation) * _trackRotations[i].transform.rotation;
            Rotations[i] = localRotation;
        }
    }

    public void SetStatusForStep(float timeDelta)
    {
        // find Center Of Mass and velocity
        Vector3 newCOM;
        if (_rigidbodyParts?.Count > 0)
            newCOM = GetCenterOfMass(_rigidbodyParts);
        else
            newCOM = GetCenterOfMass(_articulationBodyParts);
        if (!LastIsSet)
        {
            LastCenterOfMassInWorldSpace = newCOM;
        }

        // generate Horizontal Direction
        var newHorizontalDirection = new Vector3(0f, _root.transform.eulerAngles.y, 0f);

        // set this object to be f space
        transform.position = newCOM;
        transform.rotation = Quaternion.Euler(newHorizontalDirection);

        // get Center Of Mass velocity in f space
        var velocity = transform.position - LastCenterOfMassInWorldSpace;
        velocity /= timeDelta;
        CenterOfMassVelocity = transform.InverseTransformVector(velocity);
        CenterOfMassVelocityMagnitude = CenterOfMassVelocity.magnitude;

        LastCenterOfMassInWorldSpace = newCOM;
        
        GetAllPoints(Points);
        if (!LastIsSet)
        {
            Array.Copy(Points, 0, _lastPoints, 0, Points.Length);
        }
        for (int i = 0; i < Points.Length; i++)
        {
            PointVelocity[i] = (Points[i] - _lastPoints[i]) / timeDelta;
        }
        Array.Copy(Points, 0, _lastPoints, 0, Points.Length);

        for (int i = 0; i < _trackRotations.Count; i++)
        {
            Quaternion localRotation = _trackRotations[i].transform.localRotation;
            if (_trackRotations[i].gameObject == _root)
                localRotation = Quaternion.Inverse(transform.rotation) * _trackRotations[i].transform.rotation;
            Rotations[i] = localRotation;

        }

        LastIsSet = true;
    }

    public List<float> GetPointDistancesFrom(DReConRewardStats target)
    {
        List<float> distances = new List<float>();
        for (int i = 0; i < Points.Length; i++)
        {
            float distance = (Points[i] - target.Points[i]).magnitude;
            distances.Add(distance);
        }
        return distances;
    }

    public List<float> GetPointVelocityDistancesFrom(DReConRewardStats target) {
        List<float> distances = new List<float>();
        for (int i = 0; i < PointVelocity.Length; i++) {
            float distance = (PointVelocity[i] - target.PointVelocity[i]).magnitude;
            distances.Add(distance);
        }
        return distances;
    }

    public void AssertIsCompatible(DReConRewardStats target)
    {
        Assert.AreEqual(Points.Length, target.Points.Length);
        Assert.AreEqual(_lastPoints.Length, target._lastPoints.Length);
        Assert.AreEqual(PointVelocity.Length, target.PointVelocity.Length);
        Assert.AreEqual(Points.Length, _lastPoints.Length);
        Assert.AreEqual(Points.Length, PointVelocity.Length);
        Assert.AreEqual(_colliders.Count, target._colliders.Count);
        for (int i = 0; i < _colliders.Count; i++)
        {
            string debugStr = $" _colliders.{_colliders[i].name} vs target._colliders.{target._colliders[i].name}";
            Assert.AreEqual(_colliders[i].name, target._colliders[i].name, $"name:{debugStr}");
            // Assert.AreEqual(_colliders[i].direction, target._colliders[i].direction, $"direction:{debugStr}");
            // Assert.AreEqual(_colliders[i].height, target._colliders[i].height, $"height:{debugStr}");
            // Assert.AreEqual(_colliders[i].radius, target._colliders[i].radius, $"radius:{debugStr}");
        }
        Assert.AreEqual(ColliderNames.Count, target.ColliderNames.Count);
        Assert.AreEqual(RotationNames.Count, target.RotationNames.Count);
        Assert.AreEqual(BodyPartNames.Count, target.BodyPartNames.Count);
        for (int i = 0; i < ColliderNames.Count; i++)
            Assert.AreEqual(ColliderNames[i], target.ColliderNames[i]);
        for (int i = 0; i < RotationNames.Count; i++)
            Assert.AreEqual(RotationNames[i], target.RotationNames[i]);
        for (int i = 0; i < BodyPartNames.Count; i++)
            Assert.AreEqual(BodyPartNames[i], target.BodyPartNames[i]);
    }

    void GetAllPoints(Vector3[] pointBuffer)
    {
        int idx = 0;
        foreach (var collider in _colliders)
        {
            CapsuleCollider capsule = collider as CapsuleCollider;
            BoxCollider box = collider as BoxCollider;
            SphereCollider sphere = collider as SphereCollider;
            if (capsule != null)
                idx = SetCapusalPoints(capsule, pointBuffer, idx);
            else if (box != null)
                idx = SetBoxPoints(box, pointBuffer, idx);
            else if (sphere != null)
                idx = SetSpherePoints(sphere, pointBuffer, idx);
            else
                throw new NotImplementedException();
        }
    }
    int SetBoxPoints(BoxCollider collider, Vector3[] pointBuffer, int idx)
    {
        Vector3 c = collider.center;
        var b = new Bounds(c, collider.size);

        Vector3 point1, point2, point3, point4, point5, point6;
        point1 = collider.transform.TransformPoint(new Vector3(b.max.x, c.y, c.z));
        point2 = collider.transform.TransformPoint(new Vector3(b.min.x, c.y, c.z));
        point3 = collider.transform.TransformPoint(new Vector3(c.x, b.max.y, c.z));
        point4 = collider.transform.TransformPoint(new Vector3(c.x, b.min.y, c.z));
        point5 = collider.transform.TransformPoint(new Vector3(c.x, c.y, b.max.z));
        point6 = collider.transform.TransformPoint(new Vector3(c.x, c.y, b.min.z));

        // transform from world space, into local space for COM 
        point1 = this.transform.InverseTransformPoint(point1);
        point2 = this.transform.InverseTransformPoint(point2);
        point3 = this.transform.InverseTransformPoint(point3);
        point4 = this.transform.InverseTransformPoint(point4);
        point5 = this.transform.InverseTransformPoint(point5);
        point6 = this.transform.InverseTransformPoint(point6);

        pointBuffer[idx++] = point1;
        pointBuffer[idx++] = point2;
        pointBuffer[idx++] = point3;
        pointBuffer[idx++] = point4;
        pointBuffer[idx++] = point5;
        pointBuffer[idx++] = point6;

        return idx;
    }
    int SetSpherePoints(SphereCollider collider, Vector3[] pointBuffer, int idx)
    {
        Vector3 c = collider.center;
        var r = collider.radius;
        var b = new Bounds(c, new Vector3(r,r,r));

        Vector3 point1, point2, point3, point4, point5, point6;
        point1 = collider.transform.TransformPoint(new Vector3(b.max.x, c.y, c.z));
        point2 = collider.transform.TransformPoint(new Vector3(b.min.x, c.y, c.z));
        point3 = collider.transform.TransformPoint(new Vector3(c.x, b.max.y, c.z));
        point4 = collider.transform.TransformPoint(new Vector3(c.x, b.min.y, c.z));
        point5 = collider.transform.TransformPoint(new Vector3(c.x, c.y, b.max.z));
        point6 = collider.transform.TransformPoint(new Vector3(c.x, c.y, b.min.z));

        // transform from world space, into local space for COM 
        point1 = this.transform.InverseTransformPoint(point1);
        point2 = this.transform.InverseTransformPoint(point2);
        point3 = this.transform.InverseTransformPoint(point3);
        point4 = this.transform.InverseTransformPoint(point4);
        point5 = this.transform.InverseTransformPoint(point5);
        point6 = this.transform.InverseTransformPoint(point6);

        pointBuffer[idx++] = point1;
        pointBuffer[idx++] = point2;
        pointBuffer[idx++] = point3;
        pointBuffer[idx++] = point4;
        pointBuffer[idx++] = point5;
        pointBuffer[idx++] = point6;

        return idx;
    }    

    int SetCapusalPoints(CapsuleCollider capsule, Vector3[] pointBuffer, int idx)
    {
        Vector3 c = capsule.center;

        float radius = capsule.radius;
        float halfHeight = capsule.height * 0.5f;
        Vector3 point1, point2, point3, point4, point5, point6;
        switch (capsule.direction)
        {
            default:
            case (0):
                point1 = capsule.transform.TransformPoint(new Vector3(c.x+halfHeight, c.y+0f, c.z+0f));
                point2 = capsule.transform.TransformPoint(new Vector3(c.x-halfHeight, c.y+0f, c.z+0f));
                point3 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y+radius, c.z+0f));
                point4 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y-radius, c.z+0f));
                point5 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y+0f, c.z+radius));
                point6 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y+0f, c.z-radius));
                break;
            case (1):
                point1 = capsule.transform.TransformPoint(new Vector3(c.x+radius, c.y+0f, c.z+0f));
                point2 = capsule.transform.TransformPoint(new Vector3(c.x-radius, c.y+0f, c.z+0f));
                point3 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y+halfHeight, c.z+0f));
                point4 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y-halfHeight, c.z+0f));
                point5 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y+0f, c.z+radius));
                point6 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y+0f, c.z-radius));
                break;
            case (2):
                point1 = capsule.transform.TransformPoint(new Vector3(c.x+radius, c.y+0f, c.z+0f));
                point2 = capsule.transform.TransformPoint(new Vector3(c.x-radius, c.y+0f, c.z+0f));
                point3 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y+radius, c.z+0f));
                point4 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y-radius, c.z+0f));
                point5 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y+0f, c.z+halfHeight));
                point6 = capsule.transform.TransformPoint(new Vector3(c.x+0f, c.y+0f, c.z-halfHeight));
                break;
        }
        // transform from world space, into local space for COM 
        point1 = this.transform.InverseTransformPoint(point1);
        point2 = this.transform.InverseTransformPoint(point2);
        point3 = this.transform.InverseTransformPoint(point3);
        point4 = this.transform.InverseTransformPoint(point4);
        point5 = this.transform.InverseTransformPoint(point5);
        point6 = this.transform.InverseTransformPoint(point6);

        pointBuffer[idx++] = point1;
        pointBuffer[idx++] = point2;
        pointBuffer[idx++] = point3;
        pointBuffer[idx++] = point4;
        pointBuffer[idx++] = point5;
        pointBuffer[idx++] = point6;

        return idx;
    }
	Vector3 GetCenterOfMass(IEnumerable<ArticulationBody> bodies)
	{
		var centerOfMass = Vector3.zero;
		float totalMass = 0f;
		foreach (ArticulationBody ab in bodies)
		{
			centerOfMass += ab.worldCenterOfMass * ab.mass;
			totalMass += ab.mass;
		}
		centerOfMass /= totalMass;
		// centerOfMass -= _spawnableEnv.transform.position;
		return centerOfMass;        
	}    
	Vector3 GetCenterOfMass(IEnumerable<Rigidbody> bodies)
	{
		var centerOfMass = Vector3.zero;
		float totalMass = 0f;
		foreach (Rigidbody ab in bodies)
		{
			centerOfMass += ab.worldCenterOfMass * ab.mass;
			totalMass += ab.mass;
		}
		centerOfMass /= totalMass;
		// centerOfMass -= _spawnableEnv.transform.position;
		return centerOfMass;
	}    
    public void DrawPointDistancesFrom(DReConRewardStats target, int objIdex)
    {
        int start = 0;
        int end = Points.Length-1;
        if (objIdex >=0)
        {
            start = objIdex*6;
            end = (objIdex*6)+6;
        }
        for (int i = start; i < end; i++)
        {
            Gizmos.color = Color.white;
            var from = Points[i];
            var to = target.Points[i];
            var toTarget = target.Points[i];
            // transform to this object's world space
            from = this.transform.TransformPoint(from);
            to = this.transform.TransformPoint(to);
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(from, to);
            // transform to target's world space
            toTarget = target.transform.TransformPoint(toTarget);
            Gizmos.color = Color.white;
            Gizmos.DrawLine(from, toTarget);
            // show this objects velocity
            Vector3 velocity = PointVelocity[i];
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(from, velocity);
            // show targets velocity
            Vector3 velocityTarget = target.PointVelocity[i];
            Gizmos.color = Color.green;
            Gizmos.DrawRay(toTarget, velocityTarget);
        }
    }
}
