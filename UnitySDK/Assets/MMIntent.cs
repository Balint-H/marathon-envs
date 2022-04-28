using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MMIntent : MonoBehaviour, IAnimationController
{
    [SerializeField]
    AutoInput landmarkGenerator;
    public Vector3 GetDesiredVelocity()
    {
        return landmarkGenerator.AnalogueDirection.ProjectTo3D();
    }

    public void OnAgentInitialize()
    {

    }

    public void OnReset()
    {

    }

}
