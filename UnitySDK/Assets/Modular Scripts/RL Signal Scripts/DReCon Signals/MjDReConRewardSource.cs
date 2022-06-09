using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System;
using Kinematic;
using DReCon;
using Mujoco;

public class MjDReConRewardSource : DReConRewardSource
{

    unsafe public override void OnAgentInitialize()
    {
        //MjScene.Instance.CreateScene();
        AddColliders(simulationTransform);
        AddColliders(kinematicTransform);
        base.OnAgentInitialize();
        RemoveColliders(simulationTransform);
        RemoveColliders(kinematicTransform);
    }

    public void AddColliders(Transform rootTransform)
    {
        foreach (var body in rootTransform.GetComponentsInChildren<MjBody>())
        {

            var colObject = new GameObject();
            colObject.transform.SetParent(body.transform);
            colObject.transform.SetPositionAndRotation(body.transform.TransformPoint(body.GetLocalCenterOfMass()), body.transform.rotation * body.GetLocalCenterOfMassRotation());


            var box = colObject.AddComponent(typeof(BoxCollider)) as BoxCollider;

            var diagInertia = body.GetInertia();
            var mass = body.GetMass();

            box.size = new Vector3(Mathf.Sqrt((diagInertia[1] + diagInertia[2] - diagInertia[0]) / mass * 6.0f),
                                   Mathf.Sqrt((diagInertia[0] + diagInertia[2] - diagInertia[1]) / mass * 6.0f),
                                   Mathf.Sqrt((diagInertia[0] + diagInertia[1] - diagInertia[2]) / mass * 6.0f));

        }
    }

    public void RemoveColliders(Transform rootTransform)
    {
        foreach (var geom in rootTransform.GetComponentsInChildren<MjGeom>())
        {

            if (geom.transform.parent.GetComponent<MjBody>() == null)
            {
                continue;
            }

            var collider = geom.gameObject.GetComponent<Collider>();
            Destroy(collider);

        }
    }

}

    
