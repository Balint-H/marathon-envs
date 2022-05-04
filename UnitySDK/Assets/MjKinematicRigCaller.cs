using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MjKinematicRigCaller : MonoBehaviour
{
    [SerializeField]
    MjKinematicRig rig;

    private void FixedUpdate()
    {
        rig.TrackKinematics();
    }

}
