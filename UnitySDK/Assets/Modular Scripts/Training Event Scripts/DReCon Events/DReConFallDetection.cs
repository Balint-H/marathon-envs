using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DReCon
{
    public class DReConFallDetection : TrainingEvent
    {
        [SerializeField]
        private Transform KinematicHead;

        [SerializeField]
        private Transform SimulationHead;

        [SerializeField]
        private bool useHeigthOnly;

        [SerializeField]
        float maxDistance;

        private void Update()
        {
            if ( useHeigthOnly?  (KinematicHead.position - SimulationHead.position).magnitude > maxDistance : Mathf.Abs(KinematicHead.position.y - SimulationHead.position.y) > maxDistance ) OnTrainingEvent(EventArgs.Empty);
        }
    }
}