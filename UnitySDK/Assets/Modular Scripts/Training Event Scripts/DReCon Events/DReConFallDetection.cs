using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DReCon
{
    public class DReConFallDetection : TrainingEvent
    {
        [SerializeField]
        DReConAgent agent;

        [SerializeField]
        private Transform KinematicHead;

        [SerializeField]
        private Transform SimulationHead;

        [SerializeField]
        private bool useHeightOnly;

        [SerializeField]
        float maxDistance;

        public override void SubscribeHandler(EventHandler subscriber)
        {
            agent.onActionHandler += CheckFall;
            base.SubscribeHandler(subscriber);
        }

        private void CheckFall(object sender, AgentEventArgs args)
        {
            if ( useHeightOnly?  Mathf.Abs(KinematicHead.position.y - SimulationHead.position.y) > maxDistance : (KinematicHead.position - SimulationHead.position).magnitude > maxDistance ) OnTrainingEvent(EventArgs.Empty);
        }
    }
}