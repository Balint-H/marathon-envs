using DReCon;
using Kinematic;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class ThrowObservationSource : ObservationSource
{
    [SerializeField]
    DReConAgent agent;

    [SerializeField]
    Transform kinematicTransform;
    
    [SerializeField]
    Transform simulationTransform;

    private BodyChain kinChain;
    private BodyChain simChain;

    [SerializeField]
    Target target;

    public override int Size => 4;

    public override void FeedObservationsToSensor(VectorSensor sensor)
    {
        ReferenceFrame fKin = new ReferenceFrame(kinChain.RootForward, simChain.CenterOfMass);

        sensor.AddObservation(fKin.WorldToCharacter(target.transform.position));
        sensor.AddObservation(target.h);
    }

    public override void OnAgentInitialize()
    {
        kinChain = new BodyChain(kinematicTransform);
        simChain = new BodyChain(simulationTransform);
    }

    


}