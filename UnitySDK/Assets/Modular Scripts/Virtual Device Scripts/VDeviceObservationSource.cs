using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Mujoco;

public class VDeviceObservationSource : ObservationSource
{
    [SerializeField]
    MjSiteVectorSensor shankACC;

    [SerializeField]
    MjSiteVectorSensor shankGYR;

    [SerializeField]
    MjSiteVectorSensor footACC;

    [SerializeField]
    MjSiteVectorSensor footGYR;

    [SerializeField]
    Transform target;

    [SerializeField]
    Transform result;

    [SerializeField]
    MjActuator actuator;

    public override int Size => 25;

    public override void FeedObservationsToSensor(VectorSensor sensor)
    {
        sensor.AddObservation(shankACC.SensorReading);
        sensor.AddObservation(shankGYR.SensorReading);
        sensor.AddObservation(footACC.SensorReading);
        sensor.AddObservation(footGYR.SensorReading);
        sensor.AddObservation(result.position);
        sensor.AddObservation(result.localRotation.eulerAngles);
        sensor.AddObservation(target.position);
        sensor.AddObservation(target.localRotation.eulerAngles);
        sensor.AddObservation(actuator.LengthInRad());
    }

    public override void OnAgentInitialize()
    {
        
    }

}
