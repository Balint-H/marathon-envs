using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LineStretcher : TrainingEventHandler
{
    // Start is called before the first frame update
    [SerializeField]
    Transform Origin;

    [SerializeField]
    Transform Insertion;

    LineRenderer renderer;

    public override EventHandler Handler => (object sender, EventArgs args) => Stretch();

    void Start()
    {
        renderer = GetComponent<LineRenderer>();
    }

    // Update is called once per frame
    void Stretch()
    {
        renderer.SetPositions(new Vector3[] { Origin.position, Insertion.position });
    }
}
