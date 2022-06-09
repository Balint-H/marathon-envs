using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class IntentDrawer : MonoBehaviour
{
    // Start is called before the first frame update
    [SerializeField]
    LineRenderer line;

    [SerializeField]
    AutoInput input;

    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        line.SetPositions(input.CurrentTrajectory.Select(v=>v.Horizontal3D()+input.fauxRootInWorld.position.Horizontal3D()).ToArray());
    }
}
