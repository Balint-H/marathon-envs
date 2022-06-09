#if UNITY_EDITOR
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using System.Linq;
using UnityEditor.SceneManagement;

public class PrintHierarchyHandler : TrainingEventHandler
{
    private List<MjComponent> _orderedComponents;

    public override EventHandler Handler => (object sender, EventArgs e) => 
    {
        var hierarchyRoots = StageUtility.GetCurrentStageHandle().FindComponentsOfType<MjComponent>()
        .Where(component => MjHierarchyTool.FindParentComponent(component) == null)
        .Select(component => component.transform)
        .Distinct();
        _orderedComponents = new List<MjComponent>();
        foreach (var root in hierarchyRoots)
        {
            _orderedComponents.AddRange(MjHierarchyTool.LinearizeHierarchyBFS(root));
        }
        print(string.Join(',', _orderedComponents.Select(c => c.MujocoName)));
    };

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
#endif