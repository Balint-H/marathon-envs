using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class ChainConverterEditorWindow : EditorWindow
{
    [MenuItem("GameObject/Convert Chain/Convert MuJoCo Chain to ArticulationBodies")]
    // Start is called before the first frame update
    public static void ConvertMjToArticulationBody()
    {
        //Selection.activeGameObject;
    }

    [MenuItem("GameObject/Convert Chain/Convert ArticulationBody Chain to MuJoCo")]
    // Start is called before the first frame update
    public static void ConvertArticulationBodyToMj()
    {

    }

    private static void AddArticulationBody(GameObject gameObject, float mass, Vector3 anchorPosition, float damping, Vector3[] jointAxes, DoFCharacteristics[] dofProps)
    {
        var artB = gameObject.AddComponent<ArticulationBody>();
        artB.mass = mass;
        artB.anchorPosition = anchorPosition;
        artB.jointFriction = damping;


    }

    struct DoFCharacteristics
    {
        public readonly float stiffness;
        public readonly float damping;
        public readonly float target;

        public DoFCharacteristics(float stiffness, float damping, float target)
        {
            this.stiffness = stiffness;
            this.damping = damping;
            this.target = target;
        }
    }
}
