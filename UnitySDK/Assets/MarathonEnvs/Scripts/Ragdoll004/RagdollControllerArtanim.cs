﻿using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RagdollControllerArtanim : MonoBehaviour

    //this class does exactly the symetrical of MocapControllerArtanim: it maps animations from a ragdoll to a rigged character
{

    [SerializeField]
	ArticulationBody _articulationBodyRoot;

    //to generate an environment automatically from a rigged character and an animation (see folder ROM-extraction)
    public ArticulationBody ArticulationBodyRoot { set => _articulationBodyRoot = value; 
		get =>  _articulationBodyRoot; }
  

    private List<ArticulationBody> _articulationbodies = null;

	private List<Transform> _targetPoseTransforms = null;

    private List<MappingOffset> _offsetsRB2targetPoseTransforms = null;


	private List<MappingOffset> _offsetsSource2RB = null;


	[Space(20)]



	//not used in 
	[SerializeField]
	float _debugDistance= 0.0f;


	[SerializeField]
	bool _isGeneratedProcedurally;

	public bool IsGeneratedProcedurally { set => _isGeneratedProcedurally = value; }


	/*

    [SerializeField]
    bool SetUpConstraintsFromData= false;

    [SerializeField]
    ROMinfoCollector data4constraints;
    */


	//we use this to bypass the physical character, using only the ragdoll with rigidbodies, instead of the ragdoll with articulation Joints
	//This is INCOMPATIBLE with SetUPConstraintsFromData
	[SerializeField]
	bool _debugWithRigidBody = false;
	[SerializeField]
	Rigidbody _rigidbodyRoot;


	private List<Rigidbody> _rigidbodies = null;




    // Start is called before the first frame update
    public void Start()
    {
        //if (SetUpConstraintsFromData) {

        //    Debug.LogError("this automatic method to set up the constraints does NOT work yet");
        //    SetConstraints(data4constraints);
        //}
    }



	//this one is for the case where everything is generated procedurally
	void  SetOffsetRB2targetPoseInProceduralWorld() {

		_targetPoseTransforms = GetComponentsInChildren<Transform>().ToList();

		_offsetsRB2targetPoseTransforms = new List<MappingOffset>();

		_articulationbodies = _articulationBodyRoot.GetComponentsInChildren<ArticulationBody>().ToList();


		foreach (Transform t in _targetPoseTransforms) {

			string abname = "articulation:" + t.name;
			ArticulationBody ab = _articulationbodies.First(x => x.name == abname);
			


			Quaternion qoffset = ab.transform.rotation * Quaternion.Inverse(t.rotation);
			MappingOffset r = new MappingOffset(t, ab, Quaternion.Inverse(qoffset));
			if (ab.isRoot)
			{
				r.SetAsRoot(true, _debugDistance);

			}

			_offsetsRB2targetPoseTransforms.Add(r);

		}



	}






	MappingOffset SetOffsetRB2targetPose(string rbname, string tname)
	{
		//here we set up:
		// a. the transform of the rigged character output
		// b. the rigidbody of the physical character
		// c. the offset calculated between the rigged character INPUT, and the rigidbody


		if (_targetPoseTransforms == null)
		{
			_targetPoseTransforms = GetComponentsInChildren<Transform>().ToList();
			Debug.Log("the number of transforms  intarget pose is: " + _targetPoseTransforms.Count);

		}


		if (_offsetsRB2targetPoseTransforms == null)
		{
			_offsetsRB2targetPoseTransforms = new List<MappingOffset>();

		}

		if (_articulationbodies == null)
		{
            if (_debugWithRigidBody) { 
			_rigidbodies = _rigidbodyRoot.GetComponentsInChildren<Rigidbody>().ToList();
            }
            else { 
				_articulationbodies = _articulationBodyRoot.GetComponentsInChildren<ArticulationBody>().ToList();
			}
		}


		Transform rb;
		if (_debugWithRigidBody)
		{
			rb = _rigidbodies.First(x => x.name == rbname).transform;
		}else
		{
            rb = null;
          
            try
            {
                rb = _articulationbodies.First(x => x.name == rbname).transform;
                if (rb == null)
                {
                    Debug.LogError("no rigidbody with name " + rbname);

                }

            }
            catch (Exception e) {

                Debug.LogError("problem with finding rigidbody with a name like: " + rbname);

            }




          
		}

		Transform t = null;
		try
		{

			t = _targetPoseTransforms.First(x => x.name == tname);

		}
		catch (Exception e)
		{
			Debug.LogError("no bone transform with name in target pose" + tname);

		}

		Transform tref = null;
		try
		{

			tref = _targetPoseTransforms.First(x => x.name == tname);

		}
		catch (Exception e)
		{
			Debug.LogError("no bone transform with name in input pose " + tname);

		}

		//from refPose to Physical body:
		//q_{physical_body} = q_{offset} * q_{refPose}
		//q_{offset} = q_{physical_body} * Quaternion.Inverse(q_{refPose})

		//Quaternion qoffset = rb.transform.localRotation * Quaternion.Inverse(tref.localRotation);

		Quaternion qoffset = rb.transform.rotation * Quaternion.Inverse(tref.rotation);


		//from physical body to targetPose:
		//q_{target_pose} = q_{offset2} * q_{physical_body}
		//q_{offset2} = Quaternion.Inverse(q_{offset})

		MappingOffset r;
		if (_debugWithRigidBody)
		{
			Rigidbody myrb = rb.GetComponent<Rigidbody>();
			r = new MappingOffset(t, myrb, Quaternion.Inverse(qoffset));
			r.SetAsRagdollcontrollerDebug(_debugWithRigidBody);
		}
		else 
		{
			ArticulationBody myrb = rb.GetComponent<ArticulationBody>();
			r = new MappingOffset(t, myrb, Quaternion.Inverse(qoffset));
		}




		_offsetsRB2targetPoseTransforms.Add(r);
		return r;
	}


	void MimicPhysicalChar()
	{

		try
		{
			foreach (MappingOffset o in _offsetsRB2targetPoseTransforms)
			{
				o.UpdateRotation();
			}
		}
		catch (Exception e)
		{
			Debug.Log("not calibrated yet...");

		}


	}

    private void FixedUpdate()
    {
		MimicAnimationArtanim();
    }
    void MimicAnimationArtanim()
	{


        if (_offsetsRB2targetPoseTransforms == null)
		{

            if (_isGeneratedProcedurally)
            {

				SetOffsetRB2targetPoseInProceduralWorld();
            }
            else { 

				MappingOffset o = SetOffsetRB2targetPose("articulation:Hips", "mixamorig:Hips");
				o.SetAsRoot(true, _debugDistance);
				SetOffsetRB2targetPose("articulation:Spine", "mixamorig:Spine");
				SetOffsetRB2targetPose("articulation:Spine1", "mixamorig:Spine1");
				SetOffsetRB2targetPose("articulation:Spine2", "mixamorig:Spine2");
				SetOffsetRB2targetPose("articulation:Neck", "mixamorig:Neck");
				SetOffsetRB2targetPose("head", "mixamorig:Head");

            

				SetOffsetRB2targetPose("articulation:LeftShoulder", "mixamorig:LeftShoulder");

				SetOffsetRB2targetPose("articulation:LeftArm", "mixamorig:LeftArm");
				SetOffsetRB2targetPose("articulation:LeftForeArm", "mixamorig:LeftForeArm");

				//	SetOffsetRB2targetPose("left_hand", "mixamorig:LeftHand");
				// hands do not have rigidbodies




				SetOffsetRB2targetPose("articulation:RightShoulder", "mixamorig:RightShoulder");

				SetOffsetRB2targetPose("articulation:RightArm", "mixamorig:RightArm");
				SetOffsetRB2targetPose("articulation:RightForeArm", "mixamorig:RightForeArm");
			//	SetOffsetRB2targetPose("right_hand", "mixamorig:RightHand");


				SetOffsetRB2targetPose("articulation:LeftUpLeg", "mixamorig:LeftUpLeg");


				//			SetOffsetRB2targetPose("left_shin", "mixamorig:LeftLeg");
				SetOffsetRB2targetPose("articulation:LeftLeg", "mixamorig:LeftLeg");
				SetOffsetRB2targetPose("articulation:LeftFoot", "mixamorig:LeftFoot");
				SetOffsetRB2targetPose("articulation:LeftToeBase", "mixamorig:LeftToeBase");
				//	SetOffsetRB2targetPose("right_left_foot", "mixamorig:LeftToeBase");


				SetOffsetRB2targetPose("articulation:RightUpLeg", "mixamorig:RightUpLeg");
				//SetOffsetRB2targetPose("right_shin", "mixamorig:RightLeg");
				SetOffsetRB2targetPose("articulation:RightLeg", "mixamorig:RightLeg");

				SetOffsetRB2targetPose("articulation:RightFoot", "mixamorig:RightFoot");
				SetOffsetRB2targetPose("articulation:RightToeBase", "mixamorig:RightToeBase");
					//	SetOffsetRB2targetPose("left_right_foot", "mixamorig:RightToeBase");

			}

		}
		else
		{
			MimicPhysicalChar();


		}



	}


}
