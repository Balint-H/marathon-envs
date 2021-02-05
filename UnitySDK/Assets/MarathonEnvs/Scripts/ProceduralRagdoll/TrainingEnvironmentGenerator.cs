using System.Collections;
using System.Collections.Generic;
using System;

using UnityEngine;


using Unity.MLAgents.Policies;
using Unity.MLAgents;
using System.Linq;

public class TrainingEnvironmentGenerator : MonoBehaviour
{



    [Header("The animated character:")]


    [SerializeField]
    Animator characterReference;

    [SerializeField]
    Transform characterReferenceHead;

    [SerializeField]
    Transform characterReferenceRoot;


    [Tooltip("fingers will be excluded from physics-learning")]
    [SerializeField]
    Transform[] characterReferenceHands;

    //we assume here is the end-effector, but feet have an articulaiton (sensors will be placed on these and their immediate parents)
    //strategy to be checked: if for a quadruped we add the 4 feet here, does it work?
    [Tooltip("same as above but not taking into account fingers. Put the last joint")]
    [SerializeField]
    Transform[] characterReferenceFeet;


    [Header("How we want the generated assets stored:")]

    [SerializeField]
    string AgentName;

    [SerializeField]
    string TrainingEnvName;



    [Header("Configuration options:")]
    [SerializeField]
    string LearningConfigName;

    [Range(0, 359)]
    public int MinROMNeededForJoint = 0;


    [Tooltip("body mass in grams/ml")]
    [SerializeField]
    float massdensity = 1.01f;


    //[SerializeField]
    //ROMparserSwingTwist ROMparser;

    [SerializeField]
    public RangeOfMotion004 info2store;


    [Header("Prefabs to generate training environment:")]
    [SerializeField]
    ManyWorlds.SpawnableEnv referenceSpawnableEnvironment;

    [SerializeField]
    Material trainerMaterial;

    [SerializeField]
    PhysicMaterial colliderMaterial;



    //things generated procedurally that we store to configure after the generation of the environment:
    [HideInInspector]
    [SerializeField]
    Animator character4training;

    [HideInInspector]
    [SerializeField]
    Animator character4synthesis;

    [HideInInspector][SerializeField]
    ManyWorlds.SpawnableEnv _outcome;


    


    [HideInInspector]
    [SerializeField]
    List<ArticulationBody> articulatedJoints;

    [HideInInspector]
    [SerializeField]
    RagDoll004 muscleteam;

    public ManyWorlds.SpawnableEnv Outcome{ get { return _outcome; } }


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }


    public void GenerateTrainingEnv() {

        character4training = Instantiate(characterReference.gameObject).GetComponent<Animator>();
        character4training.gameObject.SetActive(true);

        //we assume those are already there (i.e., an animated character with a controller) 
        //character4training.gameObject.AddComponent<CharacterController>();
        //MocapAnimatorController mac = character4training.gameObject.AddComponent<MocapAnimatorController>();
        //mac.IsGeneratedProcedurally = true;


        MapAnimationController2RagdollRef mca =character4training.gameObject.AddComponent<MapAnimationController2RagdollRef>();
        //mca.IsGeneratedProcedurally = true;

        character4training.gameObject.AddComponent<TrackBodyStatesInWorldSpace>();
        character4training.name = "Source:" + AgentName;

        SkinnedMeshRenderer[] renderers = character4training.GetComponentsInChildren<SkinnedMeshRenderer>();
        foreach (SkinnedMeshRenderer r in renderers) {

            Material[] mats = r.sharedMaterials;
            for (int i =0; i < mats.Length; i++) {
                mats[i] = trainerMaterial;
            }
            
        }

        



        character4synthesis = Instantiate(characterReference.gameObject).GetComponent<Animator>();
        character4synthesis.gameObject.SetActive(true);

        character4synthesis.name = "Result:" + AgentName ;


        //we remove everything except the transform
        Component[] list = character4synthesis.GetComponents(typeof(Component));
        foreach (Component c in list)
        {

            if (c is Transform || c is Animator || c is CharacterController)
            {
            }
            else
            {
                DestroyImmediate(c);

            }

        }

        character4synthesis.GetComponent<Animator>().runtimeAnimatorController = null;




        RagdollControllerArtanim rca = character4synthesis.gameObject.AddComponent<RagdollControllerArtanim>();
        rca.IsGeneratedProcedurally = true;

        _outcome = Instantiate(referenceSpawnableEnvironment).GetComponent<ManyWorlds.SpawnableEnv>();
        _outcome.name = TrainingEnvName;


        RagDollAgent ragdollMarathon = generateRagDollFromAnimatedSource(rca, _outcome);




        MocapAnimatorController004 animationcontroller = character4training.GetComponent<MocapAnimatorController004>();

        if(animationcontroller != null)
        //we make sure they are in the same layers:
        {

           int  _layerMask = 1 << ragdollMarathon.gameObject.layer;
            _layerMask |= 1 << character4training.gameObject.layer;
            _layerMask = ~(_layerMask);

            character4training.GetComponent<MocapAnimatorController004>()._layerMask = _layerMask;
            //TODO: this will only work if we have a MocapAnimatorController004. We should REMOVE this dependency

        }





        addTrainingParameters(rca, ragdollMarathon);


        //UNITY BUG
        //This below seems to make it crash. My guess is that:
        /*
        ArticulationBody is a normal component, but it also does some odd thing: it affects the physical simluation, since it is a rigidBody plus an articulatedJoint. The way it is defined is that it has the properties of a rigidBody, but it ALSO uses the hierarchy of transforms  to define a chain of articulationJoints, with their rotation constraints, etc. Most notably, the ArticulationBody that is highest in the root gets assigned a property automatically, "isRoot", which means it's physics constraints are different.My guess is that when you change the hierarchy in the editor, at some point in the future the chain of ArticulationBody recalculates who is the root, and changes its properties. However since this relates to physics, it is not done in the same thread.
If the script doing this is short, it works because this is finished before the update of the ArticulationBody chain is triggered. But when I add more functionality, the script lasts longer, and then it crashes. This is why I kept getting those Physx errors, and why it kept happening in a not-so-reliable way, because we do not have any way to know when this recalculation is done.The fact that ArticulationBody is a fairly recent addition to Unity also makes me suspect they did not debug it properly.The solution seems to be to do all the setup while having the game objects that have articulationBody components with no hierarchy changes, and also having the rootgameobject inactive. When I do this,  I am guessing it does not trigger the update of the ArticulationBody chain.

        I seem to have a reliable crash:

            1.if I use ragdollroot.gameObject.SetActive(true) at the end of my configuration script, it crashes.
            2.if I comment that line, it does not.
            3.if I set it to active manually, through the editor, after running the script with that line commented, it works.

        */
        //ragdoll4training.gameObject.SetActive(true);


        _outcome.GetComponent<RenderingOptions>().ragdollcontroller = ragdollMarathon.gameObject;


        character4training.transform.SetParent(_outcome.transform);
        _outcome.GetComponent<RenderingOptions>().movementsource = character4training.gameObject;

        character4synthesis.transform.SetParent(_outcome.transform);


        //ragdoll4training.gameObject.SetActive(true);




    }


    /*
    public void activateRagdoll() {

        RagDollAgent ragdoll4training = _outcome.GetComponentInChildren<RagDollAgent>(true);
        if(ragdoll4training!=null)
            ragdoll4training.gameObject.SetActive(true);


    }
    */



    public void GenerateRagdollForMocap() {


        MapAnimationController2RagdollRef mca = character4training.gameObject.GetComponent<MapAnimationController2RagdollRef>();
        mca.DynamicallyCreateRagdollForMocap();

    }




RagDollAgent  generateRagDollFromAnimatedSource( RagdollControllerArtanim target, ManyWorlds.SpawnableEnv trainingenv) {

   
        GameObject temp = GameObject.Instantiate(target.gameObject);
        

        //we remove everything we do not need:
        SkinnedMeshRenderer[] renderers = temp.GetComponentsInChildren<SkinnedMeshRenderer>();
        foreach (SkinnedMeshRenderer rend in renderers)
        {

            if(rend.gameObject != null)
                DestroyImmediate(rend.gameObject);

        }


        //we remove everything except the transform
        Component[] list = temp.GetComponents(typeof(Component));
        foreach (Component c in list) {

            if (c is Transform)
            {
            }
            else {
                DestroyImmediate(c);

            }

        }
        

        temp.name = "Ragdoll:" + AgentName ;
        muscleteam=  temp.AddComponent<RagDoll004>();
        temp.transform.position = target.transform.position;
        temp.transform.rotation = target.transform.rotation;

        //it might be important to have this BEFORE we add ArticulationBody members, doing it afterwards, and having it active, makes the entire thing crash 
        temp.transform.parent = trainingenv.transform;
        temp.gameObject.SetActive(false);


       
        Transform[] pack = temp.GetComponentsInChildren<Transform>();

        Transform root = pack.First<Transform>(x => x.name == characterReferenceRoot.name);



        Transform[] joints = root.transform.GetComponentsInChildren<Transform>();



        List<Transform> listofjoints = new List<Transform>(joints);
        //we drop the sons of the limbs (to avoid including fingers in the following procedural steps)
        foreach (Transform t in characterReferenceHands) {
            string limbname = t.name;// + "(Clone)";
            Transform limb = joints.First<Transform>(x => x.name == limbname);




            List<Transform> childstodelete = new List<Transform>(limb.GetComponentsInChildren<Transform>());
            childstodelete.Remove(limb);
            foreach (Transform t2 in childstodelete)
            {
                listofjoints.Remove(t2);
                t2.DetachChildren();//otherwise, it tries to destroy the children later, and fails.
            }
            foreach (Transform t2 in childstodelete)
            {
                DestroyImmediate(t2.gameObject);
            }




        }
        joints = listofjoints.ToArray();
        articulatedJoints = new List<ArticulationBody>();


        foreach (Transform j in joints) {
            ArticulationBody ab = j.gameObject.AddComponent<ArticulationBody>();
            ab.anchorRotation = Quaternion.identity;

            ab.mass = 0.1f;



            articulatedJoints.Add(ab);

          

            //note: probably not needed
            string namebase = j.name.Replace("(Clone)", "");


            j.name = "articulation:" + namebase;

            //we only add a collider if it has a parent:
            Transform dad = ab.transform.parent;

            ArticulationBody articulatedDad = null;

            if (dad != null)
            {
                articulatedDad = dad.GetComponent<ArticulationBody>();
            }

            if(articulatedDad != null) { 

                GameObject go = new GameObject();
                go.transform.parent = dad;

                //dad.gameObject

                go.name = "collider:" + namebase;

                CapsuleCollider c = go.AddComponent<CapsuleCollider>();
                c.material = colliderMaterial;

                c.height = Vector3.Distance(dad.position, j.transform.position);



                //ugly but it seems to work.
                Vector3 direction = (dad.position - j.transform.position).normalized;
                float[] directionarray = new float[3] { Mathf.Abs(direction.x), Mathf.Abs(direction.y), Mathf.Abs(direction.z) };
                float maxdir = Mathf.Max(directionarray);

                List<float> directionlist = new List<float>(directionarray);
                c.direction = directionlist.IndexOf(maxdir);

                 

                c.center = (dad.position + j.transform.position) / 2.0f;
                c.radius = c.height / 5;



                // Rigidbody rb = go.AddComponent<Rigidbody>();
                ArticulationBody rb = go.AddComponent<ArticulationBody>();
                rb.jointType = ArticulationJointType.FixedJoint;
                rb.mass = massdensity *  Mathf.PI * c.radius *c.radius *c.height * Mathf.Pow(10,2); //we are aproximating as a cylinder, assuming it wants it in kg



                HandleOverlap ho = go.AddComponent<HandleOverlap>();

                //ho.Parent = dad.gameObject;
                ho.Parent = root.gameObject;




            }
                    
        
        }




        //I add reference to the ragdoll, the articulationBodyRoot:
        target.ArticulationBodyRoot = root.GetComponent<ArticulationBody>();



        addSensorsInFeet(root);







        //at this stage, every single articulatedBody is root. Check it out with the script below
        /*
        foreach (Transform j in joints)
        {
            ArticulationBody ab = j.transform.GetComponent<ArticulationBody>();
            if (ab.isRoot)
            {
                Debug.Log(ab.name + "is root ");
            }
        }
        */




        RagDollAgent _ragdoll4training = temp.AddComponent<RagDollAgent>();
        //      _ragdoll4training.transform.parent = trainingenv.transform;
        //_ragdoll4training.transform.SetParent(trainingenv.transform);

    

        return _ragdoll4training;

    }


    void addSensorsInFeet(Transform root) {

        //I add the sensors in the feet:
        Transform[] pack2 = root.GetComponentsInChildren<Transform>();
        foreach (Transform t in characterReferenceFeet)
        {

            Transform foot = pack2.First<Transform>(x => x.name == "articulation:" + t.name);

            GameObject sensorL = new GameObject();
            SphereCollider sphL = sensorL.AddComponent<SphereCollider>();
            sphL.radius = 0.03f;
            sensorL.AddComponent<SensorBehavior>();
            sensorL.AddComponent<HandleOverlap>();


            //TODO: we are assuming it faces towards the +z axis. It could be done more generic looking into the direction of the collider
            sensorL.transform.parent = foot;
            sensorL.transform.localPosition = new Vector3(-0.02f, 0, 0);
            sensorL.name = foot.name + "sensor_L";

            GameObject sensorR = GameObject.Instantiate(sensorL);
            sensorR.transform.parent = foot;
            sensorR.transform.localPosition = new Vector3(0.02f, 0, 0);
            sensorR.name = foot.name + "sensor_R";

            //we add another sensor for the toe:
            GameObject sensorT = GameObject.Instantiate(sensorL);
            sensorT.transform.parent = foot.parent;
            sensorT.transform.localPosition = new Vector3(0.0f, -0.01f, -0.04f);
            sensorT.name = foot.name + "sensor_T";




        }




    }

    //it needs to go after adding ragdollAgent or it automatically ads an Agent, which generates conflict
    void addTrainingParameters(RagdollControllerArtanim target, RagDollAgent temp) {



        BehaviorParameters bp = temp.gameObject.GetComponent<BehaviorParameters>();

        bp.BehaviorName = LearningConfigName;



        DecisionRequester dr =temp.gameObject.AddComponent<DecisionRequester>();
        dr.DecisionPeriod = 2;
        dr.TakeActionsBetweenDecisions = false;



        DReConRewards dcrew = temp.gameObject.AddComponent<DReConRewards>();
        dcrew.headname = "articulation:" + characterReferenceHead.name;
        dcrew.targetedRootName = "articulation:" + characterReferenceRoot.name; //it should be it's son, but let's see



        temp.MaxStep = 2000;
        temp.FixedDeltaTime = 0.0125f;
        temp.RequestCamera = true;


        temp.gameObject.AddComponent<SensorObservations>();
        DReConObservations dcobs = temp.gameObject.AddComponent<DReConObservations>();
        dcobs.targetedRootName = characterReferenceRoot.name;  // target.ArticulationBodyRoot.name;

        dcobs.BodyPartsToTrack = new List<string>();

        //TODO: this could be EVERY joint, if we follow NVIDIA's approach to a universal physics controller. Meanwhile...
        dcobs.BodyPartsToTrack.Add("articulation:" + characterReferenceRoot.name);
        dcobs.BodyPartsToTrack.Add("articulation:" + characterReferenceHead.name);
       
        dcobs.targetedRootName = "articulation:" + characterReferenceRoot.name; //it should be it's son, but let's see



        foreach (Transform t in characterReferenceFeet)
        {
            dcobs.BodyPartsToTrack.Add("articulation:" + t.name);

        }






        ApplyRangeOfMotion004 rom = temp.gameObject.AddComponent<ApplyRangeOfMotion004>();
        rom.RangeOfMotion2Store = info2store;
        //rom.ApplyROMInGamePlay = true;


    }






    public void GenerateRangeOfMotionParser() {

        
        ROMparserSwingTwist rom = gameObject.GetComponentInChildren<ROMparserSwingTwist>();
        if (rom == null) {
            GameObject go = new GameObject();
            go.name = "ROM-parser";
            go.transform.parent = gameObject.transform;
            rom = go.AddComponent<ROMparserSwingTwist>();



        }


        rom.info2store = info2store;
        rom.theAnimator = characterReference;
        rom.skeletonRoot = characterReferenceRoot;

        rom.targetRagdollRoot = character4synthesis.GetComponent<RagdollControllerArtanim>().ArticulationBodyRoot;

        rom.trainingEnv = _outcome;

    }

    void generateMuscles() {

        //muscles

        foreach(ArticulationBody ab in articulatedJoints) { 

            RagDoll004.MusclePower muscle = new RagDoll004.MusclePower();
            muscle.PowerVector = new Vector3(40, 40, 40);


            muscle.Muscle = ab.name;

            if (muscleteam.MusclePowers == null)
                muscleteam.MusclePowers = new List<RagDoll004.MusclePower>();

            muscleteam.MusclePowers.Add(muscle);

        }


    }






    public void Prepare4RangeOfMotionParsing()
    {
        _outcome.gameObject.SetActive(false);
        characterReference.gameObject.SetActive(true);

    }


    public void Prepare4EnvironmentStorage()
    {

        characterReference.gameObject.SetActive(false);

        _outcome.gameObject.SetActive(true);
      
       // RagDollAgent ra = _outcome.GetComponentInChildren<RagDollAgent>(true);
       // ra.gameObject.SetActive(true);





    }

    public void ApplyROMasConstraintsAndConfigure() {

        ApplyRangeOfMotion004 ROMonRagdoll = Outcome.GetComponentInChildren<ApplyRangeOfMotion004>(true);
        ROMonRagdoll.MinROMNeededForJoint = MinROMNeededForJoint;
        ROMonRagdoll.ConfigureTrainingForRagdoll();

        
        generateMuscles();

        ROMonRagdoll.GetComponent<DecisionRequester>().DecisionPeriod = 2;





    }




}