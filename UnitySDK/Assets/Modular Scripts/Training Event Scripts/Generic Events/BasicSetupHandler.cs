using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Teleportable;
using System.Linq;
using Mujoco;

public class BasicSetupHandler : DelayableEventHandler
{
	// The pelvis/animation root may not be the same transform. So to reset to the correct height we have to keep them separate
	// Assuming root position not baked into animation, but applied to gameobject.
	[SerializeField]
	Transform referenceAnimationParent;

	// Should be either the same or child transform of above
	[SerializeField]
	Transform referenceAnimationRoot;

    [SerializeField]
    Transform kineticRagdollRoot;

	[SerializeField]
	GameObject kinematicRigObject;

	IKinematicReference kinematicRig;

	[SerializeField]
	Vector3 resetOrigin;

	[SerializeField]
	bool shouldResetRotation;

	[SerializeField]
	bool shouldResetPosition = true;

	Quaternion resetRotation;

	IResettable kineticChainToReset;

    public override EventHandler Handler => HandleSetup;

    private void Awake()
    {
		kineticChainToReset = kineticRagdollRoot.GetComponentInChildren<ArticulationBody>()!=null? new ResettableArticulationBody(kineticRagdollRoot.GetComponentsInChildren<ArticulationBody>()) : new ResettableMjBody(kineticRagdollRoot.GetComponentInChildren<MjBody>());
		kinematicRig = kinematicRigObject.GetComponent<IKinematicReference>();
		resetOrigin = referenceAnimationParent.position;
		resetRotation = referenceAnimationParent.rotation;
	}

    public void HandleSetup(object sender, EventArgs eventArgs)
    {
		if (IsWaiting) return;

		//First we move the animation back to the start 

		//Debug.Log("Resetting Animation Parent!");
		if (shouldResetPosition) referenceAnimationParent.position = resetOrigin;

		if (shouldResetRotation) referenceAnimationParent.rotation = resetRotation;
		kinematicRig.TeleportRoot(referenceAnimationRoot.position, referenceAnimationRoot.rotation);

        if (framesToWait > 0)
        {
            StartCoroutine(DelayedExecution(sender, eventArgs));
            return;
        }

		//Then we move the ragdoll as well, still in different joint orientations, but overlapping roots.
		if (shouldResetPosition) kineticChainToReset.TeleportRoot(referenceAnimationRoot.position, referenceAnimationRoot.rotation);

        //We copy the rotations, velocities and angular velocities from the kinematic reference (which has the "same" pose as the animation).
        kineticChainToReset.CopyKinematicsFrom(kinematicRig);

		//We teleport the kinematic reference as well, so velocities are not tracked in the move. Since we don't need to change rotation we use to position only version.
		if (shouldResetPosition)  kinematicRig.TeleportRoot(referenceAnimationRoot.position, referenceAnimationRoot.rotation);
    }

	protected override IEnumerator DelayedExecution(object sender, EventArgs eventArgs)
    {
		IsWaiting = true;
		yield return WaitFrames();
		//Debug.Log("Teleporting Root!");
		//Then we move the ragdoll as well, still in different joint orientations, but overlapping roots.
		kineticChainToReset.TeleportRoot(referenceAnimationRoot.position, referenceAnimationRoot.rotation);


		//Debug.Log("Copying Kinematics!");
		//We copy the rotations, velocities and angular velocities from the kinematic reference (which has the "same" pose as the animation).
		kineticChainToReset.CopyKinematicsFrom(kinematicRig);

		//Debug.Log("Teleporting Kinematic Root just in case!");
		kinematicRig.TeleportRoot(referenceAnimationRoot.position, referenceAnimationRoot.rotation);
		IsWaiting = false;

	}

	//As I can see this handler to be extended to chains other then Articulationbody ones, here's a WIP interface
    private interface IResettable
    {
        public void TeleportRoot(Vector3 position, Quaternion rotation);
        public void CopyKinematicsFrom(IKinematicReference reference);

    }

    private class ResettableArticulationBody : IResettable
    {
        IEnumerable<ArticulationBody> articulationBodies;


		/// <param name="articulationBodies">Should be sorted according to the reference KinematicRig</param>
        public ResettableArticulationBody(IEnumerable<ArticulationBody> articulationBodies)
        {
            this.articulationBodies = articulationBodies;
        }

        public void CopyKinematicsFrom(IKinematicReference referenceGeneralRig)
        {
			//Set root kinematics
			KinematicRig referenceRig = referenceGeneralRig as KinematicRig; // Right now cant work with any other IKinematicReference, defeating the purpose somewhat... The groundwork is layed down already, extending IKinematic with local ref frame values
			ArticulationBody root = articulationBodies.First(ab => ab.isRoot);
			root.angularVelocity = referenceRig.RagdollAngularVelocities[0];
			root.velocity = referenceRig.RagdollLinVelocities[0];

			foreach ((ArticulationBody ab, Rigidbody sourceRigidbody) in articulationBodies.Skip(1).Zip(referenceRig.Rigidbodies.Skip(1), Tuple.Create))
			{
				Quaternion targetLocalRotation = sourceRigidbody.transform.localRotation;
				Vector3 targetLocalAngularVelocity = sourceRigidbody.transform.InverseTransformDirection(sourceRigidbody.angularVelocity);
				Vector3 targetVelocity = sourceRigidbody.velocity;

				SetArticulationBodyRotation(ab, targetLocalRotation);

				SetArticulationBodyVelocity(ab, targetVelocity, targetLocalAngularVelocity);
			}
		}

        public void TeleportRoot(Vector3 position, Quaternion rotation)
        {
			ArticulationBody root = articulationBodies.First(ab => ab.isRoot);
            root.TeleportRoot(position, rotation);
			root.transform.position = position;
			root.transform.rotation = rotation;
		}

		private static void SetArticulationBodyRotation(ArticulationBody ab, Quaternion targetLocalRotation)
        {
			if (ab.jointType == ArticulationJointType.SphericalJoint)
			{
				Vector3 decomposedRotation = Utils.GetSwingTwist(targetLocalRotation);
				List<float> thisJointPosition = new List<float>();

				var abLocks = ab.GetLocks();
				var abDrives = ab.GetDrives();
				var rotComponents = decomposedRotation.GetComponents();
				for (int dimension = 0; dimension < 3; dimension++)
                {
					if (abLocks[dimension] != ArticulationDofLock.LimitedMotion) continue;

					var drive = abDrives[dimension];
					thisJointPosition.Add(rotComponents[dimension] * Mathf.Deg2Rad);
					drive.target = rotComponents[dimension];
					ab.SetDriveAtIndex(dimension, drive);
				}

				//Apply the determined joint position
				switch (ab.dofCount)
				{
					case 1:
						ab.jointPosition = new ArticulationReducedSpace(thisJointPosition[0]);
						break;
					case 2:
						ab.jointPosition = new ArticulationReducedSpace(
							thisJointPosition[0],
							thisJointPosition[1]);
						break;
					case 3:
						ab.jointPosition = new ArticulationReducedSpace(
							thisJointPosition[0],
							thisJointPosition[1],
							thisJointPosition[2]);
						break;
					default:
						break;
				}
			}
		}

		private static void SetArticulationBodyVelocity(ArticulationBody ab, Vector3 targetLocalVelocity, Vector3 targetLocalAngularVelocity)
        {

			if (ab.jointType == ArticulationJointType.SphericalJoint)
			{
				List<float> thisJointVelocity = new List<float>();

				foreach ((var abLock, var targetLocalAngularVelocityComponent) in ab.GetLocks().Zip(targetLocalAngularVelocity.GetComponents(), Tuple.Create))
				{
					if (abLock != ArticulationDofLock.LimitedMotion) continue;
					thisJointVelocity.Add(targetLocalAngularVelocityComponent);
				}

				switch (ab.dofCount)
				{
					case 1:
						ab.jointVelocity = new ArticulationReducedSpace(thisJointVelocity[0]);
						break;
					case 2:
						ab.jointVelocity = new ArticulationReducedSpace(
							thisJointVelocity[0],
							thisJointVelocity[1]);
						break;
					case 3:
						ab.jointVelocity = new ArticulationReducedSpace(
							thisJointVelocity[0],
							thisJointVelocity[1],
							thisJointVelocity[2]);
						break;
					default:
						break;
				}
				ab.velocity = targetLocalVelocity;
			}
		}
    }

    private class ResettableMjBody : IResettable
    {
		MjBody rootBody;
		public ResettableMjBody(MjBody rootBody)
		{
			this.rootBody = rootBody;
		}
		public void CopyKinematicsFrom(IKinematicReference reference)
        {
			var mjReference = reference as MjKinematicRig;
			var sourceKinematics = MjState.GetMjKinematics(mjReference.Bodies[0]);
			MjScene.Instance.SetMjKinematics(rootBody, sourceKinematics.Item1, sourceKinematics.Item2);
        }

        public void TeleportRoot(Vector3 position, Quaternion rotation)
        {
			MjState.TeleportMjRoot(rootBody.GetComponentInChildren<MjFreeJoint>(), position, rotation);
        }
    }
}
