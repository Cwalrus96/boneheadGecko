using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//[ExecuteInEditMode]
public class KinematicSystem : MonoBehaviour
{
	/**The root of the kinematic system will be assigned in the inspector. 
	 * The rest of the bones and the tip will be determined at startup **/
	[SerializeField] KinematicBone root;
	[SerializeField] private KinematicBone tip;
	[SerializeField] private List<KinematicBone> bones;
	[SerializeField] Transform IKTarget;
	[SerializeField] float IKRotationSpeed;
	[SerializeField] float IKMovementSpeed;

	/**This determines whether the system is updating Kinematics or Inverse Kinematics **/
	[SerializeField] bool inverse;

	// Start is called before the first frame update
	void Start()
	{
		bones = new List<KinematicBone>();
		recursiveAdd(root); 
		foreach (KinematicBone bone in bones)
		{
			bone.setIKMovementSpeed(IKMovementSpeed);
			bone.setIKRotationSpeed(IKRotationSpeed);
		}
	}

	void recursiveAdd(KinematicBone current)
	{
		bones.Add(current); 
		if(current.getChildren().Count != 0)
		{
			foreach(KinematicBone child in current.getChildren())
			{
				recursiveAdd(child); 
			}
		}
	}

	// Update is called once per frame
	void Update()
	{
		if (inverse)
		{
			updateInverseKinematics();
		}
		else
		{
			updateKinematics();
		}
	}

	void updateInverseKinematics()
	{
		KinematicBone current = tip;
		Transform currentTarget = IKTarget;
		while (current.parent != null && current != root)
		{
			current.updateInverseKinematics(currentTarget);
			currentTarget = current.getAnchorPoint();
			current = current.parent;
			if(current.getChildren().Count > 1)
			{
				current.updateKinematics(); 
			}
		}
		//At this point current is the root of the system
		Vector3 savedPosition = current.getAnchorPoint().position + Vector3.zero;
		current.updateInverseKinematics(currentTarget);
		current.getAnchorPoint().position = savedPosition;
		recursiveUpdateIKPositions(current); 
	}

	void recursiveUpdateIKPositions(KinematicBone current)
	{
		current.transform.position = current.getAnchorPoint().position + (current.getWorldRotation() * current.getAnchorVector());
		current.getEndPoint().position = current.transform.position + (current.getWorldRotation() * current.getEndVector());
		if(current.getChildren().Count > 0)
		{
			foreach(KinematicBone child in current.getChildren())
			{
				child.getAnchorPoint().position = current.transform.position - (current.getWorldRotation() * current.getChildAnchors()[child]);
				recursiveUpdateIKPositions(child); 
			}
		}
	}

	void updateKinematics()
	{
		foreach (KinematicBone bone in bones)
		{
			bone.updateKinematics();
		}
	}
}

