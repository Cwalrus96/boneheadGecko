using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/** This class will be used to give an object the behavior of a kinematic object. Basically, it should function as a 3D version of the octopus project in Processing
 *	This object will have 2 main modes. In Kinematic mode it will adjust it's position and orientation based on the position and orientation of it's "kinematicParent" 
 *	This should function identically to simply parenting the objects in the Unity editor, but the objects will be independent. 
 *	The second mode is Inverse Kinematic mode. In this mode the endpoint will try to match it's position and rotation to an object in the scene, and it will update the 
 *	position and orientation of any parent objects. **/
public class KinematicBone : MonoBehaviour
{
	//CLASS VARIABLES
	//----------------------------------------------------------------------------------------------

	private List<KinematicBone> children;
	public KinematicBone parent;
	
	[SerializeField] GameObject boneMesh; //This object should contain a mesh and a mesh renderer

	Transform anchorPoint; //Represents the point where this object is "linked" to it's parent
	Transform endPoint;    //Represents the point where this object is "linked" to it's children 

	/**These relative angles are used to adjust the relative position of the child bone either with code 
	 * or in the inspector **/
	[SerializeField] Vector3 relativeEulerAngles;

	/**anchorVector and endVector keep track of the positions of the joints relative to the bones **/
	Vector3 anchorVector;
	Vector3 endVector;

	Dictionary<KinematicBone, Vector3> childAnchors; /**This list ensures that all child bones remain in the same relative positions to the parent **/

	private Quaternion worldRotation;   //worldRotation is used to set the absolute orientation of the object relative to the world
	private Quaternion baseRotation;    //baseRotation is the "base" or "initial" orientation of the object relative to it's parent
	private Quaternion relativeRotation;    //relativeRotation is the rotation of the bone relative to it's initial orientation. 
											//This is very useful for applying restrictions on rotations

	Vector3 savedPosition;  //This will be used to check if object moved - if so, update position of children. 
	Vector3 anchorPosition; //This is used to keep the position of the root fixed relative to its parent
	Quaternion savedOrientation;

	bool hasMoved; //Has moved will be set if this bone moves, but can also be set by the parent if they have moved
	float IKRotationSpeed;
	float IKMovementSpeed;

	[SerializeField] float maxRotX;
	[SerializeField] float minRotX;
	[SerializeField] float maxRotY;
	[SerializeField] float minRotY;
	[SerializeField] float maxRotZ;
	[SerializeField] float minRotZ;

	[SerializeField] bool rotateFromCenter; /**Some objects rotate from the center rather than from an endpoint **/ 

	

	//UNITY FUNCTIONS
	//----------------------------------------------------------------------------------------------

	/** Awake initializes this object, so all bones are initialized before being linked with 
	 * Kinematic System **/
	void Awake()
	{
		worldRotation = transform.rotation;
		relativeRotation = Quaternion.identity;
		endPoint = new GameObject(this.name + "EndPoint").transform;
		anchorPoint = new GameObject(this.name + "AnchorPoint").transform; 
		Renderer rend = boneMesh.GetComponent<MeshRenderer>();
		float zLength = rend.bounds.extents.z;
		Vector3 forwardOffset = boneMesh.transform.forward * zLength;
		if (rotateFromCenter)
		{
			anchorPoint.position = rend.bounds.center; 
		}
		else
		{
			anchorPoint.position = rend.bounds.center - forwardOffset;
		}
		
		endPoint.position = rend.bounds.center + forwardOffset; 
		endVector = endPoint.position - transform.position;
		anchorVector = transform.position - anchorPoint.position;
		endVector = Quaternion.Inverse(transform.rotation) * endVector;
		anchorVector = Quaternion.Inverse(transform.rotation) * anchorVector;
		children = new List<KinematicBone>();
		childAnchors = new Dictionary<KinematicBone, Vector3>();
		savedPosition = transform.position * 1;
		savedOrientation = transform.rotation * Quaternion.identity;
		relativeEulerAngles = Vector3.zero;
		hasMoved = false;
		IKMovementSpeed = 1;
		IKRotationSpeed = 1;
		anchorPosition = anchorPoint.transform.position;
	}

	private void Start()
	{
		if (parent != null)//If parent isn't null, positions and orientations are all relative to parent
		{
			parent.children.Add(this);
			Transform parentTransform = parent.transform;
			Vector3 parentOffset = parentTransform.position - anchorPosition;
			parentOffset = Quaternion.Inverse(parent.worldRotation) * parentOffset;
			parent.childAnchors.Add(this, parentOffset);
			baseRotation = worldRotation * Quaternion.Inverse(parent.worldRotation);
			this.anchorPoint.position = parent.transform.position - (parent.worldRotation * parent.childAnchors[this]);
		}
		else //Otherwise, everything is absolute, or "relative" to the scene
		{
			baseRotation = transform.localRotation;
		}
		baseRotation.Normalize();
	}

	/** Update is not called on each individual IK bone. Instead, updates are controlled by the
	 * KinematicSystem class **/
	void Update()
	{

	}

	//KINEMATIC FUNCTIONS
	//-----------------------------------------------------------------------------------------------

	/**Update the position and orientation of this object, then update all child objects based on 
	the new position and orientation of this object. **/
	public void updateKinematics()
	{
		Transform transform = GetComponent<Transform>();
		Vector3 currentPosition = transform.position;
		Quaternion currentOrientation = transform.rotation;
		if ((Vector3.Distance(currentPosition, savedPosition) != 0)
			|| (Quaternion.Angle(savedOrientation, currentOrientation) != 0)
			|| (Quaternion.Angle(relativeRotation, Quaternion.Euler(relativeEulerAngles)) != 0))
		{
			hasMoved = true;
		}
		if (hasMoved)    //Object has moved - update position of children 
		{
			//Debug.Log("Updating Kinematics");
			setRelativeRotation(Quaternion.Euler(relativeEulerAngles)); 
			calculateEndpoints();
			if (children.Count != 0)
			{
				foreach (KinematicBone child in children)
				{
					child.anchorPoint.position = transform.position - (worldRotation * childAnchors[child]);
					child.hasMoved = true;
					child.updateKinematics();
				}
			}
			savedPosition = currentPosition * 1;
			savedOrientation = currentOrientation * Quaternion.identity;
			hasMoved = false;
		}
		Vector3 facingDirection = endPoint.position - anchorPoint.position;
		Debug.DrawRay(anchorPoint.position, facingDirection, Color.black, 0.01f, false);
	}



	/**For now I will simply update the positions of the endPoint based on the position
	 * of the anchor, the bone orientation, and the boneLength **/
	void calculateEndpoints()
	{
		Transform objectTransform = GetComponent<Transform>();
		objectTransform.position = anchorPoint.position + (worldRotation * anchorVector);
		endPoint.position = objectTransform.position + (worldRotation * endVector);
	}

	/**This function will update the position and orientation of this bone based on the target. **/
	public void updateInverseKinematics(Transform target)
	{
		//1. This section handles rotating the bone towards the target orientation (capped by IKRotationSpeed)
		Transform transform = GetComponent<Transform>();
		Vector3 savedAnchorPosition = anchorPoint.position; 
		Vector3 facingDirection = endPoint.position - anchorPoint.position;
		Debug.DrawRay(anchorPoint.position, facingDirection, Color.black, 0.01f, false);
		Vector3 targetDirection = target.position - anchorPoint.position;
		Debug.DrawRay(anchorPoint.position, targetDirection, Color.red, 0.01f, false);
		Quaternion targetRotation = Quaternion.FromToRotation(facingDirection, targetDirection);
		//Add custom code to ensure it rotates based on Rotation speed (degrees per second) 
		float rotationAngle = 0.0f;
		Vector3 rotationAxis = Vector3.zero;
		targetRotation.ToAngleAxis(out rotationAngle, out rotationAxis); 
		if(rotationAngle > (IKRotationSpeed * Time.deltaTime))
		{
			rotationAngle = IKRotationSpeed * Time.deltaTime; 
		}
		targetRotation = Quaternion.AngleAxis(rotationAngle, rotationAxis);
		setWorldRotation(worldRotation * targetRotation); 
		//2. This section handles moving the bone towards the target position (cappted by IKMovementSpeed)
		Vector3 movementDirection = target.position - endPoint.position;
		if (movementDirection.magnitude > (IKMovementSpeed * Time.deltaTime))
		{
			movementDirection = movementDirection.normalized * (IKMovementSpeed * Time.deltaTime); 
		}
		anchorPoint.position = savedAnchorPosition + movementDirection;
		calculateEndpoints(); 
		savedPosition = transform.position * 1;
		savedOrientation = transform.rotation * Quaternion.identity;
	}

	/**Make sure that the rotations do not exceed the maximum or minimum values in any axis. 
	 * These values are all relative to the bone's base orientation **/
	public Vector3 getConstrainedRotation(Vector3 eulerAngles)
	{
		Vector3 baseAngles = baseRotation.eulerAngles;
		float tempX = eulerAngles.x - baseAngles.x;
		float tempY = eulerAngles.y - baseAngles.y;
		float tempZ = eulerAngles.z - baseAngles.z;

		if (tempX > 180)
		{
			tempX -= 360;
		}
		if (tempY > 180)
		{
			tempY -= 360;
		}
		if (tempZ > 180)
		{
			tempZ -= 360;
		}

		if (tempX > maxRotX)
		{
			tempX = maxRotX;
		}
		else if (tempX < minRotX)
		{
			tempX = minRotX;
		}
		if (tempY > maxRotY)
		{
			tempY = maxRotY;
		}
		else if (tempY < minRotY)
		{
			tempY = minRotY;
		}
		if (tempZ > maxRotZ)
		{
			tempZ = maxRotZ;
		}
		else if (tempZ < minRotZ)
		{
			tempZ = minRotZ;
		}

		if (tempX < 0)
		{
			tempX += 360;
		}
		if (tempY < 0)
		{
			tempY += 360;
		}
		if (tempZ < 0)
		{
			tempZ += 360;
		}

		return new Vector3(tempX, tempY, tempZ);
	}

	//GETTERS
	//-------------------------------------------------------------------------------------------------

	public List<KinematicBone> getChildren()
	{
		return children;
	}

	public Transform getEndPoint()
	{
		return endPoint;
	}

	public Transform getAnchorPoint()
	{
		return anchorPoint;
	}

	public Vector3 getAnchorVector()
	{
		return anchorVector;
	}

	public Vector3 getEndVector()
	{
		return endVector;
	}

	public Quaternion getWorldRotation()
	{
		return worldRotation;
	}

	public Dictionary<KinematicBone, Vector3> getChildAnchors()
	{
		return childAnchors; 
	}


	//SETTERS
	//------------------------------------------------------------------------------------------------

	public void setIKMovementSpeed(float speed)
	{
		IKMovementSpeed = speed;
	}

	public void setIKRotationSpeed(float speed)
	{
		IKRotationSpeed = speed;
	}

	/*This function should always be called when setting the world rotation of a bone - should never be set independently */
	public void setWorldRotation(Quaternion rotation)
	{
		Transform objectTransform = GetComponent<Transform>();
		if (parent != null)
		{
			relativeEulerAngles = getConstrainedRotation((rotation * Quaternion.Inverse(parent.worldRotation)).eulerAngles);
			relativeRotation.eulerAngles = relativeEulerAngles;
			worldRotation = relativeRotation * baseRotation * parent.worldRotation;
		}
		/**else
		{
			relativeEulerAngles = getConstrainedRotation(rotation.eulerAngles);
			relativeRotation.eulerAngles = relativeEulerAngles;
			worldRotation = relativeRotation * baseRotation;
		}**/
		objectTransform.rotation = worldRotation;
	}

	/*This function should always be called when setting the relative rotation of a bone - should never be set independently */
	public void setRelativeRotation(Quaternion rotation)
	{
		Transform objectTransform = GetComponent<Transform>();
		relativeEulerAngles = getConstrainedRotation(rotation.eulerAngles);
		relativeRotation.eulerAngles = relativeEulerAngles;
		if (parent != null)
		{
			worldRotation = relativeRotation * baseRotation * parent.worldRotation;
		}
		/**else
		{
			worldRotation = relativeRotation * baseRotation;
		} **/
		objectTransform.rotation = worldRotation;
	}

}
