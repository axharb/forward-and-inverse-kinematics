package c2g2.kinematics.engine;

import java.util.ArrayList;

import org.joml.Vector2d;
import org.joml.Vector3d;

import c2g2.kinematics.*;
import c2g2.kinematics3d.RigidLink3D;
import c2g2.kinematics3d.Skeleton3D;

public class Scene
{
	public Skeleton3D skeleton;
	
	public Scene()
	{
		
	}

	public Scene(Skeleton3D skeleton)
	{
		this.skeleton = skeleton;
	}

	void loadfromXML(String filename)
	{
		skeleton = new Skeleton3D(filename);
	}

	public ArrayList<Vector3d> getJointPos()
	{
		ArrayList<Vector3d> output = new ArrayList<Vector3d>();
		RigidLink3D current = skeleton.getRoot();
		visitNodePos(current, output);
		return output;
	}

	private void visitNodePos(RigidLink3D current, ArrayList<Vector3d> output)
	{
		output.add(current.getParentJoint().getPos());
		output.add(current.getChildJoint().getPos());

		for(int i = 0; i < current.childsize(); i++)
		{
			if(current.getChild(i).isEnd())
				return;
			visitNodePos(current.getChild(i).getChild(), output);
		}
	}
}