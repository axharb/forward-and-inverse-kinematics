package c2g2.kinematics.engine;

import java.util.ArrayList;

import org.joml.Vector2d;

import c2g2.kinematics.*;

public class Scene
{
	public Skeleton2D skeleton;
	
	public Scene()
	{
		
	}

	public Scene(Skeleton2D skeleton)
	{
		this.skeleton = skeleton;
	}

	void loadfromXML(String filename)
	{
		skeleton = new Skeleton2D(filename);
	}

	public ArrayList<Vector2d> getJointPos()
	{
		ArrayList<Vector2d> output = new ArrayList<Vector2d>();
		RigidLink2D current = skeleton.getRoot();
		visitNodePos(current, output);
		return output;
	}

	private void visitNodePos(RigidLink2D current, ArrayList<Vector2d> output)
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