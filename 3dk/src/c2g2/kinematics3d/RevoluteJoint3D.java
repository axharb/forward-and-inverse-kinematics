package c2g2.kinematics3d;

import org.joml.Vector3d;

public class RevoluteJoint3D extends Joint3D
{
	/*
	 * When the two links look like what follows (form into a straight line)
	 * -------o------- The joint rotateAngle is zero
	 */
	public RevoluteJoint3D(Vector3d pos)
	{
		position = pos;
	}
	
	public Vector3d getPosition()
	{
		return position;
	}
	
	public void setPosition(Vector3d pos)
	{
		position = pos;
	}
}