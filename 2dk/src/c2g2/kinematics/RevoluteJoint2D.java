package c2g2.kinematics;

import org.joml.Vector2d;

public class RevoluteJoint2D extends Joint2D
{

	public RevoluteJoint2D(Vector2d pos)
	{
		position = pos;
	}
	
	public Vector2d getPosition()
	{
		return position;
	}
	
	public void setPosition(Vector2d pos)
	{
		position = pos;
	}
}