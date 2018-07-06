package c2g2.kinematics;

import org.joml.Vector2d;

public abstract class Joint2D {

	protected Vector2d position = new Vector2d(0.,0.);
	
	protected boolean fixed = false;
	
	public Vector2d getPos(){
		return position;
	}
	
	public void setPos(Vector2d pos)
	{
		position.x = pos.x;
		position.y = pos.y;
	}

	public boolean isFixed() {
		return fixed;
	}
}
