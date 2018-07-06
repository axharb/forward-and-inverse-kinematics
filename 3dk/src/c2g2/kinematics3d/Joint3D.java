package c2g2.kinematics3d;

import org.joml.Vector3d;

public abstract class Joint3D {
	/*
	 * Position of the 3D joint
	 */
	protected Vector3d position = new Vector3d(0.,0., 0.);
	
	protected boolean fixed = false;
	
	public Vector3d getPos(){
		return position;
	}
	
	public void setPos(Vector3d pos)
	{
		position.x = pos.x;
		position.y = pos.y;
	}

	public boolean isFixed() {
		return fixed;
	}
}
