package c2g2.kinematics3d;

public class LinkConnection3D {

	/*
	 * When the two links look like what follows (form into a straight line)
	 * ------(o)------
	 * Here the left link is the parent link, and the right link is the child link
	 * The joint rotateAngle is zero
	 */
	private RigidLink3D parent = null;
	
	private RigidLink3D child = null;
	
	private Joint3D joint = null;
	
	private int index;
	
	public LinkConnection3D(int index)
	{
		this.index = index;
	}

	public Joint3D getJoint() {
		return joint;
	}
	
	public RigidLink3D getParent() {
		return parent;
	}
	
	public RigidLink3D getChild(){
		return child;
	}
	
	public int getIndex()
	{
		return index;
	}

	public void setIndex(int index)
	{
		this.index = index;
	}
	
	
	public boolean isEnd() {
		return child == null;
	}
	
	public void setParent(RigidLink3D p){
		parent = p;
	}
	
	public void setChild(RigidLink3D c){
		child = c;
	}
	
	public void setJoint(Joint3D j){
		joint = j;
	}
	
}
