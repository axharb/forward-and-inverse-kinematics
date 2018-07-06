package c2g2.kinematics;

public class LinkConnection2D {

	private RigidLink2D parent = null;
	
	private RigidLink2D child = null;
	
	private Joint2D joint = null;
	
	public Joint2D getJoint() {
		return joint;
	}
	
	public RigidLink2D getParent() {
		return parent;
	}
	
	public RigidLink2D getChild(){
		return child;
	}
	
	public boolean isEnd() {
		return child == null;
	}
	
	public void setParent(RigidLink2D p){
		parent = p;
	}
	
	public void setChild(RigidLink2D c){
		child = c;
	}
	
	public void setJoint(Joint2D j){
		joint = j;
	}
}
