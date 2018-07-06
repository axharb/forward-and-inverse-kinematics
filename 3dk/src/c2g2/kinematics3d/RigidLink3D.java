package c2g2.kinematics3d;

import java.util.ArrayList;

import org.joml.Vector3d;

/*
 * The class that represents a rigid link or a robotic arm.
 * 
 * Here we assume the links are connected into a tree structure. So
 * each link has one parent, and multiple children
 */
public class RigidLink3D {
	/*
	 * physical length of the link
	 */
	private double length;
	private double xangle;
	private double yangle;
	private double zangle;
	private Vector3d ePos;
	
	/*
	 * The connection to the parent link 
	 * If the link is a root link, its parent LinkConnection2D must 
	 * have a fixed joint. See isRoot() method
	 */
	public RigidLink3D() {
		children = new ArrayList<LinkConnection3D>();
	}

	public double getXAngle()
	{
		return xangle;
	}

	public void setXAngle(double angle)
	{
		this.xangle = angle;
	}
	
	public double getYAngle()
	{
		return yangle;
	}

	public void setYAngle(double angle)
	{
		this.yangle = angle;
	}
	
	public double getZAngle()
	{
		return zangle;
	}

	public void setZAngle(double angle)
	{
		this.zangle = angle;
	}
	
	public Vector3d getEPos()
	{
		return ePos;
	}

	public void setEPos(Vector3d ePos)
	{
		this.ePos = ePos;
	}
	
	private LinkConnection3D parent = null;

	private ArrayList<LinkConnection3D> children = null;
	
	public int childsize(){
		return children.size();
	}
	
	public boolean isRoot() {
		return parent.getParent() == null;
	}
	
	public void setLength(double len){
		length = len;
	}
	
	public double getLength(){
		return length;
	}
	
	public void setParent(LinkConnection3D p){
		parent = p;
	}
	
	public LinkConnection3D getParent(){
		return parent;
	}
	
	public void addChild(LinkConnection3D c){
		children.add(c);
	}
	
	public LinkConnection3D getChild(int i){
		return children.get(i);
	}
	
	public Joint3D getParentJoint(){
		return parent.getJoint();
	}
	
	public Joint3D getChildJoint() {
		if (children.size()==0) {
			System.err.println("no child joint");
			return null;
		}
		return children.get(0).getJoint();
	}
	
	public ArrayList<LinkConnection3D> getChildren()
	{
		return children;
	}
}
