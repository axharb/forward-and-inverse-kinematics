package c2g2.kinematics;

import java.util.ArrayList;

import org.joml.Vector2d;

public class RigidLink2D {

	private double length;
	private double thetha;
	private Vector2d pos;

	public RigidLink2D() {
		children = new ArrayList<LinkConnection2D>();
	}
	
	public double getAngle()
	{
		return thetha;
	}

	public void setAngle(double angle)
	{
		this.thetha = angle;
	}
	
	public void updateAngle(double inc)
	{
		this.thetha += inc;
	}
	
	public Vector2d getEPos()
	{
		return pos;
	}

	public void setEPos(Vector2d ePos)
	{
		this.pos = ePos;
	}

	private LinkConnection2D parent = null;
	
	private ArrayList<LinkConnection2D> children = null;
	
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
	
	public void setParent(LinkConnection2D p){
		parent = p;
	}
	
	public LinkConnection2D getParent(){
		return parent;
	}
	
	public void addChild(LinkConnection2D c){
		children.add(c);
	}
	
	public LinkConnection2D getChild(int i){
		return children.get(i);
	}
	
	public Joint2D getParentJoint(){
		return parent.getJoint();
	}
	
	public Joint2D getChildJoint() {
		if (children.size()==0) {
			System.err.println("no child joint");
			return null;
		}
		return children.get(0).getJoint();
	}
	
	public ArrayList<LinkConnection2D> getChildren()
	{
		return children;
	}
}
