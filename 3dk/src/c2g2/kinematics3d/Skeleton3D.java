package c2g2.kinematics3d;

import org.joml.Vector3d;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.DocumentBuilder;
import org.w3c.dom.Document;
import org.w3c.dom.NodeList;



import org.w3c.dom.Element;

public class Skeleton3D
{
	private RigidLink3D root = null;
	private int counter = 0;

	public RigidLink3D getRoot()
	{
		return root;
	}

	public void setRoot(RigidLink3D r)
	{
		root = r;
	}

	// construct skeleton from xml file
	public Skeleton3D(String xml)
	{
		try
		{
			File inputFile = new File(xml);
			DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
			DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
			Document doc = dBuilder.parse(inputFile);

			doc.getDocumentElement().normalize();
			NodeList nList = doc.getElementsByTagName("root");
			Element e = (Element) nList.item(0);

			double px1 = Double.parseDouble(e.getAttribute("x1"));
			double py1 = Double.parseDouble(e.getAttribute("y1"));
			double pz1 = Double.parseDouble(e.getAttribute("z1"));
			double px2 = Double.parseDouble(e.getAttribute("x2"));
			double py2 = Double.parseDouble(e.getAttribute("y2"));
			double pz2 = Double.parseDouble(e.getAttribute("z2"));

			double dx = px2 - px1, dy = py2 - py1, dz = pz2 - pz1;

			root = new RigidLink3D();
			root.setXAngle(Math.atan2(dy, dz));
			root.setYAngle(Math.atan2(dz, dx));
			root.setZAngle(Math.atan2(dy, dx));
			root.setLength(Math.sqrt(dx * dx + dy * dy + dz * dz));
			Joint3D a0 = new RevoluteJoint3D(new Vector3d(px1, py1, pz1));
			Joint3D a1 = new RevoluteJoint3D(new Vector3d(px2, py2, pz2));
			LinkConnection3D rootconnection = new LinkConnection3D(counter++);
			rootconnection.setChild(root);
			rootconnection.setJoint(a0);
			root.setParent(rootconnection);

			buildSkeletonFromDoc(e, root, a1, 1);
			
			counter = 0;
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}

	private void buildSkeletonFromDoc(Element e, RigidLink3D parent, Joint3D joint0, int level)
	{
		NodeList nList = e.getElementsByTagName("joint" + level);

		if(nList.getLength() == 0)
		{
			LinkConnection3D dummyLink = new LinkConnection3D(counter++);
			dummyLink.setParent(parent);
			dummyLink.setJoint(joint0);
			parent.addChild(dummyLink);
			return;
		}

		Vector3d pPos = joint0.getPos();

		for(int i = 0; i < nList.getLength(); i++)
		{
			Element cElement = (Element) nList.item(i);
			if(cElement == null)
				continue;
			
			double px = Double.parseDouble(cElement.getAttribute("x"));
			double py = Double.parseDouble(cElement.getAttribute("y"));
			double pz = Double.parseDouble(cElement.getAttribute("z"));

			double dx = px - pPos.x, dy = py - pPos.y, dz = pz - pPos.z;

			RigidLink3D ri = new RigidLink3D();
			ri.setXAngle(Math.atan2(dy, dz));
			ri.setYAngle(Math.atan2(dz, dx));
			ri.setZAngle(Math.atan2(dy, dx));
			ri.setLength(Math.sqrt(dx * dx + dy * dy + dz * dz));
			LinkConnection3D connection2d = new LinkConnection3D(counter++);
			Joint3D ji = new RevoluteJoint3D(new Vector3d(px, py, pz));
			connection2d.setParent(parent);
			connection2d.setJoint(joint0);
			connection2d.setChild(ri);
			ri.setParent(connection2d);
			parent.addChild(connection2d);

			buildSkeletonFromDoc(cElement, ri, ji, level + 1);
		}
	}

	public List<LinkConnection3D> getConnections()
	{
		List<LinkConnection3D> output = new ArrayList<>();
		visitNode(root, output);
		return output;
	}

	private void visitNode(RigidLink3D current, List<LinkConnection3D> output)
	{
		output.add(current.getParent());
		for(int i = 0; i < current.childsize(); i++)
		{
			if(!current.getChild(i).isEnd())
				visitNode(current.getChild(i).getChild(), output);
		}
	}


	
	private List<LinkConnection3D> getAllConnections()
	{
		List<LinkConnection3D> output = new ArrayList<>();
		visitAllNode(root, output);
		return output;
	}

	private void visitAllNode(RigidLink3D current, List<LinkConnection3D> output)
	{
		output.add(current.getParent());
		for(int i = 0; i < current.childsize(); i++)
		{
			if(!current.getChild(i).isEnd())
				visitAllNode(current.getChild(i).getChild(), output);
			else
				output.add(current.getChild(i));
		}
	}
	
	private void buildArrays(List<LinkConnection3D> connections, float[] positions, List<Integer> indices)
	{
		//build(root, positions, indices);
		for(LinkConnection3D conn : connections)
		{
			int index = conn.getIndex();
			Vector3d pos = conn.getJoint().getPos();
			positions[index * 3] = (float)pos.x;
			positions[index * 3 + 1] = (float)pos.y;
			positions[index * 3 + 2] = (float)pos.z;
			
			if(conn.getParent() != null)
			{
				indices.add(conn.getParent().getParent().getIndex());
				indices.add(index);
			}
		}
	}
}