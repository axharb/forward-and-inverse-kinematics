package c2g2.kinematics;

import org.joml.Vector2d;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.DocumentBuilder;
import org.w3c.dom.Document;
import org.w3c.dom.NodeList;
import org.w3c.dom.Element;

public class Skeleton2D
{
	private RigidLink2D root = null;

	public RigidLink2D getRoot()
	{
		return root;
	}

	public void setRoot(RigidLink2D r)
	{
		root = r;
	}

	public Skeleton2D(String xml)
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
			double px2 = Double.parseDouble(e.getAttribute("x2"));
			double py2 = Double.parseDouble(e.getAttribute("y2"));

			double dx = px2 - px1, dy = py2 - py1;

			root = new RigidLink2D();
			root.setAngle(Math.atan2(dy, dx));
			root.setLength(Math.sqrt(dx * dx + dy * dy));
			Joint2D a0 = new RevoluteJoint2D(new Vector2d(px1, py1));
			Joint2D a1 = new RevoluteJoint2D(new Vector2d(px2, py2));
			LinkConnection2D rootconnection = new LinkConnection2D();
			rootconnection.setChild(root);
			rootconnection.setJoint(a0);
			root.setParent(rootconnection);

			buildSkeletonFromDoc(e, root, a1, 1);

		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}

	private void buildSkeletonFromDoc(Element e, RigidLink2D parent, Joint2D joint0, int level)
	{
		NodeList nList = e.getElementsByTagName("joint" + level);

		if(nList.getLength() == 0)
		{
			LinkConnection2D dummyLink = new LinkConnection2D();
			dummyLink.setParent(parent);
			dummyLink.setJoint(joint0);
			parent.addChild(dummyLink);
			return;
		}

		Vector2d pPos = joint0.getPos();

		for(int i = 0; i < nList.getLength(); i++)
		{
			Element cElement = (Element) nList.item(i);
			if(cElement == null)
			{
				continue;
			}
			double px = Double.parseDouble(cElement.getAttribute("x"));
			double py = Double.parseDouble(cElement.getAttribute("y"));

			double dx = px - pPos.x, dy = py - pPos.y;

			RigidLink2D ri = new RigidLink2D();
			ri.setAngle(Math.atan2(dy, dx));
			ri.setLength(Math.sqrt(dx * dx + dy * dy));
			LinkConnection2D connection2d = new LinkConnection2D();
			Joint2D ji = new RevoluteJoint2D(new Vector2d(px, py));
			connection2d.setParent(parent);
			connection2d.setJoint(joint0);
			connection2d.setChild(ri);
			ri.setParent(connection2d);
			parent.addChild(connection2d);

			buildSkeletonFromDoc(cElement, ri, ji, level + 1);
		}
	}

	public List<LinkConnection2D> getConnections()
	{
		List<LinkConnection2D> output = new ArrayList<>();
		visitNode(root, output);
		return output;
	}

	private void visitNode(RigidLink2D current, List<LinkConnection2D> output)
	{
		output.add(current.getParent());
		for(int i = 0; i < current.childsize(); i++)
		{
			if(!current.getChild(i).isEnd())
				visitNode(current.getChild(i).getChild(), output);
		}
	}

	public List<LinkConnection2D> getLeafConnections()
	{
		List<LinkConnection2D> output = new ArrayList<>();
		visitLeafNode(root, output);
		return output;
	}

	private void visitLeafNode(RigidLink2D current, List<LinkConnection2D> output)
	{
		int i =0;
		while( i < current.childsize())
		{
			if(current.getChild(i).isEnd())
				output.add(current.getChild(i));
			else
				visitLeafNode(current.getChild(i).getChild(), output);
		i++;
		}
	}
	
	public List<Double> getLinkAngles()
	{
		List<Double> output = new ArrayList<>();
		visitAngleNode(root, output);
		return output;
	}
	
	private void visitAngleNode(RigidLink2D current, List<Double> output)
	{
		output.add(current.getAngle());
		int i =0;
		while(i < current.childsize())
		{
			LinkConnection2D child = current.getChild(i);
			if(!child.isEnd())
			{
				visitAngleNode(child.getChild(), output);
			}
		i++;
		}
	}
}