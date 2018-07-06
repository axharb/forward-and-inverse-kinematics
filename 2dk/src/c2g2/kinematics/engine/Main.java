package c2g2.kinematics.engine;

public class Main
{
	public static void main(String[] args)
	{
				Scene scene = new Scene();
				scene.loadfromXML("src/resources/models/test.xml");
				Renderer r = new Renderer(scene);
				r.run();

		}
	}
