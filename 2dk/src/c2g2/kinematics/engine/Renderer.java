package c2g2.kinematics.engine;

import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MAJOR;
import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MINOR;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_ESCAPE;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_SPACE;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_LEFT;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_CORE_PROFILE;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_FORWARD_COMPAT;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_PROFILE;
import static org.lwjgl.glfw.GLFW.GLFW_RELEASE;
import static org.lwjgl.glfw.GLFW.GLFW_RESIZABLE;
import static org.lwjgl.glfw.GLFW.GLFW_VISIBLE;
import static org.lwjgl.glfw.GLFW.glfwCreateWindow;
import static org.lwjgl.glfw.GLFW.glfwDefaultWindowHints;
import static org.lwjgl.glfw.GLFW.glfwDestroyWindow;
import static org.lwjgl.glfw.GLFW.glfwGetPrimaryMonitor;
import static org.lwjgl.glfw.GLFW.glfwGetVideoMode;
import static org.lwjgl.glfw.GLFW.glfwInit;
import static org.lwjgl.glfw.GLFW.glfwMakeContextCurrent;
import static org.lwjgl.glfw.GLFW.glfwPollEvents;
import static org.lwjgl.glfw.GLFW.glfwSetCursorPosCallback;
import static org.lwjgl.glfw.GLFW.glfwSetErrorCallback;
import static org.lwjgl.glfw.GLFW.glfwSetKeyCallback;
import static org.lwjgl.glfw.GLFW.glfwSetMouseButtonCallback;
import static org.lwjgl.glfw.GLFW.glfwSetWindowPos;
import static org.lwjgl.glfw.GLFW.glfwSetWindowShouldClose;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.glfw.GLFW.glfwSwapBuffers;
import static org.lwjgl.glfw.GLFW.glfwSwapInterval;
import static org.lwjgl.glfw.GLFW.glfwTerminate;
import static org.lwjgl.glfw.GLFW.glfwWindowHint;
import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;
import static org.lwjgl.opengl.GL11.GL_COLOR_BUFFER_BIT;
import static org.lwjgl.opengl.GL11.GL_DEPTH_BUFFER_BIT;
import static org.lwjgl.opengl.GL11.GL_FALSE;
import static org.lwjgl.opengl.GL11.GL_FLOAT;
import static org.lwjgl.opengl.GL11.GL_TRIANGLES;
import static org.lwjgl.opengl.GL11.GL_TRUE;
import static org.lwjgl.opengl.GL11.glClear;
import static org.lwjgl.opengl.GL11.glDrawArrays;
import static org.lwjgl.opengl.GL15.GL_ARRAY_BUFFER;
import static org.lwjgl.opengl.GL15.GL_STATIC_DRAW;
import static org.lwjgl.opengl.GL15.glBindBuffer;
import static org.lwjgl.opengl.GL15.glBufferData;
import static org.lwjgl.opengl.GL15.glGenBuffers;
import static org.lwjgl.opengl.GL20.GL_COMPILE_STATUS;
import static org.lwjgl.opengl.GL20.GL_FRAGMENT_SHADER;
import static org.lwjgl.opengl.GL20.GL_LINK_STATUS;
import static org.lwjgl.opengl.GL20.GL_VERTEX_SHADER;
import static org.lwjgl.opengl.GL20.glAttachShader;
import static org.lwjgl.opengl.GL20.glCompileShader;
import static org.lwjgl.opengl.GL20.glCreateProgram;
import static org.lwjgl.opengl.GL20.glCreateShader;
import static org.lwjgl.opengl.GL20.glEnableVertexAttribArray;
import static org.lwjgl.opengl.GL20.glGetProgramInfoLog;
import static org.lwjgl.opengl.GL20.glGetProgrami;
import static org.lwjgl.opengl.GL20.glGetShaderInfoLog;
import static org.lwjgl.opengl.GL20.glGetShaderi;
import static org.lwjgl.opengl.GL20.glLinkProgram;
import static org.lwjgl.opengl.GL20.glShaderSource;
import static org.lwjgl.opengl.GL20.glUniform4f;
import static org.lwjgl.opengl.GL20.glUseProgram;
import static org.lwjgl.opengl.GL20.glVertexAttribPointer;
import static org.lwjgl.opengl.GL30.glBindVertexArray;
import static org.lwjgl.opengl.GL30.glGenVertexArrays;
import static org.lwjgl.system.MemoryUtil.NULL;
import java.util.ArrayList;
import java.util.List;
import org.joml.Vector2d;
import c2g2.kinematics.engine.Renderer;
import c2g2.kinematics.engine.Scene;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import org.joml.Vector2d;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWCursorPosCallback;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWKeyCallback;
import org.lwjgl.glfw.GLFWMouseButtonCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import c2g2.kinematics.LinkConnection2D;
import c2g2.kinematics.RigidLink2D;
import c2g2.kinematics.Skeleton2D;

public class Renderer
{
	private static int fkChild = 0;

	// We need to strongly reference callback instances.
	private GLFWErrorCallback errorCallback;
	private GLFWKeyCallback keyCallback;
	private GLFWMouseButtonCallback mouseButtonCallback;

	// The window handle
	private long window;
	Scene mScene;
	private ArrayList<Float> ptlist;
	private ArrayList<Float> linelist;
	private boolean isClicked;
	private Skeleton2D skeleton;
    private double step = Math.PI / 36;
	
    //forward kinematics
	public Renderer(Skeleton2D ske, double m)
	{
		if(ske == null)
			throw new NullPointerException("NULL skeleton");
		skeleton = ske;
		m = 0;
	}

	public void forwardStateUpdate(int fkChild, int sign)
	{
		step *= sign;
		List<LinkConnection2D> connList = skeleton.getConnections();
		fkChild %= connList.size();
		LinkConnection2D current = connList.get(fkChild);
		Vector2d bPos = current.getJoint().getPos();
		forwardNodeUpdate(current, bPos);
	}

	private void forwardNodeUpdate(LinkConnection2D current, Vector2d bPos)
	{
		current.getJoint().setPos(bPos);

		RigidLink2D link = current.getChild();
		if(link == null)
			return;

		double angle = link.getAngle();
		angle += step;
		link.setAngle(angle);
		Vector2d ePos = new Vector2d(bPos.x + link.getLength() * Math.cos(angle), bPos.y + link.getLength() * Math.sin(angle));

		for(int i = 0; i < link.childsize(); i++)
			forwardNodeUpdate(link.getChild(i), ePos);
	}
	
	// inverse kinematics
	public Renderer(Skeleton2D ske, double p, double q)
	{
		if(ske == null)
			throw new NullPointerException("The provided skeleton is NULL");
		skeleton = ske;
		p = 0;
		q = 0;
	}

	public void inverseStateUpdate(double cx, double cy)
	{
		cx = (cx - 300) / 300;
		cy = (300 - cy) / 300;
		List<LinkConnection2D> connList = skeleton.getLeafConnections();
		LinkConnection2D selected = null;
		for(LinkConnection2D conn : connList)
		{
			Vector2d jPos = conn.getJoint().getPos();
			double dx = cx - jPos.x, dy = cy - jPos.y;
			if(dx * dx + dy * dy <= 0.0025)
			{
				selected = conn;
				break;
			}
		}
		
		if(selected == null || selected.getChild() != null)
			return;
		
		InverseAngleUpdate(selected, cx, cy);
		
		LinkConnection2D rootconn = skeleton.getRoot().getParent();
		Vector2d bPos = rootconn.getJoint().getPos();
		inverseNodeUpdate(rootconn, bPos);
	}
	
	private void InverseAngleUpdate(LinkConnection2D selected, double tx, double ty)
	{
		LinkConnection2D curr = selected, thischild = curr;
		while(curr.getParent() != null)
		{
			thischild = curr;
			RigidLink2D link = curr.getParent();
			curr = link.getParent();
			Vector2d bPos = curr.getJoint().getPos();
			double angle = Math.atan2(ty - bPos.y, tx - bPos.x);
			double inc = angle - link.getAngle();
			link.setAngle(angle);
			tx -= link.getLength() * Math.cos(angle);
			ty -= link.getLength() * Math.sin(angle);
			if(link.childsize() > 1)
			{
				for(LinkConnection2D conn : link.getChildren())
					if(conn != thischild)
						conn.getChild().updateAngle(inc);
			}
		}
	}
	
	private void inverseNodeUpdate(LinkConnection2D current, Vector2d bPos)
	{
		current.getJoint().setPos(bPos);

		RigidLink2D link = current.getChild();
		if(link == null)
			return;

		double angle = link.getAngle();
		Vector2d ePos = new Vector2d(bPos.x + link.getLength() * Math.cos(angle), bPos.y + link.getLength() * Math.sin(angle));

		for(int i = 0; i < link.childsize(); i++)
			inverseNodeUpdate(link.getChild(i), ePos);
	}
	
	
		
	public Renderer(Scene scene)
	{
		mScene = scene;
		ptlist = new ArrayList<Float>();
		linelist = new ArrayList<Float>();
		isClicked = false;
	}

	public void run()
	{
		try
		{
			init();
			loop();
			// Release window and window callbacks
			glfwDestroyWindow(window);
			keyCallback.free();
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		finally
		{
			// Terminate GLFW and release the GLFWerrorfun
			glfwTerminate();
			errorCallback.free();
		}
	}

	private void init()
	{
		// Setup an error callback. The default implementation
		// will print the error message in System.err.
		glfwSetErrorCallback(errorCallback);

		// Initialize GLFW. Most GLFW functions will not work before doing this.
		if(glfwInit() != true)
			throw new IllegalStateException("Unable to initialize GLFW");

		// Configure our window
		glfwDefaultWindowHints(); // optional, the current window hints are already the default
		glfwWindowHint(GLFW_VISIBLE, GL_FALSE); // the window will stay hidden after creation
		glfwWindowHint(GLFW_RESIZABLE, GL_TRUE); // the window will be resizable
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

		int WIDTH = 600;
		int HEIGHT = 600;

		// Create the window
		window = glfwCreateWindow(WIDTH, HEIGHT, "Kinematics in 2D", NULL, NULL);
		if(window == NULL)
			throw new RuntimeException("Failed to create the GLFW window");

		// Setup a key callback. It will be called every time a key is pressed,
		// repeated or released.
		glfwSetKeyCallback(window, keyCallback = new GLFWKeyCallback()
		{
			@Override
			public void invoke(long window, int key, int scancode, int action, int mods)
			{
				if(key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
					glfwSetWindowShouldClose(window, true); // We will detect this in our rendering loop
				
				else if(key == GLFW_KEY_SPACE)
				{
					// switch between RigidLink children for forward kinematics
					fkChild++;
					try
					{
						Thread.sleep(100);
					}
					catch(Exception e)
					{
						e.printStackTrace();
					}
				}
				else if(key == GLFW_KEY_RIGHT)
				{
					// increase rotation angles
					Renderer fw = new Renderer(mScene.skeleton, 0);
					fw.forwardStateUpdate(fkChild, 1);
					try
					{
						Thread.sleep(100);
					}
					catch(Exception e)
					{
						e.printStackTrace();
					}
				}
				else if(key == GLFW_KEY_LEFT)
				{
					// decrease rotation angles
					Renderer fw = new Renderer(mScene.skeleton, 0);
					fw.forwardStateUpdate(fkChild, -1);
					try
					{
						Thread.sleep(100);
					}
					catch(Exception e)
					{
						e.printStackTrace();
					}
				}

				
				
				
			}
		});

		glfwSetMouseButtonCallback(window, mouseButtonCallback = new GLFWMouseButtonCallback()
		{
			@Override
			public void invoke(long window, int button, int action, int mods)
			{
				if(action == 1)
					isClicked = true;
				if(action == 0)
					isClicked = false;
			}
		});

		glfwSetCursorPosCallback(window, new GLFWCursorPosCallback()
		{
			// implement your mouse callback function here.
			@Override
			public void invoke(long window, double xpos, double ypos)
			{
				if(isClicked)
				{
					Renderer ik = new Renderer(mScene.skeleton, 0,0);
					ik.inverseStateUpdate(xpos, ypos);
				}
			}
		});
		// Get the resolution of the primary monitor
		GLFWVidMode vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		// Center our window
		glfwSetWindowPos(window, (vidmode.width() - WIDTH) / 2, (vidmode.height() - HEIGHT) / 2);
		// Make the OpenGL context current
		glfwMakeContextCurrent(window);
		// Enable v-sync
		glfwSwapInterval(1);
		// Make the window visible
		glfwShowWindow(window);
	}

	private void loop()
	{
		// This line is critical for LWJGL's interoperation with GLFW's OpenGL context, or any context that is managed externally.
		// LWJGL detects the context that is current in the current thread, creates the ContextCapabilities instance and makes the OpenGL
		// bindings available for use.
		GL.createCapabilities();

		final String vertex_shader = "#version 330\n" + 
									 "in vec3 vp;\n" + 
									 "void main () {\n" + 
									 "  gl_Position = vec4 (vp, 1.0);\n" + 
									 "}";

		final String frag_shader = "#version 330\n" + 
								   "uniform vec4 uColor;" + 
								   "out vec4 frag_colour;" + 
								   "void main () {" + 
								   "  frag_colour = uColor;" + 
								   "}";

		int shader_programme = glCreateProgram();

		int vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertexShaderID, vertex_shader);
		glCompileShader(vertexShaderID);

		if(glGetShaderi(vertexShaderID, GL_COMPILE_STATUS) == 0)
		{
			System.err.println(glGetShaderInfoLog(vertexShaderID, 1024));
			System.exit(1);
		}

		glAttachShader(shader_programme, vertexShaderID);
		
		int fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fragmentShaderID, frag_shader);
		glCompileShader(fragmentShaderID);

		if(glGetShaderi(fragmentShaderID, GL_COMPILE_STATUS) == 0)
		{
			System.err.println(glGetShaderInfoLog(fragmentShaderID, 1024));
			System.exit(1);
		}

		glAttachShader(shader_programme, fragmentShaderID);
		glLinkProgram(shader_programme);
		if(glGetProgrami(shader_programme, GL_LINK_STATUS) == 0)
		{
			System.err.println(glGetProgramInfoLog(shader_programme, 1024));
			System.exit(1);
		}

		while(glfwWindowShouldClose(window) == false)
		{
			ptlist.clear();
			linelist.clear();
			renderSkeleton();
			drawlinelist(shader_programme);
			drawcirclelist(shader_programme);

			// update other events like input handling
			glfwPollEvents();
			// put the stuff we've been drawing onto the display
			glfwSwapBuffers(window);

		}
	}

	private void renderSkeleton()
	{
		ArrayList<Vector2d> pts = mScene.getJointPos();
		for(int i = 0; i < pts.size(); i = i + 2)
		{
			Vector2d p0 = pts.get(i);
			Vector2d p1 = pts.get(i + 1);
			addCircle((float) p0.x, (float) p0.y, 0.05f);
			addCircle((float) p1.x, (float) p1.y, 0.05f);
			addLine((float) p0.x, (float) p0.y, (float) p1.x, (float) p1.y, 0.03f);
		}
	}

	private void addLine(float p0x, float p0y, float p1x, float p1y, float width)
	{
		float dx = p1x - p0x;
		float dy = p1y - p0y;
		float ll = (float) Math.sqrt((float) dx * dx + dy * dy);
		dx = dx / ll * width;
		dy = dy / ll * width;
		glUniform4f(0, 0f, 0.5f, 0f, 1.0f);
		addLinePoint(p0x + dy, p0y - dx, 0f);
		addLinePoint(p0x - dy, p0y + dx, 0f);
		addLinePoint(p1x + dy, p1y - dx, 0f);

		addLinePoint(p0x - dy, p0y + dx, 0f);
		addLinePoint(p1x + dy, p1y - dx, 0f);
		addLinePoint(p1x - dy, p1y + dx, 0f);
	}

	private void addCircle(float cx, float cy, float r)
	{
		int num = 36;
		for(int i = 0; i < num; i++)
		{
			addPoint(cx, cy, 0f);
			float p1x = (float) (Math.cos((float) i * Math.PI * 2 / (float) num) * r) + cx;
			float p1y = (float) (Math.sin((float) i * Math.PI * 2 / (float) num) * r) + cy;
			addPoint(p1x, p1y, 0f);
			float p2x = (float) (Math.cos((float) (i + 1) * Math.PI * 2 / (float) num) * r) + cx;
			float p2y = (float) (Math.sin((float) (i + 1) * Math.PI * 2 / (float) num) * r) + cy;
			addPoint(p2x, p2y, 0f);
		}
	}

	private void addPoint(float x, float y, float z)
	{
		ptlist.add(x);
		ptlist.add(y);
		ptlist.add(z);
	}

	private void addLinePoint(float x, float y, float z)
	{
		linelist.add(x);
		linelist.add(y);
		linelist.add(z);
	}

	private void drawcirclelist(int shader_programme)
	{
		float[] pts = new float[ptlist.size()];

		for(int i = 0; i < ptlist.size(); i++)
			pts[i] = ptlist.get(i);

		glUniform4f(0, 0.5f, 0.5f, 0f, 1.0f);

		FloatBuffer vertices = BufferUtils.createFloatBuffer(pts.length);
		vertices.put(pts);
		// Rewind the vertices
		vertices.rewind();

		int vbo = glGenBuffers();
		int vao = glGenVertexArrays();

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, vertices, GL_STATIC_DRAW);

		glBindVertexArray(vao);

		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

		// wipe the drawing surface clear
		glUseProgram(shader_programme);
		glBindVertexArray(vao);
		// draw points 0-3 from the currently bound VAO with current in-use
		// shader
		glDrawArrays(GL_TRIANGLES, 0, pts.length / 3);
	}

	private void drawlinelist(int shader_programme)
	{
		float[] pts = new float[linelist.size()];

		for(int i = 0; i < linelist.size(); i++)
			pts[i] = linelist.get(i);

		glUniform4f(0, 0.5f, 0f, 0f, 1.0f);

		FloatBuffer vertices = BufferUtils.createFloatBuffer(pts.length);
		vertices.put(pts);
		// Rewind the vertices
		vertices.rewind();

		int vbo = glGenBuffers();
		int vao = glGenVertexArrays();

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, vertices, GL_STATIC_DRAW);

		glBindVertexArray(vao);

		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

		// wipe the drawing surface clear
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glUseProgram(shader_programme);
		glBindVertexArray(vao);
		// draw points 0-3 from the currently bound VAO with current in-use
		// shader
		glDrawArrays(GL_TRIANGLES, 0, pts.length / 3);
	}
}