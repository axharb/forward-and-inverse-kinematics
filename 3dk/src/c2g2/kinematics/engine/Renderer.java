package c2g2.kinematics.engine;
import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MAJOR;
import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MINOR;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_ESCAPE;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_SPACE;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_LEFT;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_UP;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_DOWN;
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
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import org.joml.Vector3d;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWKeyCallback;
import org.lwjgl.glfw.GLFWMouseButtonCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import c2g2.kinematics3d.LinkConnection3D;
import c2g2.kinematics3d.RigidLink3D;
import c2g2.kinematics3d.Skeleton3D;

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
	private double time;   
	private double step = Math.PI / 36;
	private Skeleton3D skeleton;

	//forward kinematics in 3D
	public Renderer(Skeleton3D ske)
	{
		if(ske == null)
			throw new NullPointerException(" NULL skeletonL");
		skeleton = ske;
	}
	public void updateState(int fkChild, int axis, int sign)
	{
		step *= sign;
		List<LinkConnection3D> connList = skeleton.getConnections();
		fkChild %= connList.size();
		LinkConnection3D current = connList.get(fkChild);
		Vector3d bPos = current.getJoint().getPos();
		
		if(axis == 0)
			updateNodeX(current, bPos);
		else if(axis == 1)
			updateNodeY(current, bPos);
		else
			updateNodeZ(current, bPos);
		
	}

	private void updateNodeX(LinkConnection3D current, Vector3d bPos)
	{
		current.getJoint().setPos(bPos);

		RigidLink3D link = current.getChild();
		if(link == null)
			return;

		double angle = link.getXAngle();
		angle += step;
		link.setXAngle(angle);
		Vector3d ePos = new Vector3d(bPos.x, bPos.y + link.getLength() * Math.cos(angle), bPos.z + link.getLength() * Math.sin(angle));

		for(int i = 0; i < link.childsize(); i++)
			updateNodeX(link.getChild(i), ePos);
	}
	
	private void updateNodeY(LinkConnection3D current, Vector3d bPos)
	{
		current.getJoint().setPos(bPos);

		RigidLink3D link = current.getChild();
		if(link == null)
			return;

		double angle = link.getYAngle();
		angle += step;
		link.setYAngle(angle);
		Vector3d ePos = new Vector3d(bPos.x + link.getLength() * Math.cos(angle), bPos.y, bPos.z + link.getLength() * Math.sin(angle));

		for(int i = 0; i < link.childsize(); i++)
			updateNodeY(link.getChild(i), ePos);
	}
	
	private void updateNodeZ(LinkConnection3D current, Vector3d bPos)
	{
		current.getJoint().setPos(bPos);

		RigidLink3D link = current.getChild();
		if(link == null)
			return;

		double angle = link.getZAngle();
		angle += step;
		link.setZAngle(angle);
		Vector3d ePos = new Vector3d(bPos.x + link.getLength() * Math.cos(angle), bPos.y + link.getLength() * Math.sin(angle), bPos.z);

		for(int i = 0; i < link.childsize(); i++)
			updateNodeZ(link.getChild(i), ePos);
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
		window = glfwCreateWindow(WIDTH, HEIGHT, "3D Forward Kinematics!", NULL, NULL);
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
					Renderer fw = new Renderer(mScene.skeleton);
					fw.updateState(fkChild, 1,1);
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
					Renderer fw = new Renderer(mScene.skeleton);
					fw.updateState(fkChild, 1,-1);
					try
					{
						Thread.sleep(100);
					}
					catch(Exception e)
					{
						e.printStackTrace();
					}
				}
				else if(key == GLFW_KEY_UP)
				{
					// decrease rotation angles
					Renderer fw = new Renderer(mScene.skeleton);
					fw.updateState(fkChild, 0,-1);
					try
					{
						Thread.sleep(100);
					}
					catch(Exception e)
					{
						e.printStackTrace();
					}
				}
				else if(key == GLFW_KEY_DOWN)
				{
					// decrease rotation angles
					Renderer fw = new Renderer(mScene.skeleton);
					fw.updateState(fkChild, 0,-1);
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
		ArrayList<Vector3d> pts = mScene.getJointPos();
		for(int i = 0; i < pts.size(); i = i + 2)
		{
			Vector3d p0 = pts.get(i);
			Vector3d p1 = pts.get(i + 1);


			addLine((float) p0.x, (float) p0.y,(float) p0.z, (float) p1.x, (float) p1.y, (float) p1.z, 0.03f);
		}
	}

	private void addLine(float p0x, float p0y, float p1x, float p1y,float p0z, float p1z, float width)
	{
		float dx = p1x - p0x;
		float dy = p1y - p0y;
		float dz = p1z - p0z;
		float ll = (float) Math.sqrt((float) dx * dx + dy * dy + dz*dz);
		dx = dx / ll * width;
		dy = dy / ll * width;
		dz = dz / ll * width;
		glUniform4f(0, 0f, 0.5f, 0f, 1.0f);

		addLinePoint(p0x + dy, p0y - dx, 0f);
		addLinePoint(p0x - dy, p0y + dx, 0f);
		addLinePoint(p1x + dy, p1y - dx, 0f);

		addLinePoint(p0x - dy, p0y + dx, 0f);
		addLinePoint(p1x + dy, p1y - dx, 0f);
		addLinePoint(p1x - dy, p1y + dx, 0f);
	}


	private void addPoint(float x, float y,float z,float k)
	{
		ptlist.add(x);
		ptlist.add(y);
		ptlist.add(z);
		ptlist.add(k);
		
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

		glUniform4f(0,1.9f,1.8f, 0.8f, 1.0f);

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