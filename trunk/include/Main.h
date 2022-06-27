/**
 * @mainpage
 * <h4>The Robotics Library: librobot.so</h4>
 * This robotics library offers classes for the design of robot controllers. A robot controller
 * is separated into motion planning and motion control. Motion planning is performed by a
 * task sequencer that is calculating trajectories based on a list of task objects. It uses
 * the kinematic model of a robot to calculate motor positions from trajectories given in
 * Cartesian coordinates or given in joint angles. Planned motor positions, velocities and
 * accelerations are then passed from the task sequencer to the robot controller that performs
 * the motion control of motors.
 * <br/><br/>
 * The following UML class diagram shows an overview of the classes of this library and their
 * association.
 * <br/><br/>
 * <div style="text-align:center"><img src="robot.png" width="800"/></div>
 * <div style="text-align:center"><b>Overview of the classes of this library</b></div>
 * <br/>
 * The class <code>RobotModel</code> is an abstract class that defines the interfaces of various
 * coordinate transformations, from Cartesian coordinates to joint angles to actuator coordinates.
 * This class needs to be implemented by a specific robot model class for a given robot system.
 * The <code>RobotController</code> class implements the motion control kernel of a robot controller.
 * This class is also an abstract class that defines the interfaces to set all desired motion values
 * for various coordinate spaces, like Cartesian coordinates, joint angles and actuator coordinates.
 * This class must also be implemented by a specific robot controller class for a given robot
 * system. The controller class usually contains a periodic task with control algorithms, so it may
 * be derived from a realtime thread class also.
 * <br/><br/>
 * The class <code>TaskSequencer</code> is an active class that handles the processing of motion
 * planning tasks. It is derived from a realtime thread class which contains a handler method that
 * is called periodically, i.e. every millisecond. The actual motion planning is implemented in
 * specific <code>Task</code> classes, depending on the type of task to execute. All task classes
 * are derived from a common abstract <code>Task</code> class that defines standardised interfaces
 * for the task sequencer. Specific implementations of the <code>Task</code> class allow to plan
 * motions in joint coordinates, in Cartesian coordinates, or execute other tasks such as opening
 * or closing a gripper with digital outputs. All task classes implement the <code>increment()</code>
 * method which calculates motion planning incrementally. This allows to change the task elements
 * with the task sequencer any time.
 * <br/><br/>
 * Many classes use other helper classes for their operation, such as the <code>Tool</code> class
 * which stores the properties, including TCP offsets of a tool of the robot, or the <code>Motion</code>
 * class, which implements the motion planning algorithms for a single degree of freedom. Other utility
 * classes in this library are <code>Transformation</code>, <code>Matrix</code>, <code>Vector</code> or
 * <code>Quaternion</code>, which allow to represent positions and orientations of a frame in space.
 * <br/><br/>
 * The classes <code>MyRobotModel</code> and <code>MyRobotController</code> are robot system specific
 * implementations of a robot model and a robot controller. The other classes of this library may
 * be used unmodified for any type of industrial robot.
 * <br/>
 * The class <code>RealtimeThread</code> which is used by some classes of this library is part of
 * another library, the Industrial Input/Output library.
 * <br/><br/>
 * See the documentation about the individual classes for more information.
 */
 
