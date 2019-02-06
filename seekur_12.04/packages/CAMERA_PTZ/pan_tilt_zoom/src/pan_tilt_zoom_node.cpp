#include <ros/ros.h>

#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif

#include <ArCommands.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "pan_tilt_zoom_node"); // parâmentro testNode = nome do nó a ser executado, por padrão usa-se o mesmo nome do arquivo.

	ros::NodeHandle n; // The main access point to communication with the ROS system.

	while(ros::ok())
	{
		// Início da implementaação...
		Aria::init();
		ArArgumentParser parser(&argc, argv);
		parser.loadDefaultArguments();
		ArRobot robot;
		ArRobotConnector robotConnector(&parser, &robot);

		// This is used to create and configure the PTZ interface object based on the
	  	// robot's parameter file and this program's command line options (run with
	  	// -help for list). Must be created before calling Aria::parseArgs().
	  	ArPTZConnector ptzConnector(&parser, &robot);

		if(!robotConnector.connectRobot())
		{
		    ArLog::log(ArLog::Terse, "Move Pan-Tilt PTZ Node: Warning, Could not connect to the robot. Won't use robot parameter file for defaults");
		}

		if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
		{
		    Aria::logOptions();
		    Aria::exit(1);
			// TODO: ros exit command
		}
	
		ArLog::log(ArLog::Normal, "Move Pan-Tilt PTZ Node: Connected to robot.");

		
		
		// ArRobot contains an exit action for the Escape key. It also 
		// stores a pointer to the keyhandler so that other parts of the program can
		// use the same keyhandler.
		// Used to perform actions when keyboard keys are pressed
  		ArKeyHandler keyHandler;
		robot.attachKeyHandler(&keyHandler);
		printf("You may press escape to exit\n");


  		// Start the robot task loop running in a new background thread. The 'true' argument means if it loses
 		// connection the task loop stops and the thread exits.
	  	robot.runAsync(true);
		
		robot.com2Bytes(116,10,1); // Sets PTZ power.

		// now add the modes for this demo
  		// these classes are defined in ArModes.cpp in ARIA's source code.
		ArModeCamera camera(&robot, "camera", 'c', 'C');
		// activate the default mode (como só cria um modo, ativar o modo camera
		camera.activate();
	
		// Block execution of the main thread here and wait for the robot's task loop
  		// thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
		// signal)
		robot.waitForRunExit();
		
  		Aria::exit(0);
		
	}
	return 0;

}
