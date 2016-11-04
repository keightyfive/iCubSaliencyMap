#include "movingrobot.h"


void movingrobot::movingrobotInit() {

	//open a recieving port and connect it to the targetPort from find_location
	targetPort.open("/tutorial/target/in");
	Network::connect("/tutorial/target/out", "/tutorial/target/in");

	//initialise the PolyDriver for the robot head
	options.put("device", "remote_controlboard");
	options.put("local", "/tutorial/motor/client");
	options.put("remote", "/icubSim/head");
	robotHead = new PolyDriver(options);
	if (!robotHead->isValid()) {
		printf("Cannot connect to robot head\n");
	}
	
	//connects robotHead propperties to locals pos, vel and enc. these will be used to move the robot
	robotHead->view(pos);
	robotHead->view(vel);
	vel->setVelocityMode();
	robotHead->view(enc);
	//check if we were able to connect everything from the robot, if not, close the connection
	if (pos == NULL || vel == NULL || enc == NULL) {
		printf("Cannot get interface to robot head\n");
		robotHead->close();
	}
	//get the robot head joints
	jnts = 0;
	pos->getAxes(&jnts);
	setpoints.resize(jnts);
	printf("Joints %d", jnts);
}

movingrobot::movingrobot()
{
	movingrobotInit();
}

void movingrobot::doLook()
{
	//get info from the targetPort 
	Vector *target = targetPort.read(); 
	if (target != NULL) { 
		//if we got information, read it in x, y and conf
		double x = (*target)[0];
		double y = (*target)[1];
		double conf = (*target)[2];

		//recaculate x and y depending on the image size - in this case 320x240 px
		//divided by 2 to get the center of the image
		x -= 320 / 2;
		y -= 240 / 2;

		//velocity on axes, multipy by 0.1 so that we don't make the robot move too fast
		double vx = x*0.1;
		double vy = -y*0.1;

		// setpoints will be sent to velocityMove to move the robot, we only want to use joints 3 and 4 so we set all others to 0
		for (int i = 0; i<jnts; i++) 
		{
			setpoints[i] = 0;
		}

		if (conf > 0.5)
		{
			setpoints[3] = vy;
			setpoints[4] = vx;

			vel->velocityMove(setpoints.data());
		}
		if (conf > 0.5)
		{
			setpoints[2] = vy;
			setpoints[0] = vx;
			vel->velocityMove(setpoints.data());
		}
	}
}
