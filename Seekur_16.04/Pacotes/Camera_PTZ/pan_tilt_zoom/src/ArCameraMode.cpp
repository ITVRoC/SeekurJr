#include <ArExport.h>
#include <ariaOSDef.h>
#include <ArMode.h>
#include "ArCameraMode.h"
#include <ArKeyHandler.h>
#include <ArRVisionPTZ.h>
#include <ArAnalogGyro.h>
#include <ArRobotConfigPacketReader.h>
#include <ariaInternal.h>
#include <ros/ros.h>

AREXPORT ArCameraMode::ArCameraMode(ArRobot *robot, const char *name, char key, char key2): 
  ArMode(robot, name, key, key2),
  myUpCB(this, &ArCameraMode::up),
  myDownCB(this, &ArCameraMode::down),
  myLeftCB(this, &ArCameraMode::left),
  myRightCB(this, &ArCameraMode::right),
  myCenterCB(this, &ArCameraMode::center),
  myZoomInCB(this, &ArCameraMode::zoomIn),  
  myZoomOutCB(this, &ArCameraMode::zoomOut),
  myExerciseCB(this, &ArCameraMode::exercise),
  myRVisionSerialCB(this, &ArCameraMode::rvisionSerial),
  myCom4CB(this, &ArCameraMode::com4),
  myPanAmount(5),
  myTiltAmount(3),
  myAutoFocusOn(true),
  myToggleAutoFocusCB(this, &ArCameraMode::toggleAutoFocus)
{
  myState = STATE_CAMERA;
  myExercising = false;
}

AREXPORT ArCameraMode::~ArCameraMode()
{
  
}

AREXPORT void ArCameraMode::activate(void)
{
  ArKeyHandler *keyHandler;
  if (!baseActivate())
    return;
  // see if there is already a keyhandler, if not something is wrong
  // (since constructor should make one if there isn't one yet
  if ((keyHandler = Aria::getKeyHandler()) == NULL)
  {
    ArLog::log(ArLog::Terse,"ArCameraMode::activate: There should already be a key handler, but there isn't... mode won't work");
    return;
  }

  if (myState == STATE_CAMERA)
    rvisionSerial();
  else if (myState == STATE_PORT)
    takePortKeys();
  else if (myState == STATE_MOVEMENT)
    takeMovementKeys();
  else
    ArLog::log(ArLog::Terse,"ArCameraMode in bad state.");
}

AREXPORT void ArCameraMode::deactivate(void)
{
  if (!baseDeactivate())
    return;
  if (myState == STATE_CAMERA)
    giveUpCameraKeys();
  else if (myState == STATE_PORT)
    giveUpPortKeys();
  else if (myState == STATE_MOVEMENT)
    giveUpMovementKeys();
  else
    ArLog::log(ArLog::Terse,"ArCameraMode in bad state.");
}

AREXPORT void ArCameraMode::userTask(void)
{
  if (myExercising && myCam != NULL && myLastExer.mSecSince() > 7000)
  {
    switch (myExerState) {
    case CENTER:
      /*myCam->panTilt(myCam->getMaxNegPan(), myCam->getMaxPosTilt());
      myExerState = UP_LEFT;
      myLastExer.setToNow();*/
      break;
    case UP_LEFT:
      myCam->panTilt(myCam->getMaxPosPan(), myCam->getMaxPosTilt());
      myExerState = UP_RIGHT;
      myLastExer.setToNow();
      break;
    case UP_RIGHT:
      myCam->panTilt(myCam->getMaxPosPan(), myCam->getMaxNegTilt());
      myExerState = DOWN_RIGHT;
      myLastExer.setToNow();
      break;
    case DOWN_RIGHT:
      myCam->panTilt(myCam->getMaxNegPan(), myCam->getMaxNegTilt());
      myExerState = DOWN_LEFT;
      myLastExer.setToNow();
      break;
    case DOWN_LEFT:
      myCam->panTilt(0, 0);
      myExerState = CENTER;
      myLastExer.setToNow();
      break;
    }      
  }
  if (myExercising && myCam != NULL && myCam->canZoom() && 
      myLastExerZoomed.mSecSince() > 35000)
  {
    if (myExerZoomedIn)
      myCam->zoom(myCam->getMinZoom());
    else
      myCam->zoom(myCam->getMaxZoom());
    myLastExerZoomed.setToNow();
  }
}

AREXPORT void ArCameraMode::left(void)
{
  if (myExercising == true)
    myExercising = false;
  myCam->panRel(-myPanAmount);
  //ArLog::log(ArLog::Terse, "Camera zoom = %f", myCam->getPan());
}

AREXPORT void ArCameraMode::right(void)
{
  if (myExercising == true)
    myExercising = false;
  myCam->panRel(myPanAmount);
 //ArLog::log(ArLog::Normal, "Camera Pan = %f", myCam->getPan());
}

AREXPORT void ArCameraMode::up(void)
{
  if (myExercising == true)
    myExercising = false;
  myCam->tiltRel(myTiltAmount);
  //ArLog::log(ArLog::Normal, "Camera Tilt = %f", myCam->getTilt());
}

AREXPORT void ArCameraMode::down(void)
{  
  if (myExercising == true)
    myExercising = false;
  myCam->tiltRel(-myTiltAmount);
  //ArLog::log(ArLog::Normal, "Camera Tilt = %f", myCam->getTilt());
}

AREXPORT void ArCameraMode::center(void)
{
  if (myExercising == true)
    myExercising = false;
   // setGoalPan(-180);
    //setGoalTilt(43.44);
    reachGoal();
  //myCam->panTilt(0, 0);
  myCam->zoom(myCam->getMinZoom());
}

AREXPORT void ArCameraMode::exercise(void)
{
  if (myExercising == false)
  {
    ArLog::log(ArLog::Normal, 
       "Camera will now be exercised until another command is given.");
    myExercising = true;
    myExerState = UP_LEFT;
    myLastExer.setToNow();
    myCam->panTilt(myCam->getMaxNegPan(), myCam->getMaxPosTilt());
    myLastExerZoomed.setToNow();
    myExerZoomedIn = true;
    if (myCam->canZoom())
      myCam->zoom(myCam->getMaxZoom());
  }
}

AREXPORT void ArCameraMode::toggleAutoFocus()
{
  ArLog::log(ArLog::Terse, "Turning autofocus %s", myAutoFocusOn?"off":"on");
  if(myCam->setAutoFocus(!myAutoFocusOn))
    myAutoFocusOn = !myAutoFocusOn;
}

AREXPORT void ArCameraMode::help(void)
{
  ArLog::log(ArLog::Terse, 
	     "Camera mode will let you control or exercise the camera.");
  ArLog::log(ArLog::Terse, 
      "If you start exercising the camera it will stop your other commands.");
      
      
}

AREXPORT void ArCameraMode::zoomIn(void)
{
  if (myCam->canZoom())
  {
    myCam->zoom(myCam->getZoom() + 
	 ArMath::roundInt((myCam->getMaxZoom() - myCam->getMinZoom()) * .01));
  }
}

AREXPORT void ArCameraMode::zoomOut(void)
{
  if (myCam->canZoom())
  {
    myCam->zoom(myCam->getZoom() - 
	ArMath::roundInt((myCam->getMaxZoom() - myCam->getMinZoom()) * .01));
  }
}

AREXPORT void ArCameraMode::rvisionSerial(void)
{
  myCam = new ArRVisionPTZ(myRobot);
  ArLog::log(ArLog::Terse, "\nRVision selected, now need to select serial port.");
  cameraToPort();
}

AREXPORT void ArCameraMode::com4(void)
{
  myConn.setPort(ArUtil::COM4);
  portToMovement();
}

void ArCameraMode::cameraToPort(void)
{
  myState = STATE_PORT;
  giveUpCameraKeys();
  takePortKeys();
  //helpPortKeys();
}


void ArCameraMode::portToMovement(void)
{
  ArLog::log(ArLog::Normal, "ArCameraMode: Opening connection to camera on port %s", myConn.getPortName());
  if (!myConn.openSimple())
  {
    ArLog::log(ArLog::Terse, 
	       "\n\nArCameraMode: Could not open camera on that port, try another port.\n");
    //helpPortKeys();
    return;
  }
  if(!myCam->setDeviceConnection(&myConn))
  {
    ArLog::log(ArLog::Terse, "\n\nArCameraMode: Error setting device connection!\n");
    return;
  }
  myCam->init();
  myRobot->setPTZ(myCam);
  myState = STATE_MOVEMENT;
  //giveUpPortKeys();
  takeMovementKeys();
  helpMovementKeys();
}

void ArCameraMode::giveUpCameraKeys(void)
{
  remKeyHandler(&myRVisionSerialCB);
}

void ArCameraMode::takePortKeys(void)
{
  com4();
}

void ArCameraMode::giveUpPortKeys(void)
{
  
}

void ArCameraMode::helpPortKeys(void)
{
  ArLog::log(ArLog::Terse, 
	     "You now need to select what port your camera is on.");
  ArLog::log(ArLog::Terse, "%13s:  select COM1 or /dev/ttyS0", "'1'");
  ArLog::log(ArLog::Terse, "%13s:  select COM2 or /dev/ttyS1", "'2'");
  ArLog::log(ArLog::Terse, "%13s:  select COM3 or /dev/ttyS2", "'3'");
  ArLog::log(ArLog::Terse, "%13s:  select COM4 or /dev/ttyS3", "'4'");
  ArLog::log(ArLog::Terse, "%13s:  select /dev/ttyUSB0", "'5'");
  ArLog::log(ArLog::Terse, "%13s:  select /dev/ttyUSB9", "'6'");
}


void ArCameraMode::takeMovementKeys(void)
{
  addKeyHandler(ArKeyHandler::UP, &myUpCB);
  addKeyHandler(ArKeyHandler::DOWN, &myDownCB);
  addKeyHandler(ArKeyHandler::LEFT, &myLeftCB);
  addKeyHandler(ArKeyHandler::RIGHT, &myRightCB);
  addKeyHandler(ArKeyHandler::SPACE, &myCenterCB);
  addKeyHandler('e', &myExerciseCB);
  addKeyHandler('E', &myExerciseCB);
  if (myCam->canZoom())
  {
    addKeyHandler('z', &myZoomInCB);
    addKeyHandler('Z', &myZoomInCB);
    addKeyHandler('x', &myZoomOutCB);
    addKeyHandler('X', &myZoomOutCB);
  }
  addKeyHandler('f', &myToggleAutoFocusCB);
  addKeyHandler('F', &myToggleAutoFocusCB);
}

void ArCameraMode::giveUpMovementKeys(void)
{
  remKeyHandler(&myUpCB);
  remKeyHandler(&myDownCB);
  remKeyHandler(&myLeftCB);
  remKeyHandler(&myRightCB);
  remKeyHandler(&myCenterCB);
  remKeyHandler(&myExerciseCB);
  if (myCam->canZoom())
  {
    remKeyHandler(&myZoomInCB);
    remKeyHandler(&myZoomOutCB);
  }
  remKeyHandler(&myToggleAutoFocusCB);
}

void ArCameraMode::helpMovementKeys(void)
{
  ArLog::log(ArLog::Terse, 
	     "Camera mode will now let you move the camera.");
  ArLog::log(ArLog::Terse, "%13s:  tilt camera up by %d", "up arrow", myTiltAmount);
  ArLog::log(ArLog::Terse, "%13s:  tilt camera down by %d", "down arrow", myTiltAmount);
  ArLog::log(ArLog::Terse, "%13s:  pan camera left by %d", "left arrow", myPanAmount);
  ArLog::log(ArLog::Terse, "%13s:  pan camera right by %d", "right arrow", myPanAmount);
  ArLog::log(ArLog::Terse, "%13s:  center camera and zoom out", 
	     "space bar");
  ArLog::log(ArLog::Terse, "%13s:  exercise the camera", "'e' or 'E'");
  if (myCam->canZoom())
  {
    ArLog::log(ArLog::Terse, "%13s:  zoom in", "'z' or 'Z'");
    ArLog::log(ArLog::Terse, "%13s:  zoom out", "'x' or 'X'");
  }
  ArLog::log(ArLog::Terse, "%13s:  toggle auto/fixed focus", "'f' or 'F'");
}

void ArCameraMode::reachGoal(void)
{
	myCam->panTilt(getGoalPan(), getGoalTilt());
}

void ArCameraMode::ptzSetGoal(int pan, int tilt, int zoom)
{
  // TODO: add zoom
  ArLog::log(ArLog::Normal, "CALLBACK BEING CALLED");
  setGoalPan(pan);
  setGoalTilt(tilt);
  myCam->zoom(zoom);
  reachGoal();
}

