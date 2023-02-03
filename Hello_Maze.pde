 /**
 **********************************************************************************************************************
 * @file       Haptic_Physics_Template.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       27-September-2018
 * @brief      Base project template for use with pantograph 2-DOF device and 2-D physics engine
 *             creates a blank world ready for creation
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
 
 
 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 3;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 15.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* Hello Wall*/
FBox wall;
FCircle ball;
FBox winner;
FCircle tst;
PFont font;
boolean touch = true;
boolean win = false;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 600);
  
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[2], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  println(Serial.list());
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  font = createFont("Arial", 14, true);
  PFont.list();
  
  makeWall(15, 1, 15, 3);
  makeWall(1, 6, 8, 6);
  makeWall(1, 4, 22, 4.5);
  makeWall(3, 1, 23, 6);
  makeWall(2.5, 3, 1, 7);
  makeWall(2.5, 2, 1, 3.5);
  makeWall(1, 3.5, 4, 7.25);
  makeWall(1, 2, 4, 3.5);
  makeBall(0.9, 4.4, 4.99);
  makeWall(0.1, 0.1, 3.7, 4.5);
  makeWall(1.2, 1, 5, 4);
  makeWall(1.2, 1, 5, 6);
  makeWall(3, 1, 6, 8.5);
  makeWall(0.25, 0.25, 3.4, 9.9);
  
  makeWall(5, 1, 22.5, 9.5);
  makeWall(5, 3, 22.5, 13);
  makeWall(1, 3, 19.5, 13.5);
  makeWall(2, 1, 16.5, 12.5);
  makeBall(3, 18.5, 10.4);
  makeWall(1, 3, 16, 10.5);
  makeWall(5, 1, 14, 9.5);
  makeWall(6, 1, 13.5, 7.5);
  makeWall(1, 1, 12, 8.5);
  makeWall(1, 3, 19.5, 6.5);
  makeWall(7, 1, 16.5, 5);
  makeWall(1, 2, 11, 4.5);
  makeWall(0.5, 1, 20.2, 7.5);
  
  makeWall(1, 2, 3, 11);
  makeWall(5, 1, 6, 9.5);
  makeWall(1, 4, 8.5, 13.5);
  makeWall(2, 1, 9, 9.5);
  makeWall(1, 1, 9.5, 10);
  
  makeWall(1, 3, 13, 13);
  
  winner = new FBox(1.5, 1.5);
  winner.setPosition(23.5, 10.75);
  winner.setFill(0, 255, 0);
  winner.setStatic(true);
  winner.setSensor(true);
  world.add(winner);
  
  

  
  /* moveable wall 1 */
  wall = new FBox(1.5,1);
  wall.setPosition(2.9, 4.99);
  wall.setFill(100,100,100);
  wall.setStatic(false);  
  world.add(wall);
  
  /* moveable wall 2 */
  wall = new FBox(1,1);
  wall.setPosition(4.7, 4.99);
  wall.setFill(100,100,100);
  wall.setStatic(false);  
  world.add(wall);
  
  /* moveable wall 3 */
  wall = new FBox(4,1);
  wall.setPosition(18.2, 8.49);
  wall.setFill(100,100,100);
  wall.setStatic(false);  
  world.add(wall);
  
  tst = new FCircle(1.5);
  tst.setPosition(12.5, 0.5);
  tst.setSensor(true);
  tst.setImageAlpha(0);
  tst.setNoStroke();
  tst.setNoFill();
  tst.setStatic(true);
  world.add(tst);
  
  for(int i=1;i<=40;i++){
    makeBall(random(0.4, 1), 5, 13);
  }
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(2); 
  s.h_avatar.setFill(255,0,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}

void makeWall(float widthh, float heightt, float x, float y){
  wall = new FBox(widthh,heightt);
  wall.setPosition(x, y);
  wall.setFill(0,0,0);
  wall.setStatic(true);  
  world.add(wall);
}
void makeBall(float radius, float x, float y){
  ball = new FCircle(radius);
  ball.setPosition(x, y);
  ball.setFill(random(70, 255),random(70, 255),random(70, 255));
  ball.setStatic(false);  
  world.add(ball);
}


/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  background(255);
  if(touch){
    textFont(font, 20);
    fill(0, 0, 0);
    textAlign(CENTER);
    text("Make your way to the green area", width/2, 90);
  }
  if(win){
    textFont(font, 80);
    fill(0, 180, 0);
    textAlign(CENTER);
    text("You WIN!", width/2+10, height/2-20, 0);

  }
    
  world.draw(); 
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      pos_ee.set(pos_ee.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-4); 
    s.updateCouplingForce();
    f_ee.set(-s.getVCforceX(), s.getVCforceY());
    f_ee.div(20000); //
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
  
    rendering_force = false;
    if (s.h_avatar.isTouchingBody(tst)){
      touch = true;
    }else{
      touch = false;
    }
    if (s.h_avatar.isTouchingBody(winner)){
      win = true;
    }
    //if(!init){
    //  init = true;
    //}
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
