//Team 5459 Code
//Filip Kernan
package org.usfirst.frc.team5459.robot;

import java.security.PublicKey;

import com.ni.vision.NIVision;
//import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
//import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    RobotDrive rook;//drive name
    Joystick stick1, stick2;//the joysticks
    Victor shoot1,shoot2,treads;//victor controllers
    Talon arm;
    ADXRS450_Gyro gyro;//gyro
    AnalogInput forwardSensor, sideSensor;
    DigitalInput isBallIn;
    CameraServer camera;
    Image frame;
    Integer noAuto, Auto, simpleAuto;
    SendableChooser autoChooser;
    double speedX, speedY, speedRote, gyroAngle, varSpeed, valueToMm = 0.001041/* scale factor for analog ultrasonics*/, xDistance, yDistance;
    boolean armed = false,hasShot = false,countTick1 = false, countTick2 = false, countTick3 = false, xPosition, yPosition, autoRerun = false, armDown = true, ballIn = false;
    int tickCount1 = 0, tickCount2 = 0, currentTick = 0, tickCount3 = 0, session;
    
    
    
 /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
     rook = new RobotDrive(4, 2, 0, 7);
     
     rook.setInvertedMotor(MotorType.kRearLeft, true);
     rook.setInvertedMotor(MotorType.kFrontLeft,  true);//reverses left motors
     rook.setInvertedMotor(MotorType.kRearRight, false);
     rook.setSafetyEnabled(true);
     rook.setExpiration(0.1);
     stick1 = new Joystick(0); 
     stick2 = new Joystick(1);
     shoot1 = new Victor(3);
     shoot2 = new Victor(5);
     shoot2.setInverted(true);//inverts motor
     treads = new Victor(1);
     arm = new Talon(8);
     gyro = new ADXRS450_Gyro();
     noAuto = 1;
     Auto = 0;
     simpleAuto = 2;
     gyro.calibrate();
     gyro.reset();
     forwardSensor = new AnalogInput(0);
     sideSensor = new AnalogInput(1);
     isBallIn = new DigitalInput(0);
     camera = CameraServer.getInstance();
     camera.setQuality(50);
     camera.startAutomaticCapture("cam0");
     autoChooser = new SendableChooser();
     autoChooser.addDefault("no auto", noAuto);
     autoChooser.addObject("Auto", Auto);
     autoChooser.addObject("simple auto", simpleAuto);
     SmartDashboard.putData("auto chooser", autoChooser);
     /*frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // the camera name (ex "cam0") can be found through the roborio web interface
        session = NIVision.IMAQdxOpenCamera("cam0",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);*/
    }//TODO make cross hairs for goal offset
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
     if (autoChooser.equals(Auto)) {
      gyroAngle = gyro.getAngle();
      if (gyroAngle >= 360) {
    gyroAngle = gyroAngle - 360;
   }
      xDistance = distance(forwardSensor);
      yDistance = distance(sideSensor);//gets current position
      if (tickCount1 < 20) {
    rook.mecanumDrive_Polar(0.6, 0, 0);//drives forward 
   } else {
    rook.mecanumDrive_Polar(0, 0, 0);
   }//drives forward for 4 sec 
      if (tickCount1 > 20 && xDistance > 4308 && tickCount1 < 100) {
       if (yDistance > 914) {
        rook.mecanumDrive_Cartesian(0.5, 0.5, 0, 0);
       }else {
     rook.mecanumDrive_Cartesian(0.5, 0, 0, 0);
    }
   }else {
    if (yDistance > 914 && tickCount1 < 100) {
     rook.mecanumDrive_Cartesian(0, 0.5, 0, 0);
    }else {
     rook.mecanumDrive_Cartesian(0, 0, 0, gyroAngle);
    }
   }//goes to ideal position
      if(xDistance <= 4308 ){
       xPosition = true;
      }//checks if x is correct
      
      if (yDistance <= 914 ) {
    yPosition = true;
   } //checks if y is correct
      if (xPosition && yPosition) {//in ideal position
    currentTick = tickCount1;
    if (gyroAngle < 60 && gyroAngle > 60) {
     rook.mecanumDrive_Polar(0, 0.0, 0.5);
    }else {
     rook.mecanumDrive_Polar(0, 0.0, 0);
    }//turns to 60 degrees
    if (yDistance < 400.0) {
     rook.mecanumDrive_Polar(0.75, 0.0, 0.0);
    }
    
    if (yDistance >= 400.0) {
      shoot1.set(-0.25);
      shoot2.set(-0.25);
    }else {
     shoot1.set(0.0);
     shoot2.set(0.0);
    }//shoots after in ideal shoot position
  }
     }else if (autoChooser.getSelected().equals(simpleAuto)) {

   if (tickCount1 < 70) {
    rook.mecanumDrive_Polar(-1, 0, 0);

   }
  }
     tickCount1++;//counts ticks; tick == 200msec
     Timer.delay(0.005);
     
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {


    	 SmartDashboard.putNumber("side sensor", distance(sideSensor));
	     SmartDashboard.putNumber("forward sensor", distance(forwardSensor));//smart dash board
	
	     if(stick1.getRawButton(12)){
	    	 varSpeed = 0.5;
	     }else{
	    	 varSpeed = 1.0;
	     }
	     speedX = -stick1.getX() * varSpeed;
	     speedY = -stick1.getY() * varSpeed;
	     speedRote = -stick1.getZ() * varSpeed;
	
	     gyroAngle = -gyro.getAngle();
	     if (gyroAngle >= 360) {
	    	 gyroAngle = gyroAngle - 360;
	     }
	     SmartDashboard.putNumber("Gyro angle", gyroAngle);
	     if (stick1.getRawButton(2)) {
	    	rook.mecanumDrive_Cartesian(speedX, speedY, speedRote, 0 );
	     }else {
	       rook.mecanumDrive_Cartesian(speedX, speedY, 0, 0);
	
	     }//rotation toggle
	     
	   
	     if (stick2.getRawButton(1)) {
	    	shoot1.set(1);
	    	//this can be made higher
	     }else if (stick2.getRawButton(2) && ballIn) {
		     shoot1.set(-1);
		       
		 }else {
		    shoot1.set(0.0);
		    
		 }
	      
	      //TODO maybe adjust when ball is in
	      
	         
	      
	  
	    
	     
	     if (countTick3){
	    	 tickCount3++;
	     }
	     if (countTick2) {
	    	 tickCount2++;
	     }//counts ticks
	     
	     Timer.delay(0.005);
	     
	        //NIVision.IMAQdxStopAcquisition(session);
	

        //NIVision.IMAQdxStopAcquisition(session);


    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
     
    }
    
    /*
     * 
     *
     */
    //TODO change to buttons
     
    double distance(AnalogInput sensor){
     double dis;
     dis = sensor.getValue() * valueToMm;

     //dis = dis / Math.cos(gyroAngle);

     if (dis < 0) {
   dis = dis * -1;
  }
     return dis;
    }
}
