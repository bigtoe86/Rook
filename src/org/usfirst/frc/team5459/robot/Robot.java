//Team 5459 Code
//Filip Kernan
package org.usfirst.frc.team5459.robot;

import java.security.PublicKey;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;


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
    Victor arm,shoot1,shoot2,treads;//victor controllers
    Talon leftRear;
    Servo gate,push;//servos for pushing out ball and gate
    ADXRS450_Gyro gyro;//gyro
    AnalogInput forwardSensor, sideSensor;
    CameraServer camera;
    double speedX, speedY, speedRote, gyroAngle, throttle, valueToMm = 0.001041/* scale factor for analog ultrasonics*/, xDistance, yDistance;
    
    boolean armed = false,hasShot = false,countTick = false, xPosition, yPosition;
    int tickCount = 0, currentTick = 0;
    
    
    
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	rook = new RobotDrive(4,7, 0, 2);
    	rook.setInvertedMotor(MotorType.kRearLeft, true);
    	rook.setInvertedMotor(MotorType.kFrontLeft,  true);
    	rook.setSafetyEnabled(true);
    	rook.setExpiration(0.1);
    	stick1 = new Joystick(0); 
    	stick2 = new Joystick(1);
    	shoot1 = new Victor(1);
    	shoot2 = new Victor(5);
    	treads = new Victor(3);
    	arm = new Victor(6);
    	arm.setSafetyEnabled(true);
    	arm.setExpiration(0.1);
    	gate = new Servo(8);
    	push = new Servo(9);
    	gyro = new ADXRS450_Gyro();
    	gyro.calibrate();
    	gyro.reset();
    	forwardSensor = new AnalogInput(0);
    	sideSensor = new AnalogInput(1);
    	camera = CameraServer.getInstance();
    	camera.setQuality(50);
    	camera.startAutomaticCapture("cam0 ");
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	gyro.reset();
    	gyroAngle = gyro.getAngle();
    	if (gyroAngle >= 360) {
			gyroAngle = gyroAngle - 360;
		}
    	xDistance = distance(forwardSensor);
    	yDistance = distance(sideSensor);//gets current position
    	if (tickCount< 200) {
			rook.mecanumDrive_Polar(0.6, 0, 0);//drives forward 
		} else {
			rook.mecanumDrive_Polar(0, 0, 0);
		}//drives forward for 4 sec 
    	if (tickCount > 200 && xDistance > 4308) {
    		if (tickCount > 500 && yDistance > 914) {
    			rook.mecanumDrive_Cartesian(0.5, 0.5, 0, 0);
    		}else {
				rook.mecanumDrive_Cartesian(0.5, 0, 0, 0);
			}
		}else {
			if (tickCount > 500 && yDistance > 914) {
				rook.mecanumDrive_Cartesian(0, 0.5, 0, 0);
			} 
		}
    	if(xDistance <= 4308 ){
    		xPosition = true;
    	}
    	if (tickCount > 500 && yDistance > 914) {
			rook.mecanumDrive_Cartesian(0, 0.5, 0, gyroAngle);
		}//drives to ideal y
    	if (yDistance <= 914 ) {
			yPosition = true;
		} 
    	if (xPosition && yPosition) {//in ideal position
			currentTick = tickCount;
			if (gyroAngle < 60 && gyroAngle > 60) {
				rook.mecanumDrive_Polar(0, 0.0, 0.5);
			}else {
				rook.mecanumDrive_Polar(0, 0.0, 0);
			}//turns to 60 degrees
			if (yDistance < 914.0) {
				rook.mecanumDrive_Polar(0.75, 0.0, 0.0);
			}
			
			if (yDistance >= 914.0) {
				 shoot1.set(-0.25);
				 shoot2.set(0.25);
			}else {
				shoot1.set(0.0);
				shoot2.set(0.0);
			}//shoots after in ideal shoot position
		}
    	tickCount++;//counts ticks; tick == 200msec
    	Timer.delay(0.005);
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	throttle = (stick1.getThrottle()/2);
    	speedX = stick1.getX() * throttleEncode(stick1);
    	speedY = stick1.getY() * throttleEncode(stick1);
    	speedRote = stick1.getDirectionDegrees() * throttleEncode(stick1);
    	gyroAngle = gyro.getAngle();
    	if (gyroAngle >= 360) {
			gyroAngle = gyroAngle - 360;
		}
    	if (stick1.getRawButton(2)) {
			rook.mecanumDrive_Cartesian(speedX, speedY, speedRote, gyroAngle );//if angle starts freaking out then uncomment the above if statment 
    	}else {
    		rook.mecanumDrive_Cartesian(speedX, speedY, 0, gyroAngle);
		}//rotation toggle
		if(stick1.getRawButton(1)){
    		treads.set(1.0); 
    	}else {
    		treads.set(0.0);
    	}
		//tread toggle
    	if(stick2.getRawButton(2)){//arm shooter
    		shoot1.set(-0.25);
    		shoot2.set(0.25);
    		armed = true;//will only fire if armed is true
    	}else {
			armed = false;
			shoot1.set(0.0);
			shoot2.set(0.0);
		}
    	if (stick2.getRawButton(1) && armed == true) {//shoots
			gate.set(1.0);
			push.set(1.0); 
			hasShot = true;
			countTick = true;
		}
    	
    	if (hasShot == true && tickCount == 20){//auto reset push
    		push.set(0.0);
    		countTick = false;
    		tickCount = 0;
    	}
    	if (stick2.getRawButton(12) && hasShot == true) {//closes gate
			gate.set(0.0);
		}
    	
    	   
    	if (stick2.getRawButton(6)) {
    		shoot1.set(0.25);
    		shoot2.set(-0.25);
		}else {
			shoot1.set(0.0);
			shoot2.set(0.0);
		}//draws in ball
    	
    	if (stick2.getRawButton(5)) {//arm up
			arm.set(0.2);
		}else {
			arm.set(0.0);
		}
    	if (stick2.getRawButton(3)) {//arm down
			arm.set(-0.2);
		}else {
			arm.set(0.0);
		}
    	
    	if (countTick) {
			tickCount++;
		}//counts ticks
    	Timer.delay(0.005);
    	//TODO: rerun auto on stick 2
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
     double throttleEncode(Joystick stick) {
		double[] encode ={1.0,0.825,0.65,0.475,0.3, 0.125};//values for var speed
    	if (stick.getThrottle()>= -0.6666) {
			return encode[0];
		}
		if (stick.getThrottle()<= 0.6666) {
			return encode[5];
		}
    	if (stick.getThrottle() < -0.6666 && stick.getThrottle() >= -0.3333) {
			return encode[1];
		}
    	if (stick.getThrottle() < 0.6666 && stick.getThrottle() >= 0.3333) {
			return encode[4];
		}
    	if (stick.getThrottle() < -0.3333 && stick.getThrottle() > 0) {
			return encode[2];
		}
    	if (stick.getThrottle() < 0.3333 && stick.getThrottle() >= 0) {
			return encode[3];
		}
    	return 0.0;
	}
    double distance(AnalogInput sensor){
    	double dis;
    	dis = sensor.getValue() * valueToMm;
    	dis = dis / Math.cos(gyroAngle);
    	if (dis < 0) {
			dis = dis * -1;
		}
    	return dis;
    }
}
