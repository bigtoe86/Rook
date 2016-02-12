
package org.usfirst.frc.team5459.robot;

import java.security.PublicKey;

import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    RobotDrive Rook;//drive name
    Joystick Stick1;
    Joystick Stick2;//the joysticks
    Victor Arm;//can talon
    Victor Shoot1;
    Victor Shoot2;//victors for shooter
    Victor Treads;//victor for treads
    Servo Gate;//servo for gate
    Servo Push;//servo for pushing out ball
    ADXRS450_Gyro Gyro;//gyro
    Ultrasonic ForwardSensor, SideSensor;
    double speedX, speedY, speedRote, gyroAngle;
    double Kp = 0.03;
    boolean armed = false;
    boolean hasShot = false;
    int tickCount = 0;
    boolean countTick = false;
    
    
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	Rook = new RobotDrive(4,7, 0, 2);
    	Stick1 = new Joystick(1);
    	Stick2 = new Joystick(2);
    	Shoot1 = new Victor(3);
    	Shoot2 = new Victor(5);
    	Treads = new Victor(1);
    	Arm = new Victor(6);
    	Gate = new Servo(8);
    	Push = new Servo(9);
    	Gyro = new ADXRS450_Gyro();
    	Gyro.calibrate();
    	Gyro.reset();
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	//TODO write auto
    	//TODO write auto
    	//TODO write auto
    	//TODO write auto
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	speedX = Stick1.getX() * Stick1.getThrottle();
    	speedY = Stick1.getY() * Stick1.getThrottle();
    	speedRote = Stick1.getDirectionDegrees() * Stick1.getThrottle();
    	gyroAngle = Gyro.getAngle();
    	/*if (gyroAngle >= 360) {
			gyroAngle = gyroAngle - 360;
		}*/
    	if (Stick1.getRawButton(2)) {
			Rook.mecanumDrive_Cartesian(speedX, speedY, speedRote, -gyroAngle * Kp);//if angle starts freaking out then uncomment the above if statment 
    	}else {
    		Rook.mecanumDrive_Cartesian(speedX, speedY, 0, -Gyro.getAngle() * Kp);
		}
		if(Stick1.getRawButton(1)){
    		Treads.set(1.0);
    	}//activate treads
    	if(Stick2.getRawButton(2)){//arm shooter
    		Shoot1.set(1.0);
    		Shoot2.set(-1.0);
    		armed = true;//will only fire if armed is true
    	}else {
			armed = false;
		}
    	if (Stick2.getRawButton(1) && armed == true) {//shoots
			Gate.set(1.0);
			Push.set(1.0); 
			hasShot = true;
			countTick = true;
		}
    	
    	if (hasShot == true && tickCount == 2){//auto reset push
    		Push.set(0.0);
    		countTick = false;
    	}
    	if (Stick2.getRawButton(12) && hasShot == true) {//closes gate
			Gate.set(0.0);
		}
    	/*TODO figure out order of operations
    	   
    	  TODO camera code 
    	*/
    	
    	if (Stick2.getRawButton(6)) {
    		Shoot1.set(-1.0);
    		Shoot2.set(1.0);
		}//draws in ball
    	
    	if (Stick2.getRawButton(5)) {//arm up
			Arm.set(0.5);
		}
    	if (Stick2.getRawButton(3)) {//arm down
			Arm.set(-0.5);
		}
    	
    	if (countTick) {
			tickCount++;
		}//counts ticks
    	//TODO: rerun auto on stick 2
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	//TODO write test
    }
        
}
