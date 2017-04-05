package robotcontroller;

import java.io.IOException;

import lejos.nxt.Battery;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.addon.ColorHTSensor;
import lejos.nxt.addon.CompassHTSensor;
import lejos.robotics.navigation.DifferentialPilot;
import util.Consts;
import util.Consts.CORNER;

public class StandardRobot {
	public static ColorHTSensor ballSensor, floorSensor;
	public static CompassHTSensor cps;
	public static UltrasonicSensor us;
	public static NXTRegulatedMotor leftMotor, rightMotor;
	public static NXTRegulatedMotor clawMotor;
	public static DifferentialPilot pilot;
	public static int clawStatus = Consts.CLAWOPEN; // 1 for open and 0 for close
	public static char currentCorner = ' ';
	
	// corner variables
	
	public StandardRobot() {
		// instantiate sensors
		try 
		{
			cps = new CompassHTSensor(SensorPort.S1);
			us = new UltrasonicSensor(SensorPort.S2);
			ballSensor = new ColorHTSensor(SensorPort.S3);
			floorSensor = new ColorHTSensor(SensorPort.S4);

			// instantiate motors
			leftMotor = new NXTRegulatedMotor(MotorPort.A);
			rightMotor = new NXTRegulatedMotor(MotorPort.B);
			clawMotor = new NXTRegulatedMotor(MotorPort.C);
			// instantiate Pilot
			pilot = new DifferentialPilot(Consts.WHEEL_DIA, Consts.TRACK_WIDTH, leftMotor, rightMotor, false);

		} 
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	public void moveForward()   {
		pilot.forward();
	}
	
	public void moveBackward()  {
		pilot.backward();
	}
	
	public void turnRight() {
		pilot.rotate(Consts.TURN_RIGHT);
	}
	
	public void turnLeft() {
		pilot.rotate(Consts.TURN_LEFT);
	}
	
	public void pickObject() {
		clawMotor.rotate(Consts.CLAW_TURN);
		clawStatus = Consts.CLAWCLOSE;
	}
	
	public void dropObject() {
		clawMotor.rotate(-(Consts.CLAW_TURN));
		clawStatus = Consts.CLAWOPEN;
	}
	
	public float[] getCorderDetails(char Loc) {
		/*
		-----------------------------
		|R (A)					G(B)|
		|			  W	270			|
		|			  |				|
		| 180 S-------|-------N 0	|
		|			  |				|
		|			  E 90			|
		|Ch						Y(C)|
		-----------------------------
								
		
		*/
		
		float cornerAngle[] = new float[2];
		switch (Loc) {
		case 'A':
				cornerAngle[0] = 197.0f;
				cornerAngle[1] = 287.0f;
			break;
		case 'B':
			cornerAngle[0] = 287.0f;
			cornerAngle[1] = 0.0f;
			break;
		case 'C':
			cornerAngle[0] = 0.0f;
			cornerAngle[1] = 90.0f;
			break;
			
		case 'D': // D means charging Point
			cornerAngle[0] = 90.0f;
			cornerAngle[1] = 197.0f;
			break;

		default:
			break;
		}
		return cornerAngle;
	}
	
	public static void goToCornerAB(char Loc) {
		leftMotor.setAcceleration(Consts.MOTOR_ACCELERATION);
		rightMotor.setAcceleration(Consts.MOTOR_ACCELERATION);
		leftMotor.setSpeed(Consts.ROTATION_SPEED);
		rightMotor.setSpeed(Consts.ROTATION_SPEED);
		
		
		// part 1
		float magneticLocation = cps.getDegrees();
		float angle;
		if(magneticLocation >= 90.0 && magneticLocation <= 287.0) {
			rightMotor.backward();
			leftMotor.forward();
			while(true){
				angle = cps.getDegrees();
				if(angle <= 290.0 && angle >=284.0){
					rightMotor.stop();
					leftMotor.stop();
					System.out.println(angle);
					break;
				}
				
			}
			
		} else {
			leftMotor.backward();
			rightMotor.forward();
			while(true){
				angle = cps.getDegrees();
				if(angle <= 290.0 && angle >=284.0){
					rightMotor.stop();
					leftMotor.stop();
					System.out.println(angle);
					break;
				}
				
			}
			
		}
		
		//Button.waitForAnyPress();
		// part 2
		StandardRobot sbTemp = new StandardRobot();
		float dist = sbTemp.getUltraSonicValue();
		while(sbTemp.getUltraSonicValue() > Consts.OBST_DIST_THRES){
			leftMotor.setSpeed(Consts.MOVE_SPEED);
			rightMotor.setSpeed(Consts.MOVE_SPEED);
			leftMotor.forward();
			rightMotor.forward();
			System.out.println("dist "+sbTemp.getUltraSonicValue());
		}
		
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.setSpeed(Consts.ROTATION_SPEED);
		rightMotor.setSpeed(Consts.ROTATION_SPEED);
		
		// part 3
		//Button.waitForAnyPress();
		float magneticLocation1 = cps.getDegrees();
		float angle1;
		if(magneticLocation1 >= 90.0 && magneticLocation1 <= 287.0) {
			rightMotor.backward();
			leftMotor.forward();
			while(true){
				angle1 = cps.getDegrees();
				if(angle1 <= 290.0 && angle1 >=284.0){
					rightMotor.stop();
					leftMotor.stop();
					System.out.println(angle1);
					break;
				}
				
			}
			
		} else {
			leftMotor.backward();
			rightMotor.forward();
			while(true){
				angle = cps.getDegrees();
				if(angle <= 290.0 && angle >=284.0){
					rightMotor.stop();
					leftMotor.stop();
					System.out.println(angle);
					break;
				}
				
			}
			
		}
		
		// part 4
		if (Loc == 'A') {
			sbTemp.turnLeft();
		} else if (Loc == 'B') {
			sbTemp.turnRight();
		}
		
		leftMotor.setSpeed(Consts.MOVE_SPEED);
		rightMotor.setSpeed(Consts.MOVE_SPEED);
		while(sbTemp.getUltraSonicValue() > Consts.OBST_DIST_THRES ){
			
			leftMotor.forward();
			rightMotor.forward();
			System.out.println("dist "+ sbTemp.getUltraSonicValue());
		}
		leftMotor.stop();
		rightMotor.stop();
	}
	
	public static void goToCornerCD(char Loc) {
		leftMotor.setAcceleration(Consts.MOTOR_ACCELERATION);
		rightMotor.setAcceleration(Consts.MOTOR_ACCELERATION);
		leftMotor.setSpeed(Consts.ROTATION_SPEED);
		rightMotor.setSpeed(Consts.ROTATION_SPEED);
		
		
		// part 1
		float magneticLocation = cps.getDegrees();
		float angle;
		if(magneticLocation >= 90.0 && magneticLocation <= 287.0) {
			rightMotor.forward();
			leftMotor.backward();
			while(true){
				angle = cps.getDegrees();
				if(angle <= 93.0 && angle >=87.0){
					rightMotor.stop();
					leftMotor.stop();
					System.out.println(angle);
					break;
				}
				
			}
			
		} else {
			leftMotor.forward();
			rightMotor.backward();
			while(true){
				angle = cps.getDegrees();
				if(angle <= 93.0 && angle >=87.0){
					rightMotor.stop();
					leftMotor.stop();
					System.out.println(angle);
					break;
				}
				
			}
			
		}
		
		//Button.waitForAnyPress();
		// part 2
		StandardRobot sbTemp = new StandardRobot();
		float dist = sbTemp.getUltraSonicValue();
		while(sbTemp.getUltraSonicValue() > Consts.OBST_DIST_THRES){
			leftMotor.setSpeed(Consts.MOVE_SPEED);
			rightMotor.setSpeed(Consts.MOVE_SPEED);
			leftMotor.forward();
			rightMotor.forward();
			System.out.println("dist "+sbTemp.getUltraSonicValue());
		}
		
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.setSpeed(Consts.ROTATION_SPEED);
		rightMotor.setSpeed(Consts.ROTATION_SPEED);
		
		// part 3
		//Button.waitForAnyPress();
		float magneticLocation1 = cps.getDegrees();
		float angle1;
		if(magneticLocation1 >= 90.0 && magneticLocation1 <= 287.0) {
			rightMotor.forward();
			leftMotor.backward();
			while(true){
				angle1 = cps.getDegrees();
				if(angle1 <= 93.0 && angle1 >=87.0){
					rightMotor.stop();
					leftMotor.stop();
					System.out.println(angle1);
					break;
				}
				
			}
			
		} else {
			leftMotor.forward();
			rightMotor.backward();
			while(true){
				angle = cps.getDegrees();
				if(angle <= 93.0 && angle >=87.0){
					rightMotor.stop();
					leftMotor.stop();
					System.out.println(angle);
					break;
				}
				
			}
			
		}
		
		// part 4
		if (Loc == 'C') {
			sbTemp.turnLeft();
		} else if (Loc == 'D') {
			sbTemp.turnRight();
		}
		
		leftMotor.setSpeed(Consts.MOVE_SPEED);
		rightMotor.setSpeed(Consts.MOVE_SPEED);
		while(sbTemp.getUltraSonicValue() > Consts.OBST_DIST_THRES ){
			
			leftMotor.forward();
			rightMotor.forward();
			System.out.println("dist "+ sbTemp.getUltraSonicValue());
		}
		leftMotor.stop();
		rightMotor.stop();
	}

	public void goToCorner(char Loc) {
		
		/*
			-----------------------------
			|R (A)					G(B)|
			|			  W	287			|
			|			  |				|
			| 197 S-------|-------N 0	|
			|			  |				|
			|			  E 90			|
			|Ch						Y(C)|
			-----------------------------
		*/
		
		switch (Loc) {
			case 'A':
				goToCornerAB('A');
				dropObject();
				break;
			case 'B':
				System.out.println("Inside switch");
				goToCornerAB('B');
				dropObject();
				break;
				
			case 'C':
				goToCornerCD('C');
				dropObject();
				break;
			
			case 'D':
				goToCornerCD('D');
				dropObject();
				break;
	
			default:
				break;
		}
	}
	
	public float getEnergy() {
		float batteryVoltage = Battery.getVoltage();
		return batteryVoltage;
	}
	
	public boolean isObstableFront() {
		float distance = new StandardRobot().getUltraSonicValue();
		if(distance <= Consts.OBST_DIST_THRES) {
			return true;
		} else {
			return false;
		}
	}
	
	public int getColorObject() {
		int col=ballSensor.getColorID();
		if(col == Consts.RED)
			return Consts.MYCOLOR.RED.ordinal();
		else if(col == Consts.YELLOW)
			return Consts.MYCOLOR.YELLOW.ordinal();
		else if(col == Consts.GREEN)
			return Consts.MYCOLOR.GREEN.ordinal();
		return col;
	}
	
	public int getColorFloor() {
		int col=floorSensor.getColorID();
		if(col == Consts.RED)
			return Consts.MYCOLOR.RED.ordinal();
		else if(col == Consts.YELLOW)
			return Consts.MYCOLOR.YELLOW.ordinal();
		else if(col == Consts.GREEN)
			return Consts.MYCOLOR.GREEN.ordinal();
		return col;
	}
	
	public char getCorner() {
		// pending
		char cor = '@';
		return cor;
	}
	
	public int getClawStatus() {
		return clawStatus;
	}
	public void stop() {
		leftMotor.stop();
		rightMotor.stop();
		clawMotor.stop();
	}
	
	public int getColorObjectInt() {
		StandardRobot sb = new StandardRobot();
		int colorID = sb.ballSensor.getColorID();
		return sb.ballSensor.getRGBComponent(colorID);
	}
	
	public int getColorFloorInt() {
		StandardRobot sb = new StandardRobot();
		int colorID = sb.floorSensor.getColorID();
		return sb.floorSensor.getRGBComponent(colorID);
	}
	
	/*
	 * Get sensors values
	 */
	public float getUltraSonicValue() {
		return us.getRange();
	}
	
	public float getCompassValue() {
		return cps.getDegrees();
	}
	
	public void goToCenter() {
		
	}
	
	public static void main(String args[]) {
		System.out.println("Start of the program");
		Button.waitForAnyPress();
		StandardRobot sr = new StandardRobot();
		
		/*while(true) {
			float magnaticAngle = sr.getCompassValue();
			System.out.println(magnaticAngle);
			Button.waitForAnyPress();
		}*/
		
		while(true) {
			sr.goToCorner('A');
			Button.waitForAnyPress();
			sr.goToCorner('A');
			Button.waitForAnyPress();
			sr.goToCorner('B');
			Button.waitForAnyPress();
			sr.goToCorner('B');
			Button.waitForAnyPress();
			sr.goToCorner('C');
			Button.waitForAnyPress();
			sr.goToCorner('C');
			Button.waitForAnyPress();
			sr.goToCorner('D');
			Button.waitForAnyPress();
			sr.goToCorner('D');
			Button.waitForAnyPress();
		}
		
		
//		sr.goToCorner('A');
//		Button.waitForAnyPress();
//		sr.goToCorner('A');
//		Button.waitForAnyPress();
//		sr.goToCorner('A');
//		Button.waitForAnyPress();
//		sr.goToCorner('A');
		/*Button.waitForAnyPress();
		sr.goToCorner('B');
		Button.waitForAnyPress();
		sr.goToCorner('B');
		Button.waitForAnyPress();
		sr.goToCorner('B');
		Button.waitForAnyPress();
		sr.goToCorner('B');
		Button.waitForAnyPress();
		sr.goToCorner('C');
		Button.waitForAnyPress();
		sr.goToCorner('C');
		Button.waitForAnyPress();
		sr.goToCorner('C');
		Button.waitForAnyPress();
		sr.goToCorner('C');
		Button.waitForAnyPress();
		sr.goToCorner('D');
		Button.waitForAnyPress();
		sr.goToCorner('D');
		Button.waitForAnyPress();
		sr.goToCorner('D');
		Button.waitForAnyPress();
		sr.goToCorner('D');*/
	}
}