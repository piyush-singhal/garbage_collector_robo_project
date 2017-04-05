package robotcontroller;

import lejos.nxt.Battery;
import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.addon.ColorHTSensor;
import lejos.nxt.addon.CompassHTSensor;
import lejos.robotics.navigation.DifferentialPilot;
import util.Consts;

public class StandardRobot {
	public static ColorHTSensor ballSensor, floorSensor;
	public static CompassHTSensor cps;
	public static UltrasonicSensor us;
	public static NXTRegulatedMotor leftMotor, rightMotor;
	public static NXTRegulatedMotor clawMotor;
	public static DifferentialPilot pilot;
	public static int clawStatus = Consts.CLAWOPEN; // 1 for open and 0 for close
	public static char currentCorner = ' ';
	
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
	
	public void moveForward() {
		setMoveSpeed();
		leftMotor.forward();
		rightMotor.forward();
	}
	
	public void moveBackward() {
		setMoveSpeed();
		leftMotor.backward();
		rightMotor.backward();
	}
	
	public void turnRight() {
		pilot.rotate(Consts.TURN_RIGHT);
	}
	
	public void turnLeft() {
		pilot.rotate(Consts.TURN_LEFT);
	}
	
	public void pickObject() {
		// Vinod Code
	}
	
	public void dropObject() {
		// Vinod Code
	}
	
	public void setMotorAcc() {
		leftMotor.setAcceleration(Consts.MOTOR_ACCELERATION);
		rightMotor.setAcceleration(Consts.MOTOR_ACCELERATION);
	}
	
	public void setRotationSpeed() {
		leftMotor.setSpeed(Consts.ROTATION_SPEED);
		rightMotor.setSpeed(Consts.ROTATION_SPEED);
	}
	
	public void setMoveSpeed() {
		leftMotor.setSpeed(Consts.MOVE_SPEED);
		rightMotor.setSpeed(Consts.MOVE_SPEED);
	}
	
	public void rotateForCornerAB() {
		setRotationSpeed();
		float magneticLocation = getCompassValue();
		if(magneticLocation >= Consts.EAST && magneticLocation <= Consts.WEST) {
			leftMotor.forward();
			rightMotor.backward();
			while(getCompassValue() >= Consts.AB_ANGLE + Consts.ANGLE_THRES 
					|| getCompassValue() <=Consts.AB_ANGLE - Consts.ANGLE_THRES) {
				// loop until doesn't reach to right direction
			}
			stop();
		} else {
			leftMotor.backward();
			rightMotor.forward();
			while(getCompassValue() >= Consts.AB_ANGLE + Consts.ANGLE_THRES 
					|| getCompassValue() <=Consts.AB_ANGLE - Consts.ANGLE_THRES) {
				// loop until doesn't reach to right direction
			}
		}
		stop();
	}
	
	public void rotateForCornerCD() {
		setRotationSpeed();
		float magneticLocation = getCompassValue();
		if(magneticLocation >= Consts.EAST && magneticLocation <= Consts.WEST) {
			rightMotor.forward();
			leftMotor.backward();
			while(getCompassValue() >= Consts.CD_ANGLE + Consts.ANGLE_THRES 
					|| getCompassValue() <=Consts.CD_ANGLE - Consts.ANGLE_THRES) {
				// loop until doesn't reach to right direction
			}
			stop();
		} else {
			leftMotor.forward();
			rightMotor.backward();
			while(getCompassValue() >= Consts.CD_ANGLE + Consts.ANGLE_THRES 
					|| getCompassValue() <=Consts.CD_ANGLE - Consts.ANGLE_THRES) {
				// loop until doesn't reach to right direction
			}
		}
		stop();
	}
	
	public void goNearToWall() {
		moveForward();
		while(getUltraSonicValue() > Consts.OBST_DIST_THRES){
			// loop until don't reach to wall
		}
		stop();
	}
	
	
	public void goToCornerAB(char Loc) {
		setMotorAcc();
		// part 1 Rotate in direction to reach corner 
		rotateForCornerAB();
		// part 2 reach to wall
		goNearToWall();
		// part 3 align itself again
		rotateForCornerAB();
		// part 4
		if (Loc == 'A') {
			turnLeft();
		} else if (Loc == 'B') {
			turnRight();
		}
		goNearToWall();
	}
	
	public void goToCornerCD(char Loc) {
		setMotorAcc();
		// part 1 Rotate in direction to reach corner 
		rotateForCornerCD();
		// part 2 reach to wall
		goNearToWall();
		// part 3 align itself again
		rotateForCornerCD();
		// part 4
		if (Loc == 'C') {
			turnLeft();
		} else if (Loc == 'D') {
			turnRight();
		}
		goNearToWall();
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
	}
}