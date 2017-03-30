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
	
	public StandardRobot() {
		// instantiate sensors
		try 
		{
			us = new UltrasonicSensor(SensorPort.S2);
			floorSensor = new ColorHTSensor(SensorPort.S4);
			ballSensor = new ColorHTSensor(SensorPort.S3);
			cps = new CompassHTSensor(SensorPort.S1);
			// instantiate motors
			leftMotor = new NXTRegulatedMotor(MotorPort.A);
			rightMotor = new NXTRegulatedMotor(MotorPort.B);
			clawMotor = new NXTRegulatedMotor(MotorPort.C);
			// instantiate Pilot
			pilot = new DifferentialPilot(Consts.WHEEL_DIA, Consts.TRACK_WIDTH, leftMotor, rightMotor, true);

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
	
	public void goToCorner(char Loc) {
		//Pending
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
	
}