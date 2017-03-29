import java.io.IOException;

import lejos.nxt.Battery;
import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.addon.ColorHTSensor;
import lejos.nxt.addon.CompassHTSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.PilotProps;

public class StandardRobot {
	
	public static ColorHTSensor colorPrimary, colorSecondary;
	public static UltrasonicSensor ultraSonic;
	public static CompassHTSensor compass;
	public static NXTRegulatedMotor motorA, motorB;
	public static NXTMotor leftMotor;
	public static NXTMotor rightMotor;
	public static DifferentialPilot pilot;
	
	public static int primaryColor;
	public static int secondaryColor;
	public static float ultraSonicValue;
	public static float compassValue;
	
	public static final String A = "North-West";
	public static final String B = "North-East";
	public static final String C = "West-South";
	public static final String D = "South-East";
	public static final double lowEnergyThreshold = 1.5;
	public static final double highEnergyThreshold = 8;
	public static final String lowEnergyCode = "EL";
	public static final String highEnergyCode = "EH";
	public static final float obstacleDist = 8; 
	public static final int greenColorCode = 1;
	public static final int yellowColorCode = 3;
	public static final int redColorCode = 0;
	/*
	 * Class constructor
	 */
	public StandardRobot() throws IOException {
		ultraSonic = new UltrasonicSensor(SensorPort.S3);
		colorPrimary = new ColorHTSensor(SensorPort.S1);
		colorSecondary = new ColorHTSensor(SensorPort.S4);
		compass = new CompassHTSensor(SensorPort.S2);
		
		// instantiate motors
		//motorA = new NXTRegulatedMotor(MotorPort.A);
		// 	motorB = new NXTRegulatedMotor(MotorPort.B);
		leftMotor = new NXTMotor(MotorPort.A);
		rightMotor = new NXTMotor(MotorPort.B);
		
		/*PilotProps pp = new PilotProps();
		pp.loadPersistentValues();
		float wheelDiameter = Float.parseFloat(pp.getProperty(PilotProps.KEY_WHEELDIAMETER, "5.6"));
		float trackWidth = Float.parseFloat(pp.getProperty(PilotProps.KEY_TRACKWIDTH, "16.0"));
		RegulatedMotor leftMotor = PilotProps.getMotor(pp.getProperty(PilotProps.KEY_LEFTMOTOR, "B"));
		RegulatedMotor rightMotor = PilotProps.getMotor(pp.getProperty(PilotProps.KEY_RIGHTMOTOR, "C"));
		boolean reverse = Boolean.parseBoolean(pp.getProperty(PilotProps.KEY_REVERSE,"false"));
		DifferentialPilot pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor,rightMotor,reverse);*/
		pilot = new DifferentialPilot(2.1f, 4.4f, Motor.A, Motor.C, true);  // parameters in inches
	}
	
	/*
	 * Class Member Functions
	 */
	/*public float getCompassValue() {
		return compass.getDegrees();
	}
	
	public float getUltraSonicValue() {
		return ultraSonic.getRange();
	}
	
	public int getPrimaryColorSensorValue() {
		return colorPrimary.getColorID();
	}
	
	public int getSecondaryColorSensorValue() {
		return colorSecondary.getColorID();
	}
	
	public StandardRobot getSensorVales() throws IOException {
		StandardRobot snr = new StandardRobot();
		snr.primaryColor = snr.getPrimaryColorSensorValue();
		snr.secondaryColor = snr.getSecondaryColorSensorValue();
		snr.ultraSonicValue = snr.getUltraSonicValue();
		snr.compassValue = snr.getCompassValue();
		
		return snr;
	}
*/
	/*
	 * To check energy level in Robot
	 */
/*	public String getEnergyLevel() {
		double batteryVoltage = Battery.getVoltage();
		if (batteryVoltage < lowEnergyThreshold ) {
			return lowEnergyCode;
		} else if (batteryVoltage > highEnergyThreshold) {
			return highEnergyCode;
		} else {
			return "false";
		}
	}*/
	
	/*
	 * To check is there any obstacle in front
	 */
	/*public boolean isObstacleFront() throws IOException {
		float distance = new StandardRobot().getUltraSonicValue();
		if(distance <= obstacleDist) {
			return true;
		} else {
			return false;
		}
	}
	*/
	/*
	 * To check whether the color of the object is green 
	 */
	/*public boolean isGreenObject() throws IOException {
		int colorID = new StandardRobot().getPrimaryColorSensorValue();
		if (colorID == greenColorCode) {
			return true;
		} else {
			return false;
		}
	}
	*/
	/*
	 * To check whether the color of the object is Yellow 
	 */
	/*public boolean isYellowObject() throws IOException {
		int colorID = new StandardRobot().getPrimaryColorSensorValue();
		if (colorID == yellowColorCode) {
			return true;
		} else {
			return false;
		}
	}
	*/
	/*
	 * To check whether the color of the object is Red 
	 */
	/*public boolean isRedObject() throws IOException {
		int colorID = new StandardRobot().getPrimaryColorSensorValue();
		if (colorID == redColorCode) {
			return true;
		} else {
			return false;
		}
	}
	*/
	/*
	 * Move Forward
	 */
	public void moveForward() {
		pilot.forward();
	}
	
	/*
	 * Move Backward
	 */
	public void moveBackward() {
		pilot.backward();
	}
	
	/*
	 * Turn Right
	 */
	public void turnRight() {
		System.out.print("nn");
		pilot.rotateLeft();
	}
	
	/*
	 * Turn Left
	 */
	public void turnLeft() {
		System.out.print("nn");

		pilot.rotateRight();
	}
	
	/*
	 * Move Robot at point A, B, C, D
	 */
	
	public void moveRobotCorner(String point) {
		
	}
	
	public static void main(String[] args) throws IOException {
		
		System.out.print("Start of program");
		//Button.waitForAnyPress();
		StandardRobot obj = new StandardRobot();
		obj.turnLeft();
		Button.waitForAnyPress();
		obj.turnRight();
		Button.waitForAnyPress();
		obj.moveForward();
		Button.waitForAnyPress();
		obj.moveBackward();
		Button.waitForAnyPress();
		/*
		 *
		 * 	StandardRobot sensorObj = obj.getSensorVales();
			System.out.print("Color1: "+sensorObj.primaryColor);
			System.out.print("Color2: "+sensorObj.secondaryColor);
			System.out.print("us: "+sensorObj.ultraSonicValue);
			System.out.print("compass: "+sensorObj.compassValue);
		*
		*/
		
		
		
		Button.waitForAnyPress();
	}
}