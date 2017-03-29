import java.io.Serializable;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.addon.ColorHTSensor;
import lejos.nxt.addon.CompassHTSensor;

public class Sensor {
	
	public static ColorHTSensor colorPrimary, colorSecondary;
	public static UltrasonicSensor ultraSonic;
	public static CompassHTSensor compass;
	public static NXTRegulatedMotor motorA, motorB;
	public static NXTMotor leftMotor;
	public static NXTMotor rightMotor;
	
	public static int primaryColor;
	public static int secondaryColor;
	public static float ultraSonicValue;
	public static float compassValue;
	
	
	/*
	 * Class constructor
	 */
	public Sensor() {
		ultraSonic = new UltrasonicSensor(SensorPort.S3);
		colorPrimary = new ColorHTSensor(SensorPort.S1);
		colorSecondary = new ColorHTSensor(SensorPort.S4);
		compass = new CompassHTSensor(SensorPort.S2);
		
		// instantiate motors
		motorA = new NXTRegulatedMotor(MotorPort.A);
		motorB = new NXTRegulatedMotor(MotorPort.B);
		leftMotor = new NXTMotor(MotorPort.A);
		rightMotor = new NXTMotor(MotorPort.B);
	}
	
	/*
	 * Class Member Functions
	 */
	public float getCompassValue() {
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
	
	public Sensor getSensorVales() {
		Sensor snr = new Sensor();
		snr.primaryColor = snr.getPrimaryColorSensorValue();
		snr.secondaryColor = snr.getSecondaryColorSensorValue();
		snr.ultraSonicValue = snr.getUltraSonicValue();
		snr.compassValue = snr.getCompassValue();
		
		return snr;
	}
	
	public static void main(String[] args) {
		
		System.out.print("Start of program");
		Sensor obj = new Sensor();
		obj.getSensorVales();
		
		Button.waitForAnyPress();
	}
}
