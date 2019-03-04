/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The PositionClosedLoop example demonstrates the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 * Use Percent Output Mode (Holding A and using Left Joystick) to confirm talon is driving 
 * forward (Green LED on Talon/Victor) when the position sensor is moving in the postive 
 * direction. If this is not the case, flip the boolean input in setSensorPhase().
 * 
 * Controls:
 * Button 1: When pressed, start and run Position Closed Loop on Talon/Victor
 * Button 2: When held, start and run Percent Output
 * Left Joytick Y-Axis:
 * 	+ Position Closed Loop: Servo Talon forward and reverse [-10, 10] rotations
 * 	+ Percent Ouput: Throttle Talon forward and reverse
 * 
 * Gains for Position Closed Loop may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.*;


public class Robot extends TimedRobot {
    /** Hardware */
	TalonSRX _talon1 = new TalonSRX(1);
	TalonSRX _talon2 = new TalonSRX(8);
	TalonSRX _talon3 = new TalonSRX(9);
	TalonSRX _talon4 = new TalonSRX(10);
	VictorSP _victor1 = new VictorSP(0);
	VictorSP _victor2 = new VictorSP(1);
	VictorSP _victor3 = new VictorSP(2);
	VictorSP _victor4 = new VictorSP(3);
	Spark _spark1 = new Spark(4);
	Spark _spark2 = new Spark(5);
	Spark _spark3 = new Spark(6);
	Joystick _joy = new Joystick(0);
	double setValue=0;
	DoubleSolenoid deneme = new DoubleSolenoid(0,1);
    /** Used to create string thoughout loop */
	StringBuilder _sb = new StringBuilder();
	SpeedControllerGroup m_left = new SpeedControllerGroup(_victor3, _victor2);
	SpeedControllerGroup m_right = new SpeedControllerGroup(_victor4, _victor1);
	DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
	int _loops = 0;
	
    /** Track button state for single press event */
	boolean _lastButton1 = false;

	/** Save the target position to servo to */
	double targetPositionRotations;

	public void robotInit() {


		/* Config the sensor used for Primary PID and sensor direction */
        _talon1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 
                                            Constants.kPIDLoopIdx,
																																												Constants.kTimeoutMs);
								_talon2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 
																																												Constants.kPIDLoopIdx,
																																												Constants.kTimeoutMs);
								_talon3.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 
																																												Constants.kPIDLoopIdx,
																																												Constants.kTimeoutMs);
								_talon4.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 
																																												Constants.kPIDLoopIdx,
																																												Constants.kTimeoutMs);
																							

		/* Ensure sensor is positive when output is positive */
		_talon1.setSensorPhase(Constants.kSensorPhase);
		_talon2.setSensorPhase(Constants.kSensorPhase);
		_talon3.setSensorPhase(Constants.kSensorPhase);
		_talon4.setSensorPhase(Constants.kSensorPhase);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
		_talon1.setInverted(Constants.kMotorInvert);
		_talon2.setInverted(Constants.kMotorInvert);
		_talon3.setInverted(Constants.kMotorInvert);
		_talon4.setInverted(Constants.kMotorInvert);

		/* Config the peak and nominal outputs, 12V means full */
		_talon1.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon1.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon1.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		_talon2.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon2.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon2.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon2.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		_talon3.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon3.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon3.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon3.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		_talon4.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon4.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon4.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon4.configPeakOutputReverse(-1, Constants.kTimeoutMs);






		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_talon1.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon2.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon3.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon4.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		_talon1.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon1.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon1.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon1.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
		_talon2.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon2.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon2.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon2.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
		_talon3.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon3.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon3.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon3.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
		_talon4.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon4.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon4.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon4.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition1 = _talon1.getSensorCollection().getPulseWidthPosition();
		int absolutePosition2 = _talon2.getSensorCollection().getPulseWidthPosition();
		int absolutePosition3 = _talon3.getSensorCollection().getPulseWidthPosition();
		int absolutePosition4 = _talon4.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition1 &= 0xFFF;
		if (Constants.kSensorPhase) { absolutePosition1 *= -1; }
		if (Constants.kMotorInvert) { absolutePosition1 *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		_talon1.setSelectedSensorPosition(absolutePosition1, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon2.setSelectedSensorPosition(absolutePosition2, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon3.setSelectedSensorPosition(absolutePosition3, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon4.setSelectedSensorPosition(absolutePosition4, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		LiveWindow.disableAllTelemetry();
    }
    
	void commonLoop() {
		/* Gamepad processing */
		double leftYstick = _joy.getY();
		boolean button1 = _joy.getRawButton(1);	// X-Button
		boolean button2 = _joy.getRawButton(2);	// A-Button

		/* Get Talon/Victor's current output percentage */
		double motorOutput1 = _talon1.getMotorOutputPercent();
		double motorOutput2 = _talon1.getMotorOutputPercent();
		double motorOutput3 = _talon1.getMotorOutputPercent();
		double motorOutput4 = _talon1.getMotorOutputPercent();
		/* Deadband gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			/* Within 10% of zero */
			leftYstick = 0;
		}

		/* Prepare line to print */
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput1 * 100));
		_sb.append("%");	// Percent

		_sb.append("\tpos:");
		_sb.append(_talon1.getSelectedSensorPosition(0));
		_sb.append("u"); 	// Native units

		/**
		 * When button 1 is pressed, perform Position Closed Loop to selected position,
		 * indicated by Joystick position x10, [-10, 10] rotations
		 */
		if (!_lastButton1 && button1) {
			/* Position Closed Loop */

			/* 10 Rotations * 4096 u/rev in either direction */
			targetPositionRotations = leftYstick * 10.0 * 4096;
			_talon1.set(ControlMode.Position, targetPositionRotations);
			_talon2.set(ControlMode.Position, targetPositionRotations);
			_talon3.set(ControlMode.Position, targetPositionRotations);
			_talon4.set(ControlMode.Position, targetPositionRotations);
		}

		/* When button 2 is held, just straight drive */
		if (button2) {
			/* Percent Output */

			_talon1.set(ControlMode.PercentOutput, leftYstick);
			_talon2.set(ControlMode.PercentOutput, leftYstick);
			_talon3.set(ControlMode.PercentOutput, leftYstick);
			_talon4.set(ControlMode.PercentOutput, leftYstick);
		}

		/* If Talon is in position closed-loop, print some more info */
		if (_talon1.getControlMode() == ControlMode.Position) {
			/* ppend more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_talon1.getClosedLoopError(0));
			_sb.append("u");	// Native Units

			_sb.append("\ttrg:");
			_sb.append(targetPositionRotations);
			_sb.append("u");	/// Native Units
		}

		/**
		 * Print every ten loops, printing too much too fast is generally bad
		 * for performance.
		 */
		if (++_loops >= 50) {
			_loops = 0;
			System.out.println(_sb.toString());
		}

		/* Reset built string for next loop */
		_sb.setLength(0);
		
		/* Save button state for on press detect */
		_lastButton1 = button1;
    }
    
	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		System.out.println(_joy.getRawAxis(0)+"x     y"+_joy.getRawAxis(1));
if (_joy.getRawButton(7)){
	_spark2.set(1);
}
	else if (_joy.getRawButton(8)){
		_spark2.set(-1);
	}

	else{
		_spark2.set(0);
	}


			commonLoop();


	
		
		if(_joy.getRawAxis(0)>0.5){
			_talon1.set(ControlMode.Position, 1400);
			_talon2.set(ControlMode.Position, 1400);
			_talon3.set(ControlMode.Position, 600);
			_talon4.set(ControlMode.Position, 600);
			m_drive.arcadeDrive(-_joy.getRawAxis(4),0);
		}
		else{
			m_drive.arcadeDrive(_joy.getRawAxis(5), -_joy.getRawAxis(4));
			_talon1.set(ControlMode.Position, 0);
			_talon2.set(ControlMode.Position, 0);
			_talon3.set(ControlMode.Position, 0);
			_talon4.set(ControlMode.Position, 0);
		}
		/*else if(_joy.getRawAxis(1)<-0.5){
			/*_talon1.set(ControlMode.Position, 1400);
			_talon2.set(ControlMode.Position, 1400);
			_talon3.set(ControlMode.Position, 600);
			_talon4.set(ControlMode.Position, 600);
			_talon1.set(ControlMode.Position, 0);
			_talon2.set(ControlMode.Position, 0);
			_talon3.set(ControlMode.Position, 0);
			_talon4.set(ControlMode.Position, 0);
		}
		else if(_joy.getRawAxis(0)<-0.5){
			_talon1.set(ControlMode.Position, 2650);
			_talon2.set(ControlMode.Position, 2650);
			_talon3.set(ControlMode.Position, 1150);
			_talon4.set(ControlMode.Position, 1150);
		}
		else if(_joy.getRawAxis(1)>0.5){
			_talon1.set(ControlMode.Position, 3950);
			_talon2.set(ControlMode.Position, 3950);
			_talon3.set(ControlMode.Position, 1700);
			_talon4.set(ControlMode.Position, 1700);
		}
		*/

		
		if(_joy.getRawButton(5)){
			deneme.set(DoubleSolenoid.Value.kForward);
		}
		else if(_joy.getRawButton(6)){
			deneme.set(DoubleSolenoid.Value.kReverse);
		}
		else if(_joy.getRawButton(1)){
			_spark1.set(1);
		}
		else if(_joy.getRawButton(2)){
			_spark1.set(-0.25);
		}
		else {
			_spark1.set(0);
		}
	
		//m_drive.arcadeDrive(-_joy.getRawAxis(5), _joy.getRawAxis(4));
		
	/*
		if(_joy.getRawAxis(0)> 0.25 && _joy.getRawAxis(1)> 0.75 ){
			_talon1.set(ControlMode.Position, 350);
			_talon2.set(ControlMode.Position, 350);
		}
		if(_joy.getRawAxis(0)> 0.50 && _joy.getRawAxis(1)> 0.50 ){
			_talon1.set(ControlMode.Position, 700);
			_talon2.set(ControlMode.Position, 700);
		}
		if(_joy.getRawAxis(0)> 0.75 && _joy.getRawAxis(1)> 0.25 ){
			_talon1.set(ControlMode.Position, 1050);
			_talon2.set(ControlMode.Position, 1050);
		}
		if(_joy.getRawAxis(0)>= 1 && _joy.getRawAxis(1)>= 0 ){
			_talon1.set(ControlMode.Position, 1400);
			_talon2.set(ControlMode.Position, 1400);
		}
		/////////////////////////////////////////////////////////////// Aşağıda büyüktür yerine küçüktür olabilir.
		if(_joy.getRawAxis(0)> -0.75 && _joy.getRawAxis(1)> 0.25 ){
			_talon1.set(ControlMode.Position, 2062.5);
			_talon2.set(ControlMode.Position, 2062.5);
		}
		if(_joy.getRawAxis(0)> -0.50 && _joy.getRawAxis(1)> 0.50 ){
			_talon1.set(ControlMode.Position, 2425);
			_talon2.set(ControlMode.Position, 2425);
		}
		if(_joy.getRawAxis(0)> -0.25 && _joy.getRawAxis(1)> 0.75 ){
			_talon1.set(ControlMode.Position, 3387.5);
			_talon2.set(ControlMode.Position, 3387.5);
		}
		if(_joy.getRawAxis(0)>= 0 && _joy.getRawAxis(1)>= 1 ){
			_talon1.set(ControlMode.Position, 4050);
			_talon2.set(ControlMode.Position, 4050);
		}
		/////////////////////////////////////////////////////////////////
		if(_joy.getRawAxis(0)> -0.25 && _joy.getRawAxis(1)> -0.75 ){<
			_talon1.set(ControlMode.Position, 5037.5);
			_talon2.set(ControlMode.Position, 5037.5);
		}
		if(_joy.getRawAxis(0)> -0.50 && _joy.getRawAxis(1)> -0.50 ){
			_talon1.set(ControlMode.Position, 6025);
			_talon2.set(ControlMode.Position, 6025);
		}
		if(_joy.getRawAxis(0)> -0.75 && _joy.getRawAxis(1)> -0.25 ){
			_talon1.set(ControlMode.Position, 6912.5);
			_talon2.set(ControlMode.Position, 6912.5);
		}
		if(_joy.getRawAxis(0)>= 1 && _joy.getRawAxis(1)>= 0 ){
			_talon1.set(ControlMode.Position, 8000);
			_talon2.set(ControlMode.Position, 8000);
		}
		///////////////////////////////////////////////////////////////////

		*/
		/*
		double angleValueX = _joy.getMagnitude();
		angleValueX = Math.toRadians(angleValueX);
		double angleValueY = _joy.getRawAxis(0);
		angleValueY = Math.toRadians(angleValueY);
		double ortalama = ((_joy.getRawAxis(0) + _joy.getRawAxis(1))/2.0)*500;		
		System.out.println(ortalama);
		_talon1.set(ControlMode.Position, ortalama);
		*/

		
		/*if(_joy.getRawAxis(1)>0.7){
			setValue++;
		}
		else if (_joy.getRawAxis(1)<-0.7){
			setValue--;
		}
		_talon1.set(ControlMode.Position, setValue*30);
	*/

	}
	
}