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
 * The VelocityClosedLoop example demonstrates the velocity closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 * Use Percent Output Mode (Holding A and using Left Joystick) to confirm talon is driving 
 * forward (Green LED on Talon/Victor) when the postion sensor is moving in the postive 
 * direction. If this is not the case, flip the boolean input in setSensorPhase().
 * 
 * Controls:
 * Button 1: When held, start and run Velocity Closed Loop on Talon/Victor
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon forward and reverse, use to confirm hardware setup
 * 	+ Velocity Closed Loop: Servo Talon forward and reverse [-500, 500] RPM
 * 
 * Gains for Velocity Closed Loop may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;

public class Robot extends TimedRobot {
    /* Hardware */
	TalonFX motor1 = new TalonFX(1);
    
    /* String for output */
    
    /* Loop tracker for prints */
	private double kP = 0.0, kI  = 0.0, kD = 0.0, kF = 0.0;
	private int velocity = 0, error = 0;

	public void robotInit() {
		SmartDashboard.putNumber("Velocity", velocity);
		SmartDashboard.putNumber("P", kP);
		SmartDashboard.putNumber("I", kI);
		SmartDashboard.putNumber("D", kD);
		SmartDashboard.putNumber("Feed Forward", kF);
		SmartDashboard.putNumber("Error", error);

        /* Factory Default all hardware to prevent unexpected behaviour */
      	motor1.configFactoryDefault();

		/* Config sensor used for Primary PID [Velocity] */
        motor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                            Constants.kPIDLoopIdx, 
                                            Constants.kTimeoutMs);

        /**
		 * Phase sensor accordingly. 
         * Positive Sensor Reading should match Green (blinking) Leds on Talon
         */
		motor1.setSensorPhase(false);

		/* Config the peak and nominal outputs */
		motor1.configNominalOutputForward(0.0, Constants.kTimeoutMs);
		motor1.configNominalOutputReverse(0.0, Constants.kTimeoutMs);
		motor1.configPeakOutputForward(1, Constants.kTimeoutMs);
		motor1.configPeakOutputReverse(0.0, Constants.kTimeoutMs); // Don't go in reverse


		/* Config the Velocity closed loop gains in slot0 */

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		double m_velocity = SmartDashboard.getNumber("Velocity", 0); // Get desired velocity in RPM
		double m_kP = SmartDashboard.getNumber("P", 0.25);
		double m_kI = SmartDashboard.getNumber("I",  0.001);
		double m_kD = SmartDashboard.getNumber("D", 20);
		double m_kF = SmartDashboard.getNumber("Feed Forward",  1023.0/7200.0);

		//System.out.println("velocity " + m_velocity);

		motor1.config_kF(Constants.kPIDLoopIdx, m_kF, Constants.kTimeoutMs);
		motor1.config_kP(Constants.kPIDLoopIdx, m_kP, Constants.kTimeoutMs);
		motor1.config_kI(Constants.kPIDLoopIdx, m_kI, Constants.kTimeoutMs);
		motor1.config_kD(Constants.kPIDLoopIdx, m_kD, Constants.kTimeoutMs);
		/* Get Talon/Victor's current output percentage */		
		/* Prepare line to print */

	
		double targetVelocity_UnitsPer100ms = m_velocity;// * 2048 / 600;  // Convert RPM to raw units per 100ms
		
		motor1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
		
		int speed = motor1.getSelectedSensorVelocity();// * 600 / 2048; // Get speed and convert back to RPM
		SmartDashboard.putNumber("Current Velocity", speed);

		SmartDashboard.putNumber("Error", motor1.getClosedLoopError());
	}
}
