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
 * WARRANTY OF ANY m_KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
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
 */ 
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;

public class Robot extends TimedRobot {

	/* Hardware */
	TalonFX m_motor1 = new TalonFX(1);
	TalonFX m_motor2 = new TalonFX(2);
    
    /* Initialize gains */
	private double m_kP = 0.25, m_kI  = 0.0, m_kD = 0.0, m_kF = 0.045;

	public void robotInit() {

		/* Initialize Smart Dashboard display */
		SmartDashboard.putNumber("Velocity", 0);
		SmartDashboard.putNumber("P", m_kP);
		SmartDashboard.putNumber("I", m_kI);
		SmartDashboard.putNumber("D", m_kD);
		SmartDashboard.putNumber("Feed Forward", m_kF);
		SmartDashboard.putNumber("Error 1", 0);
		SmartDashboard.putNumber("Error 2", 0);


        /* Factory Default all hardware to prevent unexpected behaviour */
		m_motor1.configFactoryDefault();
		m_motor2.configFactoryDefault();
		m_motor2.setInverted(true);
		/* Config sensor used for Primary PID [m_Velocity] */
        m_motor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                            Constants.kPIDLoopIdx, 
                                            Constants.kTimeoutMs);
		m_motor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
											Constants.kPIDLoopIdx, 
											Constants.kTimeoutMs);

        /**
		 * Phase sensor accordingly. 
         * Positive Sensor Reading should match Green (blinm_king) Leds on Talon
         */
		m_motor1.setSensorPhase(false);
		m_motor2.setSensorPhase(false);

		
		/* Config the peak and nominal outputs */
		m_motor1.configNominalOutputForward(0.0, Constants.kTimeoutMs);
		m_motor1.configNominalOutputReverse(0.0, Constants.kTimeoutMs);
		m_motor1.configPeakOutputForward(1, Constants.kTimeoutMs);
		m_motor1.configPeakOutputReverse(0.0, Constants.kTimeoutMs); // Don't go in reverse

		m_motor2.configNominalOutputForward(0.0, Constants.kTimeoutMs);
		m_motor2.configNominalOutputReverse(0.0, Constants.kTimeoutMs);
		m_motor2.configPeakOutputForward(1, Constants.kTimeoutMs);
		m_motor2.configPeakOutputReverse(0.0, Constants.kTimeoutMs); // Don't go in reverse
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		/* Config the m_Velocity closed loop gains in slot0 */
		double velocity = SmartDashboard.getNumber("Velocity", 0); // Get desired m_velocity in RPM
		double kP = SmartDashboard.getNumber("P", 0.25);
		double kI = SmartDashboard.getNumber("I",  0.00);
		double kD = SmartDashboard.getNumber("D", 0.0);
		double kF = SmartDashboard.getNumber("Feed Forward",  0.045);

		m_motor1.config_kF(Constants.kPIDLoopIdx, kF, Constants.kTimeoutMs);
		m_motor1.config_kP(Constants.kPIDLoopIdx, kP, Constants.kTimeoutMs);
		m_motor1.config_kI(Constants.kPIDLoopIdx, kI, Constants.kTimeoutMs);
		m_motor1.config_kD(Constants.kPIDLoopIdx, kD, Constants.kTimeoutMs);
	
		m_motor2.config_kF(Constants.kPIDLoopIdx, kF, Constants.kTimeoutMs);
		m_motor2.config_kP(Constants.kPIDLoopIdx, kP, Constants.kTimeoutMs);
		m_motor2.config_kI(Constants.kPIDLoopIdx, kI, Constants.kTimeoutMs);
		m_motor2.config_kD(Constants.kPIDLoopIdx, kD, Constants.kTimeoutMs);
	
		// Convert RPM to raw units per 100ms
		double targetVelocity_UnitsPer100ms = velocity * 2048 / 600;
		
		// Set m_Velocity setpoint
		m_motor1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
		m_motor2.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

		// Get current speed and convert back to RPM
		int speed1 = m_motor1.getSelectedSensorVelocity() * 600 / 2048;
		SmartDashboard.putNumber("Current Velocity 1", speed1);

		int speed2 = m_motor2.getSelectedSensorVelocity() * 600 / 2048;
		SmartDashboard.putNumber("Current Velocity 2", speed2);

		// Show the current m_Error on the SDB
		SmartDashboard.putNumber("Error 1", m_motor1.getClosedLoopError());
		SmartDashboard.putNumber("Error 2", m_motor2.getClosedLoopError());

	}
}
