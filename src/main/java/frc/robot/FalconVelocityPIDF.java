/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;

/**
 * Add your docs here.
 */
public class FalconVelocityPIDF implements ISpeedControl
{
    /* Hardware */
    TalonFX m_motor1 = new TalonFX(1);
    TalonFX m_motor2 = new TalonFX(2);

    /* Gains */
    double m_kP = 0.0;
    double m_kI = 0.0;
    double m_kD = 0.0;
    double m_kF = 0.0;

    public FalconVelocityPIDF()
    {

        /* Factory Default all hardware to prevent unexpected behaviour */
        m_motor1.configFactoryDefault();
        m_motor2.configFactoryDefault();

        /* Config sensor used for Primary PID [m_Velocity] */
        m_motor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

        /**
		 * Phase sensor accordingly. 
         * Positive Sensor Reading should match Green (blinm_king) Leds on Talon
         */
		m_motor1.setSensorPhase(false);
		m_motor2.setSensorPhase(false);
		
		/* Config the peak and nominal outputs */
		m_motor1.configNominalOutputForward(0.0, 30);
		m_motor1.configNominalOutputReverse(0.0, 30);
		m_motor1.configPeakOutputForward(1.0, 30);
		m_motor1.configPeakOutputReverse(0.0, 30); // Don't go in reverse

		m_motor2.configNominalOutputForward(0.0, 30);
		m_motor2.configNominalOutputReverse(0.0, 30);
		m_motor2.configPeakOutputForward(1.0, 30);
		m_motor2.configPeakOutputReverse(0.0, 30); // Don't go in reverse

        /* Invert motor2 and have it follow motor1 */
        m_motor2.setInverted(true);
        m_motor2.follow(m_motor1);
    }

    public void updateGains(double kP, double kI, double kD, double kF)
    {
        // if PIDF coefficients have changed, write new values to controller
        if((m_kP != kP)) { m_motor1.config_kP(0, kP, 30); m_kP = kP; }
        if((m_kI != kI)) { m_motor1.config_kI(0, kI, 30); m_kI = kI; }
        if((m_kD != kD)) { m_motor1.config_kD(0, kD, 30); m_kD = kD; }
        if((m_kF != kF)) { m_motor1.config_kF(0, kF, 30); m_kF = kF; }
    }

    public int runVelocityPIDF(double targetVelocity)
    {
        // Convert RPM to raw units per 100ms
        double targetVelocity_UnitsPer100ms = targetVelocity * 2048 / 600;
                
        // Set Velocity setpoint
        m_motor1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

        // Get current speed and convert back to RPM
        return (int)((double)m_motor1.getSelectedSensorVelocity() * 600 / 2048);
    }

    public int getError()
    {
        return m_motor1.getClosedLoopError();
    }

    public double getOutputPercent()
    {
		return (m_motor1.getMotorOutputPercent() * 100);
    }
}
