/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class NEOVelocityPIDF implements ISpeedControl
{

	CANSparkMax m_motor1 = new CANSparkMax(1, MotorType.kBrushless);
	CANSparkMax m_motor2 = new CANSparkMax(2, MotorType.kBrushless);
    CANPIDController m_pidController = new CANPIDController(m_motor1);
    CANEncoder m_encoder1;
    double m_targetVelocity = 0.0;

    /* Gains */
    double m_kP = 0.0;
    double m_kI = 0.0;
    double m_kD = 0.0;
    double m_kF = 0.0;
    
    public NEOVelocityPIDF()
    {

        m_motor1.restoreFactoryDefaults();
        m_motor2.restoreFactoryDefaults();
    
        m_motor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor2.setIdleMode(CANSparkMax.IdleMode.kCoast);

        // Have motor 2 follow motor 1 in inverted mode
        m_motor2.follow(m_motor1, true);
        
        // Invert motor 1 to make positive inputs turn the wheel in the correct direction
        m_motor1.setInverted(true);

        // Encoder object created to display position values
        m_encoder1 = m_motor1.getEncoder();

        m_pidController.setOutputRange(0.0, 1.0); 
        m_pidController.setIZone(0.0);

    }

    public void updateGains(double kP, double kI, double kD, double kF)
    {
        // if PID coefficients have changed, write new values to controller
        if((m_kP != kP)) { m_pidController.setP(kP); m_kP = kP; }
        if((m_kI != kI)) { m_pidController.setI(kI); m_kI = kI; }
        if((m_kD != kD)) { m_pidController.setD(kD); m_kD = kD; }
        if((m_kF != kF)) { m_pidController.setFF(kF); m_kF = kF; }
    }

    public int runVelocityPIDF(double targetVelocity)
    {
        // Set the Velocity setpoint
        m_targetVelocity = targetVelocity;
        m_pidController.setReference(m_targetVelocity, ControlType.kVelocity);

        // Get current speed and return it
        return (int) (m_encoder1.getVelocity());

    }

    public int getError()
    {
        return (int)(m_encoder1.getVelocity() - m_targetVelocity);
    }

    public double getOutputPercent()
    {
		return (m_motor1.getAppliedOutput() * 100);
    }

}
