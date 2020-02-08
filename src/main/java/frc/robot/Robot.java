
/**
 * Description:
 */ 
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot
{

    ISpeedControl m_speedControl;
    Gains m_gains;

	public void robotInit() {

        if (Constants.useFalcons == true)
        {
            m_speedControl = new FalconVelocityPIDF();
            m_gains = Constants.kGains_Falcon;
        }
        else
        {
            m_speedControl = new NEOVelocityPIDF();
            m_gains = Constants.kGains_NEO;
        }    

        /* Initialize Smart Dashboard display */
		SmartDashboard.putNumber("P Gain", m_gains.kP);
		SmartDashboard.putNumber("I Gain", m_gains.kI);
		SmartDashboard.putNumber("D Gain", m_gains.kD);
		SmartDashboard.putNumber("Feed Forward", m_gains.kF);

        SmartDashboard.putNumber("Target Velocity", 0);
        SmartDashboard.putNumber("Current Velocity", 0);
        SmartDashboard.putNumber("Current Output Percent", 0);
    	SmartDashboard.putNumber("Error", 0);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

       // read PID coefficients from SmartDashboard
       double kP = SmartDashboard.getNumber("P Gain", 0);
       double kI = SmartDashboard.getNumber("I Gain", 0);
       double kD = SmartDashboard.getNumber("D Gain", 0);
       double kF = SmartDashboard.getNumber("Feed Forward", 0);

       // Update gains on the controller
       m_speedControl.updateGains(kP, kI, kD, kF);

        // Get desired m_velocity in RPM from SmartDasboard
        double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
       
		// Update the target velocity and get back the current velocity
        int currentVelocity = m_speedControl.runVelocityPIDF(targetVelocity);

		// Show the Current Velocity, Error, and Current Output Percent on the SDB
        SmartDashboard.putNumber("Current Velocity", currentVelocity);
		SmartDashboard.putNumber("Error", m_speedControl.getError());
        SmartDashboard.putNumber("Current Output Percent", m_speedControl.getOutputPercent());

	}
}
