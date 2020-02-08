/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

class Constants {

    /**
     * Specify if shooter motors are Falcons. If not, we assume they are NEOs
     */
    public static final boolean useFalcons = false;

	/**
	 * Default Falcon PIDF Gains
	 * 	                                    			 kP    kI   kD   kF    Iz  PeakOut */
    public final static Gains kGains_Falcon = new Gains( 0.25, 0.0, 0.0, 0.045, 0,  1.00);

	/**
	 * Default NEO PIDF Gains
	 * 	                                    		  kP       kI   kD   kF     Iz  PeakOut */
    public final static Gains kGains_NEO = new Gains( 0.00055, 0.0, 0.0, 0.0002, 0,  1.00);

}
