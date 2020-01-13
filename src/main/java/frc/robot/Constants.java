/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final int PDPId = 0;

    public static final class ShooterConstants {
        public static final int shooterMasterId = 0;
        public static final int shooterSlaveId = 1;

        public static final int kSlotIdx = 0;
        public static final int kTimeOutMs = 30;
        public static final int kPIDLoopIdx = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = (1023.0 / 50000); //50000 is measured from our motors, this is max purple velocity from pheonix tuner. Changed 1023 to 1 because that is now what is considered "full power" by CTRE
        public static final double kPeakOutput = 1; 
        //public static final double targetRpm = 4000;
    }

    public static final class OIConstants {

        public static final int joy1Port = 0;
    
    }
}