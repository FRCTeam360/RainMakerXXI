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
        public static final int shooterMasterId = 5;
        public static final int shooterSlaveId = 6;

        public static final int kSlotIdx = 0;
        public static final int kTimeOutMs = 30;
        public static final int kPIDLoopIdx = 0;
        public static final double kP = ((0.05 * 1023.0) / 1001.0) * 2; //(0.05 * 1023.0) / 453.0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = (0.6 * 1023.0) / 15900.0;
        public static final double kPeakOutput = 1; 
        //public static final double targetRpm = 4000;
    }
    public static final class DriveTrainConstants {
        public static final int motorLMasterID = 1;
        public static final int motorLSlaveID = 2;
        public static final int motorRMasterID = 3;
        public static final int motorRSlaveID = 4;
    }
    public static final class ShifterConstants {
        public static enum ShiftState {UP, DOWN, UNKNOWN};
        public static ShiftState shiftState; 

        public static final int forwardChannel = 1;
        public static final int reverseChannel = 0;

        public static final boolean isInAutoShift = false; //to ensure you don't shift when your in autonomous 
    }

    public static final class IntakeConstants{
        public static final int intakeId = 5;

        public static final int forwardChannel = 2;
        public static final int reverseChannel = 3;
    }

    public static final class OIConstants {

        public static final int joyRPort = 0;
        public static final int joyLPort = 1;
        public static final int contPort = 2; // port of xbox controller connected
    
    }
    
    public static final class AutoConstants {
        //Which auto do you want?
        public static enum AutoType {LEFT, MIDDLE, RIGHT};
        public static AutoType autoType = AutoType.LEFT; //Change here for whichever auto you would like 

        //ALL OF THESE R EXAMPLE VALUES, WE NEED TO DO CHARACTERIZATION TO FIGURE IT OUT.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final double kPDriveVel = 8.5;
        public static final double kTrackwidthMeters = 0.69; //How wide the wheels r apart
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final boolean kGyroReversed = false;
    }

}