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


    public static final class DriveTrainConstants {
        public static final int motorLMasterID = 1;
        public static final int motorLSlaveID = 2;
        public static final int motorRMasterID = 3;
        public static final int motorRSlaveID = 4;
    }
    public static final class ShifterConstants {
        public static enum ShiftState {UP, DOWN, UNKNOWN};
        public static ShiftState shiftState; 

        public static final int forwardChannel = 5;
        public static final int reverseChannel = 4;

        public static final boolean isInAutoShift = false; //to ensure you don't shift when your in autonomous 
    }
    public static final class ClimberConstants { //Needs to be changed to proper values (just guessing right now)
        public static final int erectorMotorId = 4;
        public static final int motorMasterId = 5;
        public static final int motorSlaveId = 6;
    }


    public static final class OIConstants {

        public static final int joyRPort = 0;
        public static final int joyLPort = 1;
        public static final int contPort = 2; // port of xbox controller connected
    
    }

}