/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

import java.util.List;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final int PDPId = 0;

    public static boolean inAuto = false;

    public static final class ShooterConstants {
        public static final int shooterMasterId = 9; //Shoot1
        public static final int shooterSlaveId = 10; //Shoot2

        public static final int kSlotIdx = 0;
        public static final int kTimeOutMs = 30;
        public static final int kPIDLoopIdx = 0;
        public static final double kP = ((0.05 * 1023.0) / 1001.0) * 2; //(0.05 * 1023.0) / 453.0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = (0.6 * 1023.0) / 15900.0;
        public static final double kPeakOutput = 1; 
        //public static final double targetRpm = 4000;
        public static final double targetVelocity = 15000; //ticks per 100m/s
    }
    public static final class DriveTrainConstants {
        public static final int motorLMasterID = 1;
        public static final int motorLSlaveID = 2;
        public static final int motorRMasterID = 3;
        public static final int motorRSlaveID = 4;

        public static double steer = 0.03; // how hard to turn toward the target
        public static double maxDrive = 0.7; // Simple speed limit so we don't drive too fast
    }
    public static final class ShifterConstants {
        public static enum ShiftState {UP, DOWN, UNKNOWN};
        public static ShiftState shiftState; 

        public static final int forwardChannel = 2; //high
        public static final int reverseChannel = 3; //low
    }
    public static final class LimelightConstants {
        public static final double AimMinCmd = 0.0;
    }
    public static final class IntakeConstants{
        public static final int intakeId = 11; //Intake1

        public static final int forwardChannel = 4;
        public static final int reverseChannel = 5;  //Still unkown
    }
    public static final class FeederConstants{
        public static final int loaderMotorId = 7; //Feed1
        public static final int hopperMotorId = 8; //feed2
    }
    public static final class ClimberConstants {
        public static final int erectorMotorId = 12; //Climb1 in Phoenix
        public static final int motorLeftId = 5;
        public static final int motorRightId = 6;
    }

    public static final class OIConstants {

        public static final int joyRPort = 0;
        public static final int joyLPort = 1;
        public static final int contPort = 2; // port of xbox controller connected
    
    }
    
    public static final class AutoConstants {
        //Conversions for the Neos
        private static final double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
        public static final double ticksToMeters = ( ((15.0/85.0)*(30.0/40.0)) / 1.0 ) * ( (pi * .1524) / 1.0 ); //Correct now
        public static final double hundredMstoSecond = 10.0;

        //Done with characterization, all values seem to be okay
        public static final double ksVolts = 0.124; //PB
        public static final double kvVoltSecondsPerMeter = 1.94; //PB
        public static final double kaVoltSecondsSquaredPerMeter = 0.485; //PB
        public static final double kPDriveVel = 2.43; //Potentially used by AutoAlignShoot & AlightShoot (kpAim in limelight example) 
        public static final double kTrackwidthMeters = -0.02552892613083797; //PB - measured  0.63246 - according to what i found online, ignore the measured and use what characterization says
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        //These I just make up as I go, tho they seem to be ignored?
        public static final double kMaxSpeedMetersPerSecond = .1; //Untuned
        public static final double kMaxAccelerationMetersPerSecondSquared = .15; //Untuned

        // Very Reasonable baseline values for a RAMSETE follower in units of meters and seconds - maybe change later
        public static final double kRamseteB = 2.0; //0 to infinite
        public static final double kRamseteZeta = 0.7; //0 to 1
        public static final boolean kGyroReversed = true; //cuz characterization told me
    }
	public static final class TrajectoryConstants {
        //Initializes Tarjectory configurations and Trajectories
        private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                AutoConstants.ksVolts,
                AutoConstants.kvVoltSecondsPerMeter,
                AutoConstants.kaVoltSecondsSquaredPerMeter
            ),
            AutoConstants.kDriveKinematics,
            10
        );
        private static final TrajectoryConfig config =
   			new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
        	// Add kinematics to ensure max speed is actually obeyed
        	.setKinematics(AutoConstants.kDriveKinematics)
        	// Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
            
		// An example trajectory to follow(Converted to a testing trajectory; documentation is all default).  All units in meters.
        public static final Trajectory sanityS = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of( new Translation2d(1, 1), new Translation2d(2, -1) ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config
        );
        //Straight line trajectory
        public static final Trajectory sanityLine = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), //Starts facing +X
            List.of(),
            new Pose2d(1, 0, new Rotation2d(0)),
            config
        );
        //Left Auto Trajectory
        /*
        public static final Trajectory leftAutoTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
            List.of(new Translation2d(0, 0)  ), 
            new Pose2d(0, 0, new Rotation2d(0)), 
            config);
        */
        //Middle Auto Trajectory
        /*
        public static final Trajectory middleAutoTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
            List.of(new Translation2d(0, 0)  ), 
            new Pose2d(0, 0, new Rotation2d(0)), 
            config);
        */
        //Right Auto Trajectory
        /*
        public static final Trajectory rightAutoTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
            List.of(new Translation2d(0, 0)  ), 
            new Pose2d(0, 0, new Rotation2d(0)), 
            config);
        */
        
    }

}