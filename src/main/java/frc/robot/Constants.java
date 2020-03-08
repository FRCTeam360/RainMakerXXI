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

 //The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants.  
 //This class should not be used for any other purpose.  All constants should be declared globally (i.e. public static). 
 //Do not put anything functional in this class. It is advised that you statically import this.

public final class Constants {
    public static final int PDPId = 0;

    public static boolean inAuto = false;

    public static final class ShooterConstants {
        public static final int shooterMasterId = 9; //Shoot1
        public static final int shooterSlaveId = 10; //Shoot2

        public static final int kSlotIdx = 0;
        public static final int kTimeOutMs = 30;
        public static final int kPIDLoopIdx = 0;
        public static final double kP = 1.0;//((0.05 * 1023.0) / 1001.0) * 7.5 * 2.5; //(0.05 * 1023.0) / 453.0;
        public static final double kI = 0;
        public static final double kD = 1.5;
        public static final double kF = (0.6 * 1023.0) / 15900.0;
        public static final double kPeakOutput = 1; 

        public static final double backupTargetVelocity = 14500; //Constant
        public static double targetVelocity = backupTargetVelocity; //will get changed in the future by limelight subsystem or a command...

        public static final double aVal = 2.697; //Quad Ratic regression values
        public static final double bVal = -52.912;
        public static final double cVal = 14815.146;
    }
    public static final class DriveTrainConstants {
        public static final int motorLMasterID = 1;
        public static final int motorLSlaveID = 2;
        public static final int motorRMasterID = 3;
        public static final int motorRSlaveID = 4;

        public static double steer = 0.25; // how hard to turn toward the target
        public static double maxDrive = 0.7; // Simple speed limit so we don't drive too fast
    }
    public static final class ShifterConstants {
        public static enum ShiftState {UP, DOWN, UNKNOWN};
        public static ShiftState shiftState; 

        public static final int forwardChannel = 2; //high
        public static final int reverseChannel = 3; //low
    }
    public static final class LimelightConstants {
        public static final double AimMinCmd = 0.01;
    }
    public static final class IntakeConstants{
        public static final int intakeId = 11; //Intake1
    }
    public static final class FeederConstants{
        public static final int loaderMotorId = 7; //Feed1
        public static final int hopperMotorId = 8; //feed2
    }
    public static final class ClimberConstants {
        public static final int erectorMotorId = 12; //Climb1 in Phoenix
        public static final int motorLeftId = 6; //Winch2
        public static final int motorRightId = 5; //Winch1
    }

    public static final class OIConstants {
        public static final int joyRPort = 0;
        public static final int joyLPort = 1;
        public static final int contPort = 2; // port of xbox controller connected
    }
    
    public static final class AutoConstants {
        //Conversions for the Neos
        private static final double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
        public static final double ticksToMeters = ( ((15.0/85.0)*(30.0/40.0)) / 1.0 ) * ( (pi * .1524) / 1.0 ); 

        //Values for Ramsete controller - baseline from characterization & manually tuned
        public static final double ksVolts = 0.222; 
        public static final double kvVoltSecondsPerMeter = 1.96; 
        public static final double kaVoltSecondsSquaredPerMeter = 0.473;
        public static final double kPDriveVel = 0.4; 
        public static final double kTrackwidthMeters = 0.6663144130546255; //Value from Characterization, not actual
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        //3,0.5 works and so does 6, 1 if want to increase more only change acceleration as 6 is near max robot velocity
        public static final double kMaxSpeedMetersPerSecond = 6; //6 is great
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.0; //3 seems a bit too fast so 2.5 is meta asf

        //Ramsete Values - 2,.7 are default and these have been tuned by hand
        public static final double kRamseteB = 4.0; //0 to infinite - Agression
        public static final double kRamseteZeta = 0.7; //1.0 //0 to 1 - Masking
        public static final boolean kGyroReversed = true; //Characterization says this isn't necessary but it seems to perform better....
    }
	public static final class TrajConfig { // Remember, can't be named "TrajectoryConfig(s)" cuz thats an imported class
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
        public static final TrajectoryConfig configFwd =
   			new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
        	.setKinematics(AutoConstants.kDriveKinematics) // Add kinematics to ensure max speed is actually obeyed
            .addConstraint(autoVoltageConstraint) // Apply the voltage constraint
            .setReversed(false); //forward
    
        public static final TrajectoryConfig configRev =
            new TrajectoryConfig(
             AutoConstants.kMaxSpeedMetersPerSecond,
             AutoConstants.kMaxAccelerationMetersPerSecondSquared
         )
         .setKinematics(AutoConstants.kDriveKinematics) // Add kinematics to ensure max speed is actually obeyed
         .addConstraint(autoVoltageConstraint) // Apply the voltage constraint
         .setReversed(true); //reversed
    }

    public static final class SanityTrajectories {
        // An example trajectory to follow(Converted to a testing trajectory; documentation is all default).  All units in meters.
        public static final Trajectory sanityS = TrajectoryGenerator.generateTrajectory( //If fails, it's ur AutoConstants not this.
            new Pose2d(0, 0, new Rotation2d(0)), // Start at the origin facing the +X direction
            List.of( new Translation2d(1, 1), new Translation2d(2, -1) ), // Pass through these two interior waypoints, making an 's' curve path
            new Pose2d(3, 0, new Rotation2d(0)), // End 3 meters straight ahead of where we started, facing forward
            TrajConfig.configFwd 
        );
        //Straight line trajectory, runs forward 2 meters
        public static final Trajectory sanityLine = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), //Starts facing +X
            List.of(  ),
            new Pose2d(2, 0, new Rotation2d(0)),
            TrajConfig.configFwd
        );
        //Sanity line to go reverse 1 meter after being ran forward 2
        public static final Trajectory sanityLineRev = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 0, new Rotation2d(0)), 
            List.of(  ),
            new Pose2d(1, 0, new Rotation2d(0)),
            TrajConfig.configRev
        );
    }
    public static final class trenchRunTrajectories {
        public static final Trajectory stageOne = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(  ),
            new Pose2d(-4.1, 0, new Rotation2d(0)),
            TrajConfig.configRev
        );
        public static final Trajectory stagetwo = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-4.1, 0, new Rotation2d(0)), 
            List.of(  ),
            new Pose2d(-2, 1.5, new Rotation2d(0)),
            TrajConfig.configFwd
        );
    }
    public static final class middleRunTrajectories {
        public static final Trajectory theAutoPathFirstStage = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-2, -0.5),
                new Translation2d(-2.7, 0.5) 
            ),
            new Pose2d( -2.55, 3.0, new Rotation2d(-45) ), 
            TrajConfig.configRev
        );
        public static final Trajectory theAutoPathSecondStage = TrajectoryGenerator.generateTrajectory(
            new Pose2d( -2.55, 3.0, new Rotation2d(-45) ), 
            List.of(  ),
            new Pose2d(-1, 2.25, new Rotation2d(0)),
            TrajConfig.configFwd
        );
    }
    public static final class halfMiddleRunTrajectories {
        public static final Trajectory stageOne = TrajectoryGenerator.generateTrajectory( //Auto stage 1 - backup to center
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of( new Translation2d(-1.75, -0.25) ),
            new Pose2d(-2.5, 0.5, new Rotation2d(-45)),
            TrajConfig.configRev
        );
        public static final Trajectory stageTwo = TrajectoryGenerator.generateTrajectory( //Auto stage 2 - forward 2 shooter location
            new Pose2d(-2.5, 0.5, new Rotation2d(-45)),
            List.of( new Translation2d(-1.75, -0.25) ),
            new Pose2d(-1, 0, new Rotation2d(0)), //-2, -1
            TrajConfig.configFwd
        );
    }
    
}




//Week 1 Trajectories, we can get rid of em if we know we won't need to refrence them
 /*
        public static final Trajectory theAutoPathFirstStage = TrajectoryGenerator.generateTrajectory( //Auto stage 1 - backup to center
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of( new Translation2d(-2, -0.5) ),
            new Pose2d(-3, 0.5, new Rotation2d(-45)),
            configRev
        );
        public static final Trajectory theAutoPathSecondStage = TrajectoryGenerator.generateTrajectory( //Auto stage 2 - forward 2 shooter location
            new Pose2d(-3, 0.5, new Rotation2d(-45)),
            List.of(),
            new Pose2d(-2, -1, new Rotation2d(0)), 
            config
        );
        public static final Trajectory trenchRunPathFirstStage = TrajectoryGenerator.generateTrajectory( //Stage 1 trenchRun
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of( 
                new Translation2d(-1.07, .40), //a = 1
                new Translation2d(-2.15, .80), //a = 2
                new Translation2d(-3.23, 1.21), // a = 3
                new Translation2d(-4.31, 1.61), // a = 4
                new Translation2d(-5.39, 2.02) // a = 5
            ),
            new Pose2d(-5.85, 2.19, new Rotation2d(0)),  //a = 5.43
            configRev //reverse path
        );
        public static final Trajectory trenchRunPathSecondStage = TrajectoryGenerator.generateTrajectory( //Stage 2 trenchRun
            new Pose2d(-5.85, 2.19, new Rotation2d(0)),
            List.of(
                new Translation2d(-5.39, 2.02), // a = 5
                new Translation2d(-4.31, 1.61), // a = 4
                new Translation2d(-3.23, 1.21), // a = 3
                new Translation2d(-2.15, .80) //a = 2
                //new Translation2d(-1.07, .40) //a = 1
            ),
            new Pose2d(-1.07, .40, new Rotation2d(0)), 
            config
        );
        */


