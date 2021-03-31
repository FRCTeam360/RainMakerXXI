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
        public static final double kMaxSpeedMetersPerSecond = 4.0; //6 is great 
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.0; //3 seems a bit too fast so 2.5 is meta asf

        //Ramsete Values - 2,.7 are default and these have been tuned by hand
        public static final double kRamseteB = 4.0; //0 to infinite - Agression
        public static final double kRamseteZeta = 0.7; //1.0 //0 to 1 - Masking
        public static final boolean kGyroReversed = true; //Characterization says this isn't necessary but it seems to perform better....
    }
	public static final class TrajConfig { // Remember, can't be named "TrajectoryConfig(s)" cuz thats an imported class
        //Initializes Trajectory configurations and Trajectories
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
            new Pose2d(1, 0, new Rotation2d(0)),
            TrajConfig.configFwd
        );
        //Sanity line to go reverse 1 meter after being ran forward 2
        public static final Trajectory sanityLineRev = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1, 0, new Rotation2d(0)), 
            List.of(  ),
            new Pose2d(0, 0, new Rotation2d(0)),
            TrajConfig.configRev
        );
    }
    public static final class teachihngTrajectories {
        public static final Trajectory rev = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), //Starts facing +X
            List.of(  ),
            new Pose2d(-2.07, 0.5, new Rotation2d(0)),
            TrajConfig.configRev
        );
        public static final Trajectory fwd = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-2.07, 0.5, new Rotation2d(0)), 
            List.of(  ),
            new Pose2d(1.5, -1.2, new Rotation2d(0)),
            TrajConfig.configFwd
        );
        public static final Trajectory rev2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.5, -1.2, new Rotation2d(0)), //Starts facing +X
            List.of(  ),
            new Pose2d(0, 0, new Rotation2d(0)),
            TrajConfig.configRev
        );
    }
    public static final class BarrelRunTrajectories {
        public static final Trajectory firstExperiment = TrajectoryGenerator.generateTrajectory( //initial, bad cuz random guess pretty much
            new Pose2d(0, 0, new Rotation2d(0)), //Starts facing +X
            List.of( 
                new Translation2d(2.5, -0.20),
                new Translation2d(3.5, -1),
                new Translation2d(2.5, -1.8),
                new Translation2d(2.0, -1),
                new Translation2d(5.5,0),
                new Translation2d(4.5,1),
                new Translation2d(4.25,0)
            ),
            new Pose2d(6, -1.5, new Rotation2d(0)),
            TrajConfig.configFwd
        );
        public static final Trajectory firstJakeNums = TrajectoryGenerator.generateTrajectory( //Jakes raw numbers first shot
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of( 
                new Translation2d(2.0193, 0),
                new Translation2d(3.3147, 0),
                new Translation2d(3.7465, -0.4318),
                new Translation2d(3.3147, -0.8636),
                new Translation2d(2.8829, -0.4318),
                new Translation2d(3.3147, 0),
                new Translation2d(5.7151, 0.6758),
                new Translation2d(6.0325, 1.0922),
                new Translation2d(5.6007, 1.5240),
                new Translation2d(5.1689, 1.0922),
                new Translation2d(6.8194, -.62 ), //Guessed y-val as jake left it unkown
                new Translation2d(7.1247, -0.8636),
                new Translation2d(7.5565, -0.4318),
                new Translation2d(7.1247, 0)
             ),
            new Pose2d(0.25, 0, new Rotation2d(180)),
            TrajConfig.configFwd
        );
        public static final Trajectory firstJakeNumsShift = TrajectoryGenerator.generateTrajectory( //jake nums + shift, no return
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of( 
                new Translation2d(2.0193, 0),
                new Translation2d(3.3147, 0 + 0.1),
                new Translation2d(3.7465 + 0.1, -0.4318),
                new Translation2d(3.3147, -0.8636 + -0.1),
                new Translation2d(2.8829 + -0.1, -0.4318),
                new Translation2d(3.3147, 0 + 0.1),
                new Translation2d(5.7151, 0.6758 + -0.1),
                new Translation2d(6.0325 + 0.1, 1.0922),
                new Translation2d(5.6007, 1.5240 + 0.1),
                new Translation2d(5.1689 + -0.1, 1.0922),
                //new Translation2d(6.8194 + -0.05, -.62 + -0.05 ), //Guessed y-val as jake left it unkown
                new Translation2d(7.1247, -0.8636 + -0.1),
                //new Translation2d(7.4300 + 0.05, 0.7371 + -0.05), //New circle rez point
                new Translation2d(7.5565 + 0.1, -0.4318),
                new Translation2d(7.1247, 0 + 0.1)
            ),
            new Pose2d(0.25, 0, new Rotation2d(0)),
            TrajConfig.configFwd
        );

        public static final Trajectory firstRaw = TrajectoryGenerator.generateTrajectory( //3/1.5 is 14.75 secs - 3.5/1.5 is 13.31 secs - 3/2 is 13.41 sec : settle on velo increase
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of( 
                new Translation2d(3.282, +0.164),
                new Translation2d(3.949, -0.469),
                new Translation2d(3.314, -1.104),
                new Translation2d(2.679, -0.469),
                new Translation2d(3.246, +0.161),
                new Translation2d(5.668, +0.422),
                new Translation2d(6.234, +1.093),
                new Translation2d(5.600, +1.689),
                new Translation2d(4.966, +1.093),
                new Translation2d(5.151, +0.605),
                new Translation2d(6.675, -0.918),
                new Translation2d(7.711, -0.712),
                new Translation2d(7.124, +0.165)
            ),
            new Pose2d(0.533 + 0.5, 0.165 + 0.3, new Rotation2d(0)),
            TrajConfig.configFwd
        );
        public static final Trajectory first = TrajectoryGenerator.generateTrajectory( //v=3.5 & a=1.5 is about 13.31 secs - Tuned Vals
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of( 
                new Translation2d(3.282, +0.164), //ok
                new Translation2d(3.949 +.2, -0.469 -.2),
                new Translation2d(3.314, -1.104 -.2), // y-0.2
                //new Translation2d(3.763, -0.918), //Added interrior circle point
                new Translation2d(2.679 -.1, -0.469),
                new Translation2d(3.246 -.1, +0.161),
                new Translation2d(5.668, +0.422),
                new Translation2d(6.234 +.2, +1.093),
                new Translation2d(5.600, +1.689),
                new Translation2d(4.966 -.2, +1.093),
                //new Translation2d(5.151, +0.605),
                new Translation2d(7.124, -1.104), //6.675, -0.918
                new Translation2d(7.759, -0.469), //7.711 -.1, -0.712
                new Translation2d(7.124, +0.165)
            ),
            new Pose2d(1.000, 1.250, new Rotation2d(0)),
            TrajConfig.configFwd
        );
    }
    public static final class SlalomRunTrajectories {
        public static final Trajectory slalomPathTangent = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of( 
                new Translation2d(1.066, 0.000),
                new Translation2d(2.023, 1.443),
                new Translation2d(2.552, 1.727),
                new Translation2d(5.600, 1.727),
                new Translation2d(6.129, 1.443),
                new Translation2d(6.595, 0.741),
                new Translation2d(7.124, 0.457),
                new Translation2d(7.759, 1.092),
                new Translation2d(7.124, 1.727),
                new Translation2d(6.595, 1.443),
                new Translation2d(6.129, 0.741),
                new Translation2d(5.600, 0.457),
                new Translation2d(2.552, 0.457),
                new Translation2d(2.023, 0.741),
                new Translation2d(1.369, 1.727)
            ),
            new Pose2d(0.152, 1.727, new Rotation2d(0)),
            TrajConfig.configFwd
        );
        public static final Trajectory slalomPathInteriorPoints = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of( 
                new Translation2d(1.263, 0.502),
                new Translation2d(1.557, 0.741),
                new Translation2d(2.023, 1.443 + 0.2), //
                new Translation2d(2.552, 1.727 + 0.2), //
                new Translation2d(5.600, 1.727),
                new Translation2d(6.129, 1.443),
                new Translation2d(6.595, 0.741),
                new Translation2d(7.124, 0.457),
                new Translation2d(7.759, 1.092),
                new Translation2d(7.124, 1.727),
                new Translation2d(6.595, 1.443),
                new Translation2d(6.129, 0.741),
                new Translation2d(5.600, 0.457),
                new Translation2d(2.552, 0.457),
                new Translation2d(2.023, 0.741),
                new Translation2d(1.557, 1.443),
                new Translation2d(1.028, 1.727)
            ),
            new Pose2d(0.200, 1.727, new Rotation2d(0)),
            TrajConfig.configFwd
        );
        public static final Trajectory slalomPath = TrajectoryGenerator.generateTrajectory( //v=4 a=5 (less than 8 seconds, almost world record 3/27/21, just worked)
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of( 
                new Translation2d(1.448, 0.575),
                new Translation2d(2.211 -0.200, 1.727),
                new Translation2d(5.941 -0.175 -0.650 +0.15, 1.727),
                //new Translation2d(6.783, 0.457), //6.595, 0.741 (old val tangent)
                new Translation2d(7.124 -0.650 -0.30, 0.056 -0.150), //7.124, 0.457
                new Translation2d(8.273, 1.092), //7.759, 1.092
                new Translation2d(7.124 -0.05, 2.241), //7.124, 1.727
                //new Translation2d(6.783, 1.727), //6.595, 1.443 (old val tangent)
                new Translation2d(5.941 +0.900 +0.05, 0.457 -.2 -0.150), //slight adjustment to avoid hitting a ball
                new Translation2d(2.211 -0.300 +1.2, 0.457),
                new Translation2d(1.369 + 0.4, 1.727) //point right before finishing line in theory
             ),
            new Pose2d(0.618, 2.859, new Rotation2d(0)),
            TrajConfig.configFwd
        );
    }
    public static final class BounceRunTrajectories { //v=4 a=5
        public static final Trajectory stageOne = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(  ),
            new Pose2d(1.790 -.5, 1.327 + 1.050 - 0.2 - 0.2, new Rotation2d(90)),
            TrajConfig.configFwd
        );
        public static final Trajectory stageTwo = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.790 -.5, 1.327 + 1.050 - 0.2 - 0.2, new Rotation2d(90)), 
            List.of( 
                new Translation2d(2.739 -0.3, -0.700),
                new Translation2d(3.430, -1.056 -0.4),
                new Translation2d(3.948, -0.477)
            ),
            new Pose2d(4.076 +.3 +0.2 +0.1, 1.327 + 1.050 - 0.2, new Rotation2d(-90)),
            TrajConfig.configRev
        );
        public static final Trajectory stageThree = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4.076 +.3 +0.2 +0.1, 1.327 + 1.050 - 0.2, new Rotation2d(-90)), 
            List.of( 
                new Translation2d(4.205 +1.25, -0.477),
                //new Translation2d(4.838, -1.066), 
                new Translation2d(5.219, -1.056),
                //new Translation2d(5.600, -1.066),
                new Translation2d(6.234 +.25, -0.477)
            ),
            new Pose2d(6.362 +0.2, 1.327 + 1.050 - 0.2, new Rotation2d(90)),
            TrajConfig.configFwd
        );
        public static final Trajectory stageFour = TrajectoryGenerator.generateTrajectory(
            new Pose2d(6.362 +0.2, 1.327 + 1.050 - 0.2, new Rotation2d(90)), 
            List.of(  ),
            new Pose2d(7.883, -1.202, new Rotation2d(-180)), //tecnical finish point 8.153, 0.355
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
        public static final Trajectory stageOne = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-2, -0.5),
                new Translation2d(-2.7, 0.5) 
            ),
            new Pose2d( -2.55, 3.0, new Rotation2d(-45) ), 
            TrajConfig.configRev
        );
        public static final Trajectory stagetwo = TrajectoryGenerator.generateTrajectory(
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
    public static final class stealBallsTrajectories {
        public static final Trajectory stageOne = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(), 
            new Pose2d(-3, 0, new Rotation2d(0)),
            TrajConfig.configRev
        );
        public static final Trajectory stageTwo = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-3, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(0, -4, new Rotation2d(0)),
            TrajConfig.configFwd
        );
    }
    
}
