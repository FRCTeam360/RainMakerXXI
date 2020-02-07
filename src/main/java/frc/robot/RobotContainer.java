/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrajectoryConstants;

import frc.robot.commands.*;
import frc.robot.commands.autoCommands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {   // The robot's subsystems and commands are defined here...
  private final DriveTrain drivetrain = new DriveTrain();
  private final Pneumatics pneumatics = new Pneumatics();
  private final Shifter shifter = new Shifter();
  private final Shooter shooter = new Shooter();
  private final Limelight limelight = new Limelight();
  private final Intake intake = new Intake();

  private final JoystickTankDrive joystickTankDrive = new JoystickTankDrive(drivetrain);
  private final Pressurize pressurize = new Pressurize(pneumatics);
  private final Shift shift = new Shift(shifter);
  private final ShootBalls shootBalls = new ShootBalls(shooter);

  private final RamseteCommand m_autoCommand_testing = new RamseteCommand(
    TrajectoryConstants.testingTrajectory,
    drivetrain::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(AutoConstants.ksVolts,
    AutoConstants.kvVoltSecondsPerMeter,
    AutoConstants.kaVoltSecondsSquaredPerMeter),
    AutoConstants.kDriveKinematics,
    drivetrain::getWheelSpeeds,
    new PIDController(AutoConstants.kPDriveVel, 0, 0),
    new PIDController(AutoConstants.kPDriveVel, 0, 0),
    drivetrain::tankDriveVolts,
    drivetrain
  );
  
	private final SequentialCommandGroup m_autoCommand_left = new SequentialCommandGroup(
    new AutoLowerIntake(intake),
    new AutoShootBalls(shooter),
    new ParallelRaceGroup( 
      new AutoRunIntake(intake), 
      new RamseteCommand(
        TrajectoryConstants.leftAutoTrajectory, 
        drivetrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        drivetrain::tankDriveVolts,
        drivetrain) 
      ),                                                                                            
    new AutoAlignShoot() //Add params
  );

  private final SequentialCommandGroup m_autoCommand_middle = new SequentialCommandGroup(
    new AutoLowerIntake(intake),
    new AutoShootBalls(shooter),
    new ParallelRaceGroup( 
      new AutoRunIntake(intake), 
      new RamseteCommand(
        TrajectoryConstants.middleAutoTrajectory, 
        drivetrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        drivetrain::tankDriveVolts,
        drivetrain) 
      ),                                                                                            
    new AutoAlignShoot() //Add params
  );

  private final SequentialCommandGroup m_autoCommand_right = new SequentialCommandGroup(
    new AutoLowerIntake(intake),
    new AutoShootBalls(shooter),
    new ParallelRaceGroup( 
      new AutoRunIntake(intake), 
      new RamseteCommand(
        TrajectoryConstants.rightAutoTrajectory, 
        drivetrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        drivetrain::tankDriveVolts,
        drivetrain) 
      ),                                                                                            
    new AutoAlignShoot() //Add params
  );

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(joystickTankDrive);
    pneumatics.setDefaultCommand(pressurize);
    shifter.setDefaultCommand(shift);
    shooter.setDefaultCommand(shootBalls);

    configureButtonBindings();

    m_chooser.addOption("Ramsete Testing", m_autoCommand_testing); //Right now is just the Ramsete Movement Command
    m_chooser.addOption("Left Auto", m_autoCommand_left);
    m_chooser.addOption("Right Auto", m_autoCommand_middle);
    m_chooser.addOption("Right Auto", m_autoCommand_right);

    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  public DriveTrain gDriveTrain () {
    return drivetrain;
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected(); //Sends the autonomous command initialized above
  }
}