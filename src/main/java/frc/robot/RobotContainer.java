/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.commands.autoCommands.AutoRunIntake;
import frc.robot.commands.autoCommands.AutoShootBalls;
import frc.robot.commands.autoCommands.LowerIntake;
import frc.robot.commands.autoCommands.AutoAlignShoot;
//import frc.robot.commands.autoCommands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  RamseteCommand m_autoCommand_testing = new RamseteCommand(
    exampleTrajectory,
    drivetrain::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(AutoConstants.ksVolts,
    AutoConstants.kvVoltSecondsPerMeter,
    AutoConstants.kaVoltSecondsSquaredPerMeter),
    AutoConstants.kDriveKinematics,
    drivetrain::getWheelSpeeds,
    new PIDController(AutoConstants.kPDriveVel, 0, 0),
    new PIDController(AutoConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    drivetrain::tankDriveVolts,
    drivetrain
);
	private final SequentialCommandGroup m_autoCommand_left = new SequentialCommandGroup(
  new LowerIntake(intake),
  new AutoShootBalls(shooter),
  new ParallelRaceGroup(  new AutoRunIntake(intake)), //Add Ramsete as well
  new AutoAlignShoot() //Add params
  );
  private final SequentialCommandGroup m_autoCommand_middle = new SequentialCommandGroup(
    new LowerIntake(intake),
    new AutoShootBalls(shooter),
    new ParallelRaceGroup(  new AutoRunIntake(intake)), //Add Ramsete as well
    new AutoAlignShoot() //Add params
    );
  private final SequentialCommandGroup m_autoCommand_right = new SequentialCommandGroup(
    new LowerIntake(intake),
    new AutoShootBalls(shooter),
    new ParallelRaceGroup(  new AutoRunIntake(intake)), //Add Ramsete as well
    new AutoAlignShoot() //Add params
    );

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(joystickTankDrive);
    pneumatics.setDefaultCommand(pressurize);
    shifter.setDefaultCommand(shift);
    shooter.setDefaultCommand(shootBalls);

    configureButtonBindings();

    m_chooser.addOption("Left Auto", m_autoCommand_left);
    m_chooser.addOption("Right Auto", m_autoCommand_middle);
    m_chooser.addOption("Right Auto", m_autoCommand_right);

    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected(); //Sends the autonomous command initialized above
  }
}