/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

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

  private final SeqAutoCmd m_autoCommand = new SeqAutoCmd(  ); //Define auto sequential command - drivetrain, intake, shooter, limelight

  public RobotContainer() {
    drivetrain.setDefaultCommand(joystickTankDrive);
    pneumatics.setDefaultCommand(pressurize);
    shifter.setDefaultCommand(shift);
    shooter.setDefaultCommand(shootBalls);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return m_autoCommand; //Sends the autonomous command initialized above
  }
}