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
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //private final Pneumatics pneumatics = new Pneumatics();
  //private final Shifter shifter = new Shifter();
  private final SparkMaxes sparkMaxes = new SparkMaxes();
  private final Talons talons = new Talons();

  //private final Pressurize pressurize = new Pressurize(pneumatics);
  //private final Shift shift = new Shift(shifter);
  private final RunMotors runMotors = new RunMotors(talons, sparkMaxes);

  public RobotContainer() {

    //pneumatics.setDefaultCommand(pressurize);
    //shifter.setDefaultCommand(shift);
    talons.setDefaultCommand(runMotors); //no need to do: sparkMaxes.setDefaultCommand(runMotors);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
