/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {

  private final Pneumatics pneumatics = new Pneumatics();
  private final Solenoids solenoids = new Solenoids();
  private final SparkMaxes sparkMaxes = new SparkMaxes();
  private final Talons talons = new Talons();
  private final Navx navx = new Navx();

  private final Pressurize pressurize = new Pressurize(pneumatics);
  private final FireSolenoids fireSolenoids = new FireSolenoids(solenoids);
  private final RunMotors runMotors = new RunMotors(talons, sparkMaxes);
  private final NavReadout navReadout = new NavReadout(navx);

  public RobotContainer() {
    pneumatics.setDefaultCommand(pressurize);
    solenoids.setDefaultCommand(fireSolenoids);
    talons.setDefaultCommand(runMotors); //no need to do sparkMaxes.setDefaultCommand(runMotors); cuz thats the way it be
    navx.setDefaultCommand(navReadout);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
