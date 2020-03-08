/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static frc.robot.Constants.inAuto;

//The VR is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot documentation. 
//If you change the name of this class or the package after creating this project, you must also update the build.gradle file in the project.

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() { //This function is run when the robot is first started up and should be used for any initialization code.
    m_robotContainer = new RobotContainer(); //Instantiate our RobotContainer. This will perform all our button bindings, and put our autonomous chooser on the dashboard.
  }
  
  @Override
  public void robotPeriodic() { //This function is called every robot packet, no matter the mode. Use for things wanted in all modes
    CommandScheduler.getInstance().run();//Runs Command Scheduler, must be called for any command to work
  }



  @Override
  public void disabledInit() { //This function is called once each time the robot enters Disabled mode.
    //m_robotContainer.gDriveTrain().brakeMode(); //Here for the auto testing, want robot to stop after auto sends it full speed into a wall
  }

  @Override
  public void disabledPeriodic() {
    if ( m_robotContainer.gDriveTrain().getHighestVelocity() >= 1.7 ) { //if either side is going more then 10 m/s, about 1/5 of top speed high gear
      m_robotContainer.gDriveTrain().brakeMode();
    } else {
      m_robotContainer.gDriveTrain().coastMode();
    }
  }



  @Override
  public void autonomousInit() { //This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
    inAuto = true;

    m_autonomousCommand = m_robotContainer.getAutonomousCommand(); //get auto command from robot container

    m_robotContainer.gDriveTrain().brakeMode(); //Set brake mode
    //m_robotContainer.gDriveTrain().coastMode();
    m_robotContainer.gShifter().shiftUp(); //Ensure in high gear for the Auto, 7.56 gear ratio 
    m_robotContainer.gDriveTrain().resetEncPos(); //Set encoders to zero

    if (m_autonomousCommand != null) { //If auto command exists
      m_autonomousCommand.schedule(); //run the auto command
    }
  }

  @Override
  public void autonomousPeriodic() { //This function is called periodically during autonomous.
  }



  @Override
  public void teleopInit() {
    inAuto = false; //Modify constant
    m_robotContainer.gDriveTrain().brakeMode(); //Set brake
    m_robotContainer.gDriveTrain().resetEncPos(); //Set encoders to zero

    if (m_autonomousCommand != null) { //This makes sure that the autonomous stops running when teleop starts running.
      m_autonomousCommand.cancel(); //If you want the autonomous to  continue until interrupted by another command comment it out.
    }
  }

  @Override
  public void teleopPeriodic() {    //This function is called periodically during operator control.
  }



  @Override
  public void testInit() {     // Cancels all running commands at the start of test mode.
    m_robotContainer.gDriveTrain().brakeMode(); //Set brake
    CommandScheduler.getInstance().cancelAll(); //Clear all commands
  }

  @Override
  public void testPeriodic() { //This function is called periodically during test mode.
  }
}
