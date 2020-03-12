/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser {

    private SendableChooser<String> locationChooser = new SendableChooser<>();
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public void AutoChooser() {
        locationChooser.addOption("Our Trench", "OurTrench");
        locationChooser.addOption("Center To Bullseye", "Center");
        locationChooser.addOption("Midfield", "Midfield");
        locationChooser.addOption("Opposing Trench", "OpposingTrench");
        locationChooser.setDefaultOption("Anywhere", "Anywhere" );

        SmartDashboard.putData("Auto Location", locationChooser);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void periodic() {
        if (locationChooser.getSelected().equals("OurTrench")) {
            
        } else if (locationChooser.getSelected().equals("Center")) {

        } else if (locationChooser.getSelected().equals("Midfield")) {

        } else if (locationChooser.getSelected().equals("OpposingTrench")) {

        } else { // "Anywhere"

        }
    }

    public Command getCommand() {
        return autoChooser.getSelected();
    }

}
