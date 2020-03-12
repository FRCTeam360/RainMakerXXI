/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.autos.AnywhereGroup.*;
import frc.robot.commands.autos.CenterGroup.*;
import frc.robot.commands.autos.MidfieldGroup.*;
import frc.robot.commands.autos.OpposingTrenchGroup.*;
import frc.robot.commands.autos.OurTrenchGroup.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser {

    private SendableChooser<String> locationChooser;
    private SendableChooser<Command> autoChooser;

    private String selectedLocation;

    private final Command a_S3F; //AnywhereGroup
    private final Command a_S3R;
    private final Command c_S3Gr2Sc; //CenterGroup
    private final Command c_S3Gr3;
    private final Command c_S3Gr5Sm;
    private final Command c_S3Gt5Sc;
    private final Command m_S3Cof; //MidfieldGroup
    private final Command m_S3Gr3Sm;
    private final Command m_S3Gr5Sm;
    private final Command s_Gr2Sm; //OpposingTrenchGroup
    private final Command s_St2Sm;
    private final Command s_St3Sm;
    private final Command s_St3SmGr3Sm;
    private final Command t_S3Gt3Sc; //OurTrenchGroup
    private final Command t_S3Gt3St;
    private final Command t_S3Gt5Sc;
    private final Command t_S3Gt5St;
    private final Command t_S3Gt5StCf;

    public AutoChooser(RobotContainer container) {



        selectedLocation = "None";

        locationChooser = new SendableChooser<>();
        autoChooser = new SendableChooser<>();

        locationChooser.addOption("OurTrench", "OurTrench");
        locationChooser.addOption("Center", "Center");
        locationChooser.addOption("Midfield", "Midfield");
        locationChooser.addOption("OpposingTrench", "OpposingTrench");
        locationChooser.setDefaultOption("Anywhere", "Anywhere");

        SmartDashboard.putData("Start Location", locationChooser);
        SmartDashboard.putData("Auto Choice", autoChooser);
    }

    public void periodic() {
        if ( !selectedLocation.equals( locationChooser.getSelected() ) ) { //If it changes or is being initialized

            selectedLocation = locationChooser.getSelected(); //Reset the SelectedLocation to what it actually is 
            autoChooser = new SendableChooser<>(); //Clear the auto chooser

            if (selectedLocation.equals("OurTrench")) {
                //autoChooser.addOption(name, object);
                //autoChooser.setDefaultOption(name, object);
            } else if (selectedLocation.equals("Center")) {
                //autoChooser.addOption(name, object);
                //autoChooser.setDefaultOption(name, object);
            } else if (selectedLocation.equals("Midfield")) {
                //autoChooser.addOption(name, object);
                //autoChooser.setDefaultOption(name, object);
            } else if (selectedLocation.equals("OpposingTrench")) {
                //autoChooser.addOption(name, object);
                //autoChooser.setDefaultOption(name, object);
            } else { //"Anywhere"
                //autoChooser.addOption(name, object);
                //autoChooser.setDefaultOption(name, object);
            }
        }
        //Else do nothing 
    }

    public Command getCommand() {
        return autoChooser.getSelected();
    }

}
