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

        a_S3F = new S3F(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        a_S3R = new S3R(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        c_S3Gr2Sc = new S3Gr2Sc(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        c_S3Gr3 = new S3Gr3(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        c_S3Gr5Sm = new cS3Gr5Sm(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        c_S3Gt5Sc = new cS3Gt5Sc(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        m_S3Cof = new S3Cof(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        m_S3Gr3Sm = new S3Gr3Sm(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        m_S3Gr5Sm = new mS3Gr5Sm(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        s_Gr2Sm = new Gr2Sm(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        s_St2Sm = new St2Sm(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        s_St3Sm = new St3Sm(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        s_St3SmGr3Sm = new St3SmGr3Sm(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        t_S3Gt3Sc = new S3Gt3Sc(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        t_S3Gt3St = new S3Gt3St(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        t_S3Gt5Sc = new tS3Gt5Sc(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        t_S3Gt5St = new S3Gt5St(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);
        t_S3Gt5StCf = new S3Gt5StCf(container.drivetrain, container.limelight, container.feeder, container.shooter, container.intake);

        //autoChooser starts without options, it gets initialized in periodic anyways

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

                autoChooser.addOption("Shoot 3 & Get Trench 3 & Shoot Center to Bullseye", t_S3Gt3Sc);
                autoChooser.addOption("Shoot 3 & Get Trench 3 & Shoot from Trench", t_S3Gt3St);
                autoChooser.addOption("Shoot 3 & Get Trench 5 & Shoot Center to Bullseye", t_S3Gt5Sc);
                autoChooser.addOption("Shoot 3 & Get Trench 5 & Shoot from Trench", t_S3Gt5St);
                autoChooser.addOption("Shoot 3 & Get Trench 5 & Shoot from Deep Trench & Come to our Feeder ", t_S3Gt5StCf);

            } else if (selectedLocation.equals("Center")) {

                autoChooser.addOption("Shoot 3 & Get Rendezvous 2 & Shoot Center to Bullseye", c_S3Gr2Sc);
                autoChooser.addOption("Shoot 3 & Steal Rendezvous 3", c_S3Gr3);
                autoChooser.addOption("Shoot 3 &  Get Rendezvous 5 & Shoot Midfield", c_S3Gr5Sm);
                autoChooser.addOption("Shoot 3 & Get Trench 5 & Shoot Center to Bullseye", c_S3Gt5Sc);

            } else if (selectedLocation.equals("Midfield")) {

                autoChooser.addOption("", object);
                autoChooser.addOption("", object);
                autoChooser.addOption("", object);

            } else if (selectedLocation.equals("OpposingTrench")) {

                autoChooser.addOption("", object);
                autoChooser.addOption("", object);
                autoChooser.addOption("", object);
                autoChooser.addOption("", object);

            } else { //"Anywhere" or anything else if there's an error

                autoChooser.addOption("", object);
                autoChooser.addOption("", object);

            }

            SmartDashboard.putData("Auto Choice", autoChooser); //Update the Auto Choice with the new options and new chooser

        }
        //Else do nothing 
    }

    public Command getCommand() {
        return autoChooser.getSelected();
    }

}
