/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.autos.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser {

    private SendableChooser<String> locationChooser;
    private SendableChooser<Command> autoChooser;

    private String selectedLocation;

    /*
    private Command doNothingAuto;
	private Command crossLineMotionProfiled;
	private Command startCenterDropCubeLeftSwitch;
	private Command startCenterDropCubeRightSwitch;
	private Command startCenterDropCubeLeftSwitch2Cube;
	private Command startCenterDropCubeRightSwitch2Cube;
	private Command startLeftDropCubeLeftSwitch;
	private Command startLeftDropCubeRightScale;
	private Command startLeftDropCubeLeftScale;
	private Command startRightDropCubeLeftScale;
	private Command startRightDropCubeRightSwitch;
    private Command startRightDropCubeRightScale;
    */

    public AutoChooser(RobotContainer container) {
        /*
		crossLineMotionProfiled = new CrossLineMotionProfiled();
		startCenterDropCubeLeftSwitch = new StartCenterDropCubeLeftSwitch();
		startCenterDropCubeRightSwitch = new StartCenterDropCubeRightSwitch();
		startLeftDropCubeLeftScale = new StartLeftDropCubeLeftScale();
		startLeftDropCubeLeftSwitch = new StartLeftDropCubeLeftSwitch();
		startLeftDropCubeRightScale = new StartLeftDropCubeRightScale();
		startRightDropCubeLeftScale = new StartRightDropCubeLeftScale();
		startRightDropCubeRightScale = new StartRightDropCubeRightScale();
		startRightDropCubeRightSwitch = new StartRightDropCubeRightSwitch();
		startCenterDropCubeLeftSwitch2Cube = new StartCenterDropCubeLeftSwitch2Cube();
        startCenterDropCubeRightSwitch2Cube = new StartCenterDropCubeRightSwitch2Cube();
        */

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
