package frc.team4481.robot.configuration;


import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;

/**
 * Interface for the configurations of different subsystems
 */
public interface Configuration {

    //Drivetrain
    double getMaxDriveVelocity();
    double getMaxDirectionRateLimit();
    boolean isAllowedToDo180();

    //HIDLayout
    HIDLayout getHIDLayout(ControlDevice driver, ControlDevice operator);
}
