package frc.team4481.robot.configuration;

import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.robot.HIDlayout.EventLayout;

public class EventConfig implements Configuration {
    private HIDLayout eventLayout = null;

    @Override
    public double getMaxDriveVelocity() {
        return 2;
    }

    @Override
    public double getMaxDirectionRateLimit() {
        return 0;
    }

    @Override
    public boolean isAllowedToDo180() {
        return false;
    }

    @Override
    public HIDLayout getHIDLayout(ControlDevice driver, ControlDevice operator) {
        if (eventLayout == null){
            eventLayout = new EventLayout(driver, operator);
        }
        return eventLayout;
    }
}
