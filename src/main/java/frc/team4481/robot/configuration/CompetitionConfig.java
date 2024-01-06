package frc.team4481.robot.configuration;

import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.robot.HIDlayout.CompetitionLayout;

public class CompetitionConfig implements Configuration {
    private HIDLayout compLayout = null;

    @Override
    public double getMaxDriveVelocity() {
        return 4.5;
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
        if (compLayout == null){
            compLayout = new CompetitionLayout(driver, operator);
        }
        return compLayout;
    }
}

