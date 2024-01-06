package frc.team4481.lib.hid;

import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.throwable.HardwareException;

public abstract class HIDLayout {
    protected ControlDevice operator;
    protected ControlDevice driver;

    public HIDLayout(ControlDevice driver, ControlDevice operator){
        this.operator = operator;
        this.driver = driver;
    }

    public abstract void getSubsystemManagers();

    public abstract void updateOrange() throws HardwareException;
    public abstract void updateBlack() throws HardwareException;
}
