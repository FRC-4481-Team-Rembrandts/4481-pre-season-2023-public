package frc.team4481.robot.auto.modes;

import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.robot.auto.actions.DrivePathAction;
import frc.team4481.robot.auto.actions.SetInitalPositionAction;
import frc.team4481.robot.auto.selector.AutoMode;

//@Disabled
@AutoMode(displayName = "[test] Linear Path test")
public class PathTest extends AutoModeBase {
    String path_1 = "LinearTest";

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(0.1));
        runAction(new SetInitalPositionAction(path_1));
        runAction(new DrivePathAction(path_1));
    }
}
