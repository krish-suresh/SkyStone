package org.firstinspires.ftc.teamcode.RobotLibs;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.AutoGrab;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;

public abstract class AutoState {
    // base class for all AutoState classes such as Wait, Grab

    /**
     * Does methods of subclasses
     * @return what state to run next loop
     */
    public abstract AutoState doLoop(ElapsedTime time, Robot robot);

}
