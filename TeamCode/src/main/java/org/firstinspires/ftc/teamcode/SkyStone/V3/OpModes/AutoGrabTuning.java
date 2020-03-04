package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;

@TeleOp (name = "AutoGrab Tuning")
public class AutoGrabTuning extends OpMode {

    Robot robot;

    public void init() {

        robot = new Robot(this);

    }

    public void loop() {

        robot.autoGrab.update();
        robot.stickyGamepad1.update();
        robot.stickyGamepad2.update();

    }

}
