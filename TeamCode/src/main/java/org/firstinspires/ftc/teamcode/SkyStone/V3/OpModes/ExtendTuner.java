package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

@TeleOp(name="Extend Tuner")
public class ExtendTuner extends OpMode {

    Robot robot;
    double extendPos = 0;

    @Override
    public void init() {
        robot = new Robot(this);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            extendPos = Range.clip(extendPos + 0.001, 0, 1);
        } else if (gamepad1.dpad_down) {
            extendPos = Range.clip(extendPos - 0.001, 0, 1);
        }

        robot.depositLift.extendL.setPosition(extendPos);
        robot.depositLift.extendR.setPosition(extendPos);
        robot.telemetry.addData("Pos", extendPos);
        robot.telemetry.update();
    }
}
