package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;
@TeleOp(name = "testServo")
public class Test extends OpMode {
    Robot robot;
    @Override
    public void init() {
        robot = new Robot(this);
    }

    @Override
    public void loop() {
        robot.depositLift.extendL.setPosition(gamepad1.right_bumper?0.85:0.15);
        robot.depositLift.extendR.setPosition(gamepad1.right_bumper?0.15:0.85);
    }
}
