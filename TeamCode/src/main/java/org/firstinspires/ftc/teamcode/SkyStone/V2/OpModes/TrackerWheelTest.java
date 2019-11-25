package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

public class TrackerWheelTest extends OpMode {

    Robot robot;
    private DcMotor rightEncoder, frontEncoder;

    @Override
    public void init() {
        robot = new Robot(this);
        rightEncoder = hardwareMap.dcMotor.get("LI");
        frontEncoder = hardwareMap.dcMotor.get("RI");
    }

    @Override
    public void loop() {
        telemetry.addData("Right Wheel Pos: ", rightEncoder.getCurrentPosition());
        telemetry.addData("Front Wheel Pos: ", frontEncoder.getCurrentPosition());
    }
}
