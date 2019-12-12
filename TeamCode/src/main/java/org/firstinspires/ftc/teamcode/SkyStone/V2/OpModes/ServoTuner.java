package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

public class ServoTuner extends OpMode {

    Robot robot;
    private StickyGamepad stickygamepad1 = new StickyGamepad(gamepad1);
    private boolean tempUp = true;
    private boolean tempDown = true;
    private double servoPosAdd=0;


    @Override
    public void init() {
        robot = new Robot(this);

        stickygamepad1 = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        robot.depositLift.setExtendPos(0.5,servoPosAdd);
        if (stickygamepad1.dpad_up == tempUp) {
            tempUp = !tempUp;
            servoPosAdd+=0.001;
        } else if (stickygamepad1.dpad_down == tempDown) {
            tempDown = !tempDown;
            servoPosAdd-=0.001;
        }
        stickygamepad1.update();
        telemetry.addData("servo",servoPosAdd);
        telemetry.update();
    }
}
