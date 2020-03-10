package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotLibs.JServo;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;

import java.util.ArrayList;

@TeleOp(name="Servo Tuner")
public class ServoTuner extends OpMode {

    Robot robot;
    double servoPos = 0.5;

    boolean tempLeft = true;
    boolean tempRight = true;

    ArrayList<JServo> servos;

    @Override
    public void init() {
        robot = new Robot(this);
//        servos.add(robot.autoGrab.rotate);
//        servos.add(robot.autoGrab.turn);
//        servos.add(robot.autoGrab.grab);
//        servos.add(robot.mecanumDrive.grabServoLeft);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            servoPos = Range.clip(servoPos + 0.001, 0, 1);
        } else if (gamepad1.dpad_down) {
            servoPos = Range.clip(servoPos - 0.001, 0, 1);
        }

        if (robot.stickyGamepad1.dpad_left == tempLeft) {
            tempLeft = !tempLeft;
        } else if (robot.stickyGamepad1.dpad_right == tempRight) {
            tempRight = !tempRight;
        }

        robot.autoGrab.rotate.setPosition(servoPos);
        robot.telemetry.addData("Pos", servoPos);
        robot.telemetry.update();
    }
}
