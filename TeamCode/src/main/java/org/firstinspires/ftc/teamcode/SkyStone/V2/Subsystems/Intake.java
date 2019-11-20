package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class Intake implements Subsystem {
    public DcMotorEx intakeMotorRight;
    public DcMotorEx intakeMotorLeft;
    public Servo intakeServoR;
    public Servo intakeServoL;
    //Motors from robot orientation
    public OpMode opMode;
    public StickyGamepad stickyGamepad1;

    public Intake(OpMode mode) {
        opMode = mode;
        intakeMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "LI");
        intakeMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "RI");
        intakeServoL = opMode.hardwareMap.get(Servo.class, "I.L");
        intakeServoR = opMode.hardwareMap.get(Servo.class, "I.R");
//        setCollectorPos(CollectorPoses.FOLDED_IN);
        stickyGamepad1 = new StickyGamepad(opMode.gamepad1);
    }

    @Override
    public void update() {
        setCollectorPos(opMode.gamepad1 .left_bumper ? CollectorPoses.MIDDLE : (opMode.gamepad1.x ? CollectorPoses.FOLDED_IN : CollectorPoses.RELEASED));
        setIntakePower(opMode.gamepad1.right_trigger - opMode.gamepad1.left_trigger);
        stickyGamepad1.update();
        opMode.telemetry.addData("leftEnc",intakeMotorRight.getCurrentPosition());
        opMode.telemetry.addData("horzEnc",intakeMotorLeft.getCurrentPosition());
    }
    public void setIntakePower(double intakePower) {
        intakeMotorRight.setPower(-intakePower);
        intakeMotorLeft.setPower(intakePower);
    }

    public void setCollectorPos(CollectorPoses pos) {
        double leftServoPos;
        double rightServoPos;
        switch (pos) {
            case RELEASED:
                leftServoPos = 1;
                rightServoPos = 0;
                break;
            case FOLDED_IN:
                leftServoPos = 0;
                rightServoPos = 1;
                break;
            case MIDDLE:
                leftServoPos = 0.5;
                rightServoPos = 0.8;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + pos);
        }
        intakeServoL.setPosition(leftServoPos);
        intakeServoR.setPosition(rightServoPos);
    }

    public enum CollectorPoses {
        RELEASED, FOLDED_IN, MIDDLE
    }
}
