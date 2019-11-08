package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class Intake implements Subsystem {
    public DcMotorEx intakeMotorRight;
    public DcMotorEx intakeMotorLeft;
    public Servo intakeServoR;
    public Servo intakeServoL;
    //Motors from robot orientation
    public OpMode opMode;

    public Intake(OpMode mode) {
        opMode = mode;
        intakeMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "LI");
        intakeMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "RI");
        intakeServoL = opMode.hardwareMap.get(Servo.class, "I.L");
        intakeServoR = opMode.hardwareMap.get(Servo.class, "I.R");
//        setCollectorPos(CollectorPoses.FOLDED_IN);
    }

    @Override
    public void update() {
        setCollectorPos(opMode.gamepad1.left_bumper ? CollectorPoses.MIDDLE : (opMode.gamepad1.x ? CollectorPoses.FOLDED_IN : CollectorPoses.RELEASED));
        setIntakePower(opMode.gamepad1.right_trigger - opMode.gamepad1.left_trigger);

    }
    public void setIntakePower(double intakePower) {
        intakeMotorRight.setPower(-intakePower);
        intakeMotorLeft.setPower(intakePower);
    }

    public void setCollectorPos(CollectorPoses pos) {
        double leftServoPos = 0;
        double rightServoPos = 1;
        switch (pos) {
            case RELEASED:
                leftServoPos = 0.6;
                rightServoPos = 0.7;
                break;
            case FOLDED_IN:
                leftServoPos = 1;
                rightServoPos = .2;
                break;
            case MIDDLE:
                leftServoPos = 0.85;
                rightServoPos = 0.4;
                break;
        }
        intakeServoL.setPosition(leftServoPos);
        intakeServoR.setPosition(rightServoPos);
    }

    public enum CollectorPoses {
        RELEASED, FOLDED_IN, MIDDLE
    }
}
