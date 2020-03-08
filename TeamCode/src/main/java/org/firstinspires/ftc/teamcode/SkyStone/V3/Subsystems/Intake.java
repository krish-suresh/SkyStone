//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.JMotor;
import org.firstinspires.ftc.teamcode.RobotLibs.JServo;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class Intake implements Subsystem {
    private Robot robot;
    public JMotor intakeMotorRight;
    public JMotor intakeMotorLeft;
    public JServo intakeServoR;
    public JServo intakeServoL;
    //Motors from robot orientation
    public OpMode opMode;

    public Intake(OpMode mode) {
        opMode = mode;
        intakeMotorLeft = new JMotor(mode.hardwareMap, "LI");
        intakeMotorRight = new JMotor(mode.hardwareMap, "RI");
        intakeServoL = new JServo(mode.hardwareMap, "I.L");
        intakeServoR = new JServo(mode.hardwareMap, "I.R");
        robot = Robot.getInstance();
    }

    @Override
    public void update() {
        // This makes the collector have a default slightly open position and when leftBump is pressed will close
        // TODO Need to test if this logic is better than the reverse
        if (opMode.gamepad1.left_bumper) {
            setCollectorPos(CollectorPoses.MIDDLE);
        } else {
            setCollectorPos(CollectorPoses.CLOSED);
        }

        setIntakePower(opMode.gamepad1.right_trigger - opMode.gamepad1.left_trigger);
    }

    public void setIntakePower(double intakePower) {
        setIntakePower(intakePower, 0);
    }

    public void setIntakePower(double intakePower, double reversePower) {
        intakeMotorRight.setPower(-intakePower + (reversePower * 2));
        intakeMotorLeft.setPower(intakePower);
    }

    public void setCollectorPos(CollectorPoses pos) {
        double leftServoPos;
        double rightServoPos;
        switch (pos) {
            case OPEN:
                leftServoPos = 1;
                rightServoPos = 0;
                break;
            case CLOSED:
                leftServoPos = 0;
                rightServoPos = 1;
                break;
            case MIDDLE:
                leftServoPos = 0.2;
                rightServoPos = 0.8;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + pos);
        }
        intakeServoL.setPosition(leftServoPos);
        intakeServoR.setPosition(rightServoPos);
    }

    public enum CollectorPoses {
        CLOSED, OPEN, MIDDLE
    }
}
