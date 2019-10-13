package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class DepositLift implements Subsystem {
    private final int LIFT_LEVEL_COUNTS = (int) (Math.PI * 4 * 3.7 * 28);
    private final int BOTTOM_POS_COUNT = (int) (2*Math.PI * 3.7 * 28);
    private final double ROTATION_DEFAULT = 0.9;
    private final double ROTATION_ROTATE = 0.35;
    private final double GRAB_CLOSE = 0.2;
    private final double GRAB_OPEN = 0;

    private DcMotorEx liftMotorRight;
    private DcMotorEx liftMotorLeft;
    private Servo grab;
    private Servo rotation;
    private CRServo extension;
    //TODO Mag sensor for bottoming out lift
    private OpMode opMode;
    public int liftHeight = 0;
    public int liftBottomCal = 0;
    private double liftPower = 0;
    private int targetLevel;
    private boolean tempDown=true;
    private boolean tempUp=true;
    private int targetCounts;

    PIDFController pid = new PIDFController(new PIDCoefficients(0.001, 0, 0));
    StickyGamepad stickyGamepad2;

    public DepositLift(OpMode mode) {
        opMode = mode;
        stickyGamepad2 = new StickyGamepad(mode.gamepad1);
        liftMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "L.R");
        liftMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "L.L");
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        grab = opMode.hardwareMap.get(Servo.class, "D.G");
        rotation = opMode.hardwareMap.get(Servo.class, "D.R");
        extension = opMode.hardwareMap.get(CRServo.class, "D.E");
        pid.reset();
        pid.setOutputBounds(-1, 1);
        pid.setTargetPosition(0);
    }

    @Override
    public void update() {
        //get lift height
        liftHeight = liftMotorRight.getCurrentPosition() - liftBottomCal;
        //calibrate the bottom pos
        if (opMode.gamepad2.x) {
            liftBottomCal = liftMotorRight.getCurrentPosition();
        }
        //set target height from d pad
        if (stickyGamepad2.dpad_up==tempUp) {
            tempUp = !tempUp;
            targetLevel++;
        } else if (stickyGamepad2.dpad_down==tempDown) {
            tempDown =!tempDown;
            targetLevel--;
        }
        //makes sure the target level is in the range that we can place
        targetLevel = Range.clip(targetLevel,0,7);
        //finds the target counts from the input level and sets position
        targetCounts = BOTTOM_POS_COUNT + (targetLevel * LIFT_LEVEL_COUNTS);
        pid.setTargetPosition(targetCounts);

        //if a is pressed pid to target height if not gpad input
        if (opMode.gamepad2.a) {
            liftPower = pid.update(liftHeight);
        } else {
            liftPower = Range.clip(Math.pow(opMode.gamepad2.right_stick_y, 3) + 0.1, -1, 1);
        }
        //updates the lift power to whatever the above things output
        updateLiftPower(liftPower);

        grab.setPosition(stickyGamepad2.right_bumper ? GRAB_CLOSE : GRAB_OPEN);

        extension.setPower((opMode.gamepad2.left_stick_x) / 2);

        rotation.setPosition(opMode.gamepad2.left_bumper ? ROTATION_DEFAULT : ROTATION_ROTATE);
        opMode.telemetry.addData("DEPOSIT Current Height", liftHeight);
        opMode.telemetry.addData("DEPOSIT Target Height", targetLevel);
        opMode.telemetry.addData("DEPOSIT Target Counts", targetCounts);
    }

    public void updateLiftPower(double power) {
        liftMotorLeft.setPower(power);
        liftMotorRight.setPower(power);
    }

}
