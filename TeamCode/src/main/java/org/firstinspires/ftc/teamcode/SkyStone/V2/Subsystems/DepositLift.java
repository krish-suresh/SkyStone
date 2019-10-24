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
    private final int SPOOL_CIRCUMFERENCE = (int) (Math.PI * 1.25);
    //Bare Motor has 28 ticks per rev and this is 3.7 motor; it is lifting 4 inches for the height of the place and then 2 inches for tol
    private final int LIFT_LEVEL_COUNTS = (int) (3.7 * 28*(4/ SPOOL_CIRCUMFERENCE));
    private final int BOTTOM_POS_COUNT = (int) (3.7 * 28*(2/SPOOL_CIRCUMFERENCE));
    private final double ROTATION_DEFAULT = 0.35;
    private final double ROTATION_ROTATE = 0.9;
    private final double GRAB_CLOSE = 0.22;
    private final double GRAB_OPEN = 0;

    private DcMotorEx liftMotorRight;
    private DcMotorEx liftMotorLeft;
    private Servo grab;
    private Servo rotation;
    private CRServo extension1;
    private CRServo extension2;
    //TODO Mag sensor for bottoming out lift
    private OpMode opMode;
    public int liftHeight = 0;
    public int liftBottomCal = 0;
    private double liftPower = 0;
    private int targetLevel;
    private boolean tempDown=true;
    private boolean tempUp=true;
    private int targetCounts;

    PIDFController pid = new PIDFController(new PIDCoefficients(0.005, 0, 0.001));  //TODO Calibrate PID
    StickyGamepad stickyGamepad2;

    public DepositLift(OpMode mode) {
        opMode = mode;
        stickyGamepad2 = new StickyGamepad(mode.gamepad2);
        liftMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "L.R");
        liftMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "L.L");
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        grab = opMode.hardwareMap.get(Servo.class, "D.G");
        rotation = opMode.hardwareMap.get(Servo.class, "D.R");
        extension1 = opMode.hardwareMap.get(CRServo.class, "D.E1");
        extension2 = opMode.hardwareMap.get(CRServo.class, "D.E2");
        pid.reset();
        pid.setOutputBounds(-1, 1);
        pid.setTargetPosition(0);
    }

    @Override
    public void update() {
        //get lift height
        stickyGamepad2.update();
        liftHeight = getLiftHeight() - liftBottomCal;
        //calibrate the bottom pos
        if (opMode.gamepad2.x) {
            liftBottomCal = getLiftHeight();
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
            liftPower = -pid.update(liftHeight);
            opMode.telemetry.addData("ERROR",pid.getLastError());
            opMode.telemetry.addData("LIFT POWER",liftPower);
        } else {
            liftPower = Range.clip(Math.pow(opMode.gamepad2.right_stick_y, 3), -1, 1);
        }
        //updates the lift power to whatever the above things output
        updateLiftPower(liftPower);

        grab.setPosition(stickyGamepad2.right_bumper ? GRAB_CLOSE : GRAB_OPEN);
        setExtensionPower(opMode.gamepad2.right_trigger-opMode.gamepad2.left_trigger);


        rotation.setPosition(stickyGamepad2.left_bumper ? ROTATION_DEFAULT : ROTATION_ROTATE);
        opMode.telemetry.addData("DEPOSIT Current Height", liftHeight);
        opMode.telemetry.addData("DEPOSIT Target Height", targetLevel);
        opMode.telemetry.addData("DEPOSIT Target Counts", targetCounts);
    }

    private void setExtensionPower(double power) {
        extension1.setPower(-power / 2);
        extension2.setPower(-power / 2);
    }

    public void updateLiftPower(double power) {
        liftMotorLeft.setPower(power);
        liftMotorRight.setPower(power);
    }
    private int getLiftHeight(){
        return -liftMotorRight.getCurrentPosition();
    }

}
