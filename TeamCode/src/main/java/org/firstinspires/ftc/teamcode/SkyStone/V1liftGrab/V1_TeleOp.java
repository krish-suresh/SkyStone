package org.firstinspires.ftc.teamcode.SkyStone.V1liftGrab;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleV1")
public class V1_TeleOp extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor leftIntake;
    DcMotor rightIntake;

    Servo leftLift;
    Servo rightLift;
    Servo grabServo;

    final static double DRIVE_SPEED = 1/(1+ Math.sqrt(2));             //maximum possible input to 1 motor

    final static double INPUT_WHEEL_COEFF = 0.25;                      //in case input wheels are too fast, we can tune them down here

    final static double GAME_PAD_COEFF = 2.0;                          //add ramping behavior to gamepad inputs
                                                        // higher numbers = more low-end sensitivity

    public void init() {
        leftFront = hardwareMap.dcMotor.get("LF");      //drivetrain motors
        rightFront = hardwareMap.dcMotor.get("RF");
        leftBack = hardwareMap.dcMotor.get("LB");
        rightBack = hardwareMap.dcMotor.get("RB");

        leftIntake = hardwareMap.dcMotor.get("LI");     //intake motors
        rightIntake = hardwareMap.dcMotor.get("RI");

//        leftLift = hardwareMap.servo.get("LL");         //lift and grab servos
//        rightLift = hardwareMap.servo.get("RL");
//        grabServo = hardwareMap.servo.get("GS");

        rightFront.setDirection(DcMotor.Direction.REVERSE);      //reverses wheels on left side of drivetrain
        rightBack.setDirection(DcMotor.Direction.REVERSE);       //so wheels will work as expected

        leftIntake.setDirection(DcMotor.Direction.REVERSE);     //same for intake wheels
    }

    public void loop() {
        //takes controller input and sets motors accordingly, with scaling
        scaledUpdateDriveTrain();

        //set power for input wheels
        updateInput();


        //block lifting code, tbd
        //some way to specify height?
        if(gamepad1.a) {
            //grab lift move and place block, probably using lift servos and grab sercos
        }

    }

    //linear drive input - not currently using but might revert back
//    public void updateDriveTrain() {
//        leftFront.setPower((DRIVE_SPEED)*(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
//        rightFront.setPower((DRIVE_SPEED)*(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
//        leftBack.setPower((DRIVE_SPEED)*(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
//        rightBack.setPower((DRIVE_SPEED)*(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
//    }

    //scaled drive for more low-end control
    public void scaledUpdateDriveTrain() {

        double forward = signReflectedPowerOf(gamepad1.left_stick_y, GAME_PAD_COEFF);           //left stick y gives forward motion
        double turning = signReflectedPowerOf(gamepad1.right_stick_x, GAME_PAD_COEFF);          //right stick x gives rotational motion
        double strafing = signReflectedPowerOf(gamepad1.left_stick_x, GAME_PAD_COEFF);          //left stick x gives strafing (side-to-side) motion

        leftFront.setPower(calculateMotorPower(forward, turning, strafing, 1.0, 1.0));
        rightFront.setPower(calculateMotorPower(forward, turning, strafing, -1.0, -1.0));
        leftBack.setPower(calculateMotorPower(forward, turning, strafing, 1.0, -1.0));
        rightBack.setPower(calculateMotorPower(forward, turning, strafing, -1.0, 1.0));

    }

    private double calculateMotorPower(double forward, double turning, double strafing, double turnSign, double strafeSign) {
        return (DRIVE_SPEED)  * (forward + (turnSign * turning) + (strafeSign * strafing));
    }

    public void updateInput() {
        //input wheels activated with press of right bumper button
        if(gamepad1.right_bumper) {
            leftIntake.setPower(INPUT_WHEEL_COEFF);
            rightIntake.setPower(INPUT_WHEEL_COEFF);
        }
        //reverse intake with left bumper button
        else if(gamepad1.left_bumper) {
            leftIntake.setPower(-INPUT_WHEEL_COEFF);
            rightIntake.setPower(-INPUT_WHEEL_COEFF);
        }
        //if neither button is pressed, set powers to 0
        else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    //get a base (+ or -) raised to a power (not necessarily int), returned with the same sign as the base
    //used for scaledUpdateDriveTrain
    public double signReflectedPowerOf(double base, double exponent) {
        double signOfBase = base / Math.abs(base);              //-1 or 1, based on sign of base (- or +)
        double result = Math.pow(Math.abs(base), exponent);     //positive result
        double signedResult = result * signOfBase;              //result of the same sign as base
        return signedResult;
    }

}
