package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DriveConstants.BASE_CONSTRAINTS_SLOW;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DriveConstants.kStatic;

public class MecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive implements Subsystem {

    /*
     *    front
     * 0         3
     *
     *
     * 1         2
     * */


    public OpMode opMode;
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DcMotorEx rightFront;
    Servo grabServoRight;
    Servo grabServoLeft;
    List<DcMotorEx> driveMotors;
    public Gamepad gamepad1;
    StickyGamepad stickyGamepad1;
    private Gyro gyro;
    public boolean thirdPersonDrive = false;
    //Road Runner
    DriveConstraints constraints = BASE_CONSTRAINTS;
    DriveConstraints constraintsSlow = BASE_CONSTRAINTS_SLOW;
    PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(1, 0, 0);
    PIDCoefficients HEADING_PID = new PIDCoefficients(0.3, 0, 0);
    public HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);

    public MecanumDrive(OpMode mode) {
        super(kV, kA, kStatic, TRACK_WIDTH);
        opMode = mode;
        leftFront = opMode.hardwareMap.get(DcMotorEx.class, "LF");
        leftBack = opMode.hardwareMap.get(DcMotorEx.class, "LB");
        rightBack = opMode.hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = opMode.hardwareMap.get(DcMotorEx.class, "RF");
        driveMotors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);
        gyro = new Gyro();
        this.gamepad1 = opMode.gamepad1;
        for (DcMotorEx motor : driveMotors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }
        //TODO make alliances for start pos
        grabServoRight = opMode.hardwareMap.get(Servo.class, "P.G.R");
        grabServoLeft = opMode.hardwareMap.get(Servo.class, "P.G.L");
        setLocalizer(new Odometry(opMode.hardwareMap));
        setPoseEstimate(new Pose2d(-36, -63, Math.PI/2));// Red start pos
        stickyGamepad1 = new StickyGamepad(opMode.gamepad1);
    }

    @Override
    public void update() {
        if (gamepad1.x) {
            thirdPersonDrive = true;
            gyro.setCal();
        } else if (gamepad1.right_stick_button) {
            thirdPersonDrive = true;
        } else if (gamepad1.left_stick_button) {
            thirdPersonDrive = false;
        }

        if (thirdPersonDrive) {
            updateMecanumFieldCentric(gamepad1, (gamepad1.right_bumper ? 0.25 : 1));
        } else {
            updateMecanum(gamepad1, (gamepad1.right_bumper ? 0.25 : 1));
        }
        opMode.telemetry.addData("DRIVETRAIN Gyro", gyro.getHeading());
        if (stickyGamepad1.y) {
            platformGrab();
        } else {
            platformRelease();
        }
        updatePoseEstimate();
        stickyGamepad1.update();
        opMode.telemetry.addData("POSE", getPoseEstimate());
    }

    public void platformRelease() {
        grabServoLeft.setPosition(0);
        grabServoRight.setPosition(0.9);
    }

    public void platformGrab() {
        grabServoLeft.setPosition(0.8);
        grabServoRight.setPosition(0.2);
    }

    @Deprecated
    public void setMecanum() {
        leftFront.setPower(Range.clip((gamepad1.left_stick_y - gamepad1.right_stick_x - (gamepad1.left_stick_x / 4)), -1, 1));
        rightFront.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.right_stick_x - (gamepad1.left_stick_x / 4)), -1, 1));
        leftBack.setPower(Range.clip((gamepad1.left_stick_y - gamepad1.right_stick_x + (gamepad1.left_stick_x / 4)), -1, 1));
        rightBack.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.right_stick_x + (gamepad1.left_stick_x / 4)), -1, 1));
    }

    public void setMecanum(double angle, double speed, double rotation) {
        angle += 3 * Math.PI / 4;
        speed *= Math.sqrt(2);

        double motorPowers[] = new double[4];
        motorPowers[0] = (speed * sin(angle)) + rotation;
        motorPowers[1] = (speed * -cos(angle)) + rotation;
        motorPowers[2] = (speed * -sin(angle)) + rotation;
        motorPowers[3] = (speed * cos(angle)) + rotation;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(motorPowers[0]),
                Math.abs(motorPowers[1]), Math.abs(motorPowers[2]), Math.abs(motorPowers[3])));
        if (max > 1.0) {
            for (int i = 0; i < 4; i++) {
                motorPowers[i] /= max;
            }
        }
        int i = 0;
        for (DcMotor motor : driveMotors) {
            motor.setPower(motorPowers[i]);
            i++;
        }
    }

    public void updateMecanum(Gamepad gamepad, double scaling) {
        double angle = Math.atan2(gamepad.left_stick_x, gamepad.left_stick_y);
        double speed = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y) * scaling;
        double rotation = -gamepad.right_stick_x*.8 * scaling;

        speed = scalePower(speed);
        setMecanum(angle, speed, rotation);
    }

    public void updateMecanumFieldCentric(Gamepad gamepad, double scaling) {
        double angle = Math.atan2(gamepad.left_stick_x, gamepad.left_stick_y) + Math.toRadians(gyro.getHeading());
        double speed = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y) * scaling;
        double rotation = -gamepad.right_stick_x*.8 * scaling;
        speed = scalePower(speed);
        setMecanum(angle, speed, rotation);
    }


    private static double scalePower(double speed) {
        return .5 * Math.pow(2 * (speed - .5), 3) + .5;
    }

    //Road Runner
    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : driveMotors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : driveMotors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }
    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(-v);
        leftBack.setPower(-v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : driveMotors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }


    public DriveConstraints getConstraints() {
        return constraints;
    }

    public DriveConstraints getConstraintsSlow() {
        return constraintsSlow;
    }

    public void runUsingEncoder(boolean runwEnc) {
        for (DcMotorEx motor : driveMotors) {
            if (runwEnc) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }


    //**DRIVE INNER CLASSES**//
    class Gyro {

        BNO055IMU gyro;
        Orientation angles;
        double cal = 0;

        //init
        public Gyro() {

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            gyro = opMode.hardwareMap.get(BNO055IMU.class, "gyro");
            gyro.initialize(parameters);
        }

        //get heading of gyro
        public double getHeading() {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = angles.firstAngle;

            return (-angle) + cal;
        }

        public void setCal() {
            if (gamepad1.right_stick_button) {
                cal = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        }


    }
}
