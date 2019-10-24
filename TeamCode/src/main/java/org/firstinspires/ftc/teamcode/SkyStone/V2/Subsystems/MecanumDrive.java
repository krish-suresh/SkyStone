package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;


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
    List<DcMotorEx> driveMotors;
    //TODO add/implement ODO modules
    //TODO add velocity PIDs for all drive
    //TODO add easy integration between RR control and driver
    public Gamepad gamepad1;
    private Gyro gyro;
    public boolean thirdPersonDrive = false;
    //Road Runner
    DriveConstraints constraints = new DriveConstraints(20, 40, 80, 1, 2, 4);
    private Pose2d robotPos;
    PIDCoefficients translationalPid = new PIDCoefficients(5, 0, 0);
    PIDCoefficients headingPid = new PIDCoefficients(2, 0, 0);
    public HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(translationalPid, translationalPid, headingPid);

    public MecanumDrive(OpMode mode) {
        super(8, 8, 8, 15, 15);//TODO These are random vals rn
        opMode = mode;
        leftFront = opMode.hardwareMap.get(DcMotorEx.class, "LF");
        leftBack = opMode.hardwareMap.get(DcMotorEx.class, "LB");
        rightBack = opMode.hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = opMode.hardwareMap.get(DcMotorEx.class, "RF");
        driveMotors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);
        gyro = new Gyro();
        this.gamepad1 = opMode.gamepad1;
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(10,0,0,0));
        }
        //TODO make alliances for start pos
        robotPos = new Pose2d(-36,-63,90);// Red start pos
    }

    @Override
    public void update() {
        if (gamepad1.right_stick_button) {
            thirdPersonDrive = true;
            gyro.setCal();
        } else if (gamepad1.x) {
            thirdPersonDrive = false;
        }
        if (thirdPersonDrive) {
            updateMecanumFieldCentric(gamepad1, (gamepad1.right_bumper ? 0.35 : 1));
        } else {
            updateMecanum(gamepad1, (gamepad1.right_bumper ? 0.25 : 1));
        }
        opMode.telemetry.addData("DRIVETRAIN Gyro", gyro.getHeading());
    }

    //TODO fix this function it is really bad lol
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
        motorPowers[0] = speed * sin(angle) + rotation;
        motorPowers[1] = speed * -cos(angle) + rotation;
        motorPowers[2] = speed * -sin(angle) + rotation;
        motorPowers[3] = speed * cos(angle) + rotation;

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
        double rotation = -gamepad.right_stick_x * scaling;

        speed = scalePower(speed);
        rotation = Math.pow(rotation,3);
        setMecanum(angle, speed, rotation);
    }

    public void updateMecanumFieldCentric(Gamepad gamepad, double scaling) {
        double angle = Math.atan2(gamepad.left_stick_x, gamepad.left_stick_y) + Math.toRadians(gyro.getHeading());
        double speed = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y) * scaling;
        double rotation = -gamepad.right_stick_x * scaling;
        speed = scalePower(speed);
        setMecanum(angle, speed, rotation);
    }


    private static double scalePower(double speed) {
        return .5 * Math.pow(2 * (speed - .5), 3) + .5;
    }

    //Road Runner
    @Override
    public List<Double> getWheelPositions() {
        return null;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {

    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }

    public Trajectory startToSkyStone(int skyStonePos){

        int[] skyStoneX = {-28,-36,-44};
        return new TrajectoryBuilder(robotPos,constraints)
                .splineTo(new Pose2d(-36,-48),new ConstantInterpolator(robotPos.getHeading()))
                .splineTo(new Pose2d(skyStoneX[skyStonePos],-33),new ConstantInterpolator(robotPos.getHeading()))
                .build();

    }

    public Trajectory stonesToPlatform1(){
        return new TrajectoryBuilder(robotPos,constraints)
                .splineTo(new Pose2d(0,-48),new ConstantInterpolator(robotPos.getHeading()))
                .splineTo(new Pose2d(40,33),new ConstantInterpolator(robotPos.getHeading()))
                .build();
    }
    public Trajectory platformToStones(){
        return new TrajectoryBuilder(robotPos,constraints)
                .splineTo(new Pose2d(0,-48),new ConstantInterpolator(robotPos.getHeading()))
                .splineTo(new Pose2d(40,33),new ConstantInterpolator(robotPos.getHeading()))
                .build();
    }
    public Pose2d getRobotPos(){
        return robotPos;
    }
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

        public Orientation getOrientation() {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return angles;
        }

    }
//class TrackingWheels extends Tracking {
//    public final double TICKS_PER_REV = 1;
//    public final double WHEEL_RADIUS = 2; // in
//    public final double GEAR_RATIO = 1; // output/input
//
//    public final double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
//    public final double FORWARD_OFFSET = 4; // in; offset of the lateral wheel
//
//    private DcMotor leftEncoder, rightEncoder, frontEncoder;
//
//    public TrackingWheels(HardwareMap hardwareMap) {
//        super(Arrays.asList(
//                new Vector2d(0, LATERAL_DISTANCE / 2), // left
//                new Vector2d(0, -LATERAL_DISTANCE / 2), // right
//                new Vector2d(FORWARD_OFFSET, 0) // front
//        ), Arrays.asList(0.0, 0.0, Math.PI / 2));
//
//        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
//        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
//        frontEncoder = hardwareMap.dcMotor.get("frontEncoder");
//    }
//
//    public double encoderTicksToInches(int ticks) {
//        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
//    }
//
//    @NotNull
//    @Override
//    public List<Double> getWheelPositions() {
//        return Arrays.asList(
//                encoderTicksToInches(leftEncoder.getCurrentPosition()),
//                encoderTicksToInches(rightEncoder.getCurrentPosition()),
//                encoderTicksToInches(frontEncoder.getCurrentPosition())
//        );
//    }
//}
}
