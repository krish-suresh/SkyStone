package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

public class Odometry extends TwoTrackingWheelLocalizer {
    public double TICKS_PER_REV = 4096;
    public double WHEEL_RADIUS = 1.25; // in
    public double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    private DcMotor rightEncoder;
    private DcMotor horizontalEncoder;
    Gyro gyro;
    HardwareMap hardwareMap;
    public Odometry(HardwareMap hwMap) {//TODO START DIRECTION
        super(Arrays.asList(
                new Pose2d(4, -8, 0), // TODO FIND ACTUAL POS
                new Pose2d(4, 8, Math.toRadians(90)) // front
        ));
        hardwareMap = hwMap;
        gyro = new Gyro();
        rightEncoder = hardwareMap.dcMotor.get("RI");
        horizontalEncoder = hardwareMap.dcMotor.get("LI");
    }

    @Override
    public double getHeading() {
        return Math.toRadians(-gyro.getHeading());
    }


    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2.0 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(

                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(horizontalEncoder.getCurrentPosition())
        );
    }

    //**DRIVE INNER CLASSES**//
    class Gyro {

        BNO055IMU gyro;
        Orientation angles;

        //init
        public Gyro() {

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            gyro = hardwareMap.get(BNO055IMU.class, "gyro");
            gyro.initialize(parameters);
        }

        //get heading of gyro
        public double getHeading() {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = angles.firstAngle;

            return (-angle);
        }

    }
}