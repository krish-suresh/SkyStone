package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Localizers;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.List;

public class OdometryTwoWheel extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.18110236; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    private Gyro gyro;
    private ExpansionHubEx hub;
    private DcMotor rightVertEncoder, horizontalEncoder;

    public OdometryTwoWheel(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(1.96702, -6.91359, 0), // right
                new Pose2d(-1.32999, -7.375, Math.toRadians(90)) // front
        ));
        gyro = new Gyro(hardwareMap);
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        rightVertEncoder = hardwareMap.dcMotor.get("RI");
        horizontalEncoder = hardwareMap.dcMotor.get("L.L");
    }


    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }
        return Arrays.asList(
                encoderTicksToInches(bulkData.getMotorCurrentPosition(rightVertEncoder)),
                encoderTicksToInches(-bulkData.getMotorCurrentPosition(horizontalEncoder))

        );
    }

    @Override
    public double getHeading() {
        return gyro.getHeading();
    }

    class Gyro {

        BNO055IMU gyro;
        Orientation angles;

        //init
        public Gyro(HardwareMap hardwareMap) {

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            gyro.initialize(parameters);
        }

        //get heading of gyro
        public double getHeading() {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double angle = angles.firstAngle;

            return AngleWrap(angle);
        }
        private double AngleWrap(double angle) {
            while (angle < -Math.PI) {
                angle += 2.0 * Math.PI;
            }
            while (angle > Math.PI) {
                angle -= 2.0 * Math.PI;
            }
            return angle;
        }
    }
}