package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Localizers;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;
//@Config
public class OdometryThreeWheel extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.94488; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    private ExpansionHubEx hub;
    private DcMotor rightVertEncoder, horizontalEncoder, leftVertEncoder;
    public static double HOZ_X = -1.553;//-1.03673;
    public static double HOZ_Y = -7.1;//-7.3935;
    public static double LATDIST1 = 7.12;//14.9538;
    public static double LATDIST2 = 7.677;
    public static double changeAmt = -0;
    public static double VERTICAL_X = 2;
    public OdometryThreeWheel(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(VERTICAL_X, LATDIST1+changeAmt, 0), // left 6.93
                new Pose2d(VERTICAL_X, -LATDIST2-changeAmt, 0), // right 6.93
                new Pose2d(HOZ_X, HOZ_Y, Math.toRadians(90)) // front1.942 7.33
        ));
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        leftVertEncoder = hardwareMap.dcMotor.get("LI");
        rightVertEncoder = hardwareMap.dcMotor.get("RI");
        horizontalEncoder = hardwareMap.dcMotor.get("L.L");
    }

    public static double encoderTicksToInches(int ticks,double rad) {
        return rad * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }
        return Arrays.asList(
                encoderTicksToInches(-bulkData.getMotorCurrentPosition(leftVertEncoder),WHEEL_RADIUS),
                encoderTicksToInches(-bulkData.getMotorCurrentPosition(rightVertEncoder),WHEEL_RADIUS),
                encoderTicksToInches(-bulkData.getMotorCurrentPosition(horizontalEncoder),WHEEL_RADIUS)
        );
//        return Arrays.asList(
//                encoderTicksToInches(-leftVertEncoder.getCurrentPosition(),1.25),
//                encoderTicksToInches(rightVertEncoder.getCurrentPosition(),1.27),
//                encoderTicksToInches(-horizontalEncoder.getCurrentPosition(),1.27)
//        );
    }
}