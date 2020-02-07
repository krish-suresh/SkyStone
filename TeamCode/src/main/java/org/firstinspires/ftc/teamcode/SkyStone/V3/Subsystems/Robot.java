//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

import java.util.Arrays;
import java.util.List;

public class Robot {
    public static double ROBOT_WIDTH = 8;
    //This is the robot class where we can create objects for all the subsystems in the robot
    public OpMode opMode;
    //this opmode is the opmode that each tele or auto program extends, it will be passed through the constructor so that we can use gamepads,telemetry, hardwaremap etc.

    //Subsystems, all of them implement the Subsystem interface to insure they have an update method for the robot.update()
    public MecanumDriveBase mecanumDrive;
    public DepositLift depositLift;
    public Intake intake;
    public TelemetryDisplay telemetryDisplay;
    public RefreshRate refreshRate;
    public boolean opModeIsActive = true;
    List<Subsystem> subsystems;
    public static Robot robot;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public MultipleTelemetry telemetry;
    public StickyGamepad stickyGamepad1;
    public StickyGamepad stickyGamepad2;

    /**
     * @param mode the opmode from the class who uses the robot to allow this class to have access to gamepads,telemetry, hardwaremap etc.
     */
    public Robot(OpMode mode) {
        opMode = mode;
        robot = this;//TODO TEST IF THIS WORKS

        mecanumDrive = new MecanumDriveBase(opMode);
        intake = new Intake(opMode);
        depositLift = new DepositLift(opMode);
//        camera= new Camera(opMode);
//        telemetryDisplay = new TelemetryDisplay(opMode);
        refreshRate = new RefreshRate();
        subsystems = Arrays.asList(mecanumDrive, intake, depositLift, refreshRate);//list of subsystems so that we can update all at once

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
        stickyGamepad1 = new StickyGamepad(opMode.gamepad1);
        stickyGamepad2 = new StickyGamepad(opMode.gamepad2);
    }

    public static Robot getInstance() {
        return robot;//TODO TEST IF THIS WORKS
    }

    /**
     * this function updates all the subsystems when called
     */
    public void update() {
        for (Subsystem subsystem : subsystems) {
            try {
                subsystem.update();
            } catch (Exception e) {
                opMode.telemetry.clearAll();
                opMode.telemetry.addLine(e.getMessage());

            }
        }
        stickyGamepad1.update();
        stickyGamepad2.update();
        telemetry.update();
    }
}
