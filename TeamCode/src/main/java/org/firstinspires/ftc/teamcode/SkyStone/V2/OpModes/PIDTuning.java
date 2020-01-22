package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

@Autonomous(name = "AA_Tuner")
@Config
public class PIDTuning extends OpMode {
    Robot robot;
    private ElapsedTime cycleTime;
    private double lastTime = 0;
    Pose2d currentPos;
    public static double TARGET_X = 0;
    public static double TARGET_Y = 72;
    public static double TARGET_Heading = 0;
    @Override
    public void init() {
        robot = new Robot(this);//Makes robot obj
        cycleTime = new ElapsedTime();
    }

    @Override
    public void start() {
        robot.mecanumDrive.goToPosition(new Pose2d(TARGET_X,TARGET_Y,TARGET_Heading));

    }


    @Override
    public void loop() {
        robot.mecanumDrive.updatePoseEstimate();
        currentPos = robot.mecanumDrive.getPoseEstimate();
        robot.mecanumDrive.updateGoToPos();

        robot.telemetry.addData("Refresh Rate", 1 / ((cycleTime.nanoseconds() - lastTime)/1000000000));
        lastTime = cycleTime.nanoseconds();
        robot.telemetry.addData("Robot Pos", currentPos);
        robot.telemetry.addData("errorX",robot.mecanumDrive.PID_FORWARD.getLastError());
        robot.telemetry.addData("errorY",robot.mecanumDrive.PID_STRAFE.getLastError());
        robot.telemetry.addData("errorH",robot.mecanumDrive.PID_HEADING.getLastError());
        robot.telemetry.update();
    }

}

