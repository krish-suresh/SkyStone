package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoTransitioner.AutoTransitioner;
import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

import java.util.Arrays;

public class AutoFoundation extends OpMode {
    Robot robot;
    Auto.AllianceColors allianceColor = Auto.AllianceColors.RED;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private StickyGamepad stickygamepad1;
    private ElapsedTime elapsedTime;

    private boolean tempUp = true;
    private boolean tempDown = true;
    private double waitTime = 0;
    private  AutoStates state = AutoStates.WAIT;
    @Override
    public void init() {
        robot = new Robot(this);//Makes robot obj
        stickygamepad1 = new StickyGamepad(gamepad1);//for alliance sel
        elapsedTime = new ElapsedTime();
        robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
        AutoTransitioner.transitionOnStop(this, "Tele");//transition from auto to tele when auto ends
    }

    @Override
    public void init_loop() {
        allianceColor = stickygamepad1.x ? Auto.AllianceColors.BLUE : Auto.AllianceColors.RED;
        telemetry.addData("ALLIANCE: ", allianceColor);
        if (stickygamepad1.dpad_up == tempUp) {
            tempUp = !tempUp;
            waitTime = Range.clip(waitTime+0.5,0,5);
        } else if (stickygamepad1.dpad_down == tempDown) {
            tempDown = !tempDown;
            waitTime = Range.clip(waitTime-0.5,0,5);
        }
        telemetry.addData("WAIT: ",waitTime);
        stickygamepad1.update();
        telemetry.update();
    }
    @Override
    public void start() {
        //TODO find these values
        if (allianceColor == Auto.AllianceColors.RED) {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(36, -63, Math.PI / 2));// Red start pos
        } else {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(36, 63, Math.PI * 3 / 2));// Blue start pos
        }
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        robot.mecanumDrive.updatePoseEstimate();
        switch (state){
            case WAIT:
                if (elapsedTime.seconds()>waitTime){
                    state = AutoStates.START_TO_FOUNDATION;
                    robot.mecanumDrive.follower.followTrajectory(startToFoundation());
                    elapsedTime.reset();
                }
                break;
            case START_TO_FOUNDATION:
                break;
            case MOVE_FOUNDATION:
                break;
        }
    }

    private Trajectory startToFoundation() {
        return null;
    }

    public enum AutoStates{
        WAIT, START_TO_FOUNDATION, MOVE_FOUNDATION,
    }
}
