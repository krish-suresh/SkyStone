package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

@Autonomous(name = "DumbAuto")
public class DumbAuto extends OpMode {
    Robot robot;
    private StickyGamepad stickygamepad1 = new StickyGamepad(gamepad1);

    ElapsedTime elapsedTime;

    double driveTime = 1;
    double bridgeTime = 2.7;
    double grabTime = 1.5;
    double strafeTime = 0.7;

    double forward = Math.PI;
    double backward = 0;
    double right = Math.PI * 1.0/2.0;
    double left = Math.PI * 3.0/2.0;

    AllianceColors allianceColor;


    @Override
    public void init() {
        robot = new Robot(this);
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        stickygamepad1 = new StickyGamepad(gamepad1);

    }

    public void init_loop() {
        allianceColor = (stickygamepad1.x) ? AllianceColors.BLUE : AllianceColors.RED;
        stickygamepad1.update();
        //"loop"

        //add telemetry for alliance state

        telemetry.addData("Alliance:", "" + allianceColor);
        elapsedTime.reset();
    }

    @Override
    public void loop() {

        //strafe sideways to line up
        if (elapsedTime.seconds() < 0.1) {
            robot.mecanumDrive.setMecanum(backward, 0.5, 0);


            telemetry.addData("State","Strafe");
        }
        else if (elapsedTime.seconds() < strafeTime) {
            if (allianceColor == AllianceColors.BLUE) {
                robot.mecanumDrive.setMecanum(left, 0.5, 0);
            } else {
                robot.mecanumDrive.setMecanum(right, 0.5, 0);
            }

            telemetry.addData("State","Strafe");
        }


        //drive from platform to wall
        else if (elapsedTime.seconds() < driveTime + strafeTime) {
            robot.mecanumDrive.setMecanum(backward, 0.45, 0);

            telemetry.addData("State","FP");
        }
        //wait for grab (.5 seconds?)
        /*else*/
        else if (elapsedTime.seconds() < driveTime + grabTime+ strafeTime) {
            robot.mecanumDrive.platformGrab();
            robot.mecanumDrive.setMecanum(forward, 0, 0);

            telemetry.addData("State","GP");
        }
        //drive with the platform to wall
        else if (elapsedTime.seconds() < driveTime * 2 + grabTime+ strafeTime+0.5) {
            robot.mecanumDrive.setMecanum(forward, 0.5, 0);

            telemetry.addData("State","BP");
        }
        //wait to release platform (.5 seconds?)
        else if (elapsedTime.seconds() < driveTime * 2 + grabTime * 2+ strafeTime+1) {
            robot.mecanumDrive.setMecanum(forward, 0, 0);
            robot.mecanumDrive.platformRelease();

            telemetry.addData("State","RP");
        }

        //wait for 20 seconds so we don't get in anyone's way

//        else if (elapsedTime.seconds() < driveTime * 2 + grabTime * 2 + 20+ strafeTime) {
//            robot.mecanumDrive.setMecanum(forward, 0, 0);
//
//            telemetry.addData("State","WB");
//        }

        //drive from wall to under bridge (alliance color dependent)

        else if (elapsedTime.seconds() < driveTime * 2 + grabTime * 2 /*+ 20*/ + bridgeTime+ strafeTime) {
            if (allianceColor == AllianceColors.BLUE) {
                robot.mecanumDrive.setMecanum(right, 0.5, 0);
            } else {
                robot.mecanumDrive.setMecanum(left, 0.5, 0);
            }

            telemetry.addData("State","sB");
        }

        //stop
        else {
            robot.mecanumDrive.setMecanum(Math.PI * 3.0 / 2.0, 0, 0);
            telemetry.addData("State","stop");
        }

        telemetry.update();
    }

    public enum AllianceColors {

        RED, BLUE
    }

}
