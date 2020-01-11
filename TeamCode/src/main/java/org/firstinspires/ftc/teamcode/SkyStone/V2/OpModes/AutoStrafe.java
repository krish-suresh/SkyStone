package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoTransitioner.AutoTransitioner;
import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

@Autonomous(name = "AutoStrafe")
public class AutoStrafe extends OpMode {

    Robot robot;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private StickyGamepad stickygamepad1;
    private ElapsedTime elapsedTime;

    final double TIME_TO_STRAFE = 1.0;

    Direction direction = Direction.LEFT;

    @Override
    public void init() {
        robot = new Robot(this);
        elapsedTime = new ElapsedTime();
        stickygamepad1 = new StickyGamepad(gamepad1);
        AutoTransitioner.transitionOnStop(this, "Tele");//transition from auto to tele when auto ends
    }

    @Override
    public void init_loop() {
        direction = stickygamepad1.x ? Direction.LEFT : Direction.RIGHT;
        telemetry.addData("Direction: ", direction);
        stickygamepad1.update();
        telemetry.update();
        elapsedTime.reset();
    }


    @Override
    public void loop() {
        if(elapsedTime.seconds() < TIME_TO_STRAFE) {
            robot.mecanumDrive.setMecanum(direction == Direction.LEFT ? Math.PI/2 : Math.PI*3/2,0.4,0);
        } else {
            robot.mecanumDrive.setMecanum(0,0,0);
        }
    }


    public enum Direction {
        LEFT,RIGHT
    }

}
