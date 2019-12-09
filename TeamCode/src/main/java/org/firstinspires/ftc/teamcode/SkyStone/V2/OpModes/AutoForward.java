package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoTransitioner.AutoTransitioner;
import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

@Autonomous(name = "Forward")
public class AutoForward extends OpMode {
    Robot robot;
    private ElapsedTime elapsedTime;

    final double TIME_TO_STRAFE = 1.0;


    @Override
    public void init() {
        robot = new Robot(this);
        elapsedTime = new ElapsedTime();
        AutoTransitioner.transitionOnStop(this, "Tele");//transition from auto to tele when auto ends
    }

    public void init_loop() {
        elapsedTime.reset();
    }


    @Override
    public void loop() {
        if(elapsedTime.seconds() < TIME_TO_STRAFE) {
            robot.mecanumDrive.setMecanum(Math.PI,0.4,0);
        } else {
            robot.mecanumDrive.setMecanum(0,0,0);
        }
    }
}
