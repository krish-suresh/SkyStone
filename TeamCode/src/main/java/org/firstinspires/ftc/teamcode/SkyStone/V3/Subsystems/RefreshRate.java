//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

import java.text.DecimalFormat;

public class RefreshRate implements Subsystem {
    Robot robot;
    ElapsedTime elapsedTime;
    int cycleCount;
    double lastTime;
    public double avgCycle;
    DecimalFormat df = new DecimalFormat("#.##");

    public RefreshRate() {
        robot = Robot.getInstance();
        elapsedTime = new ElapsedTime();

    }

    @Override
    public void update() {
        cycleCount++;
//        avgCycle = (avgCycle * (cycleCount - 1) + (1 / ((elapsedTime.nanoseconds() - lastTime) / 1000000000))) / cycleCount;
        avgCycle = 1.0 / ((elapsedTime.nanoseconds() - lastTime) / 1000000000.0);

        lastTime = elapsedTime.nanoseconds();
        robot.telemetry.addData("Avg Rate", df.format(avgCycle) + " hz");

    }
}
