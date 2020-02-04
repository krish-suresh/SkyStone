package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoTransitioner.AutoTransitioner;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

@Autonomous(name = "Forward")
public class AutoForward extends OpMode {
    Robot robot;
    private ElapsedTime elapsedTime;

    double timeToDrive = 1.0;
    double waitTime = 0;
    private boolean tempUp = true;
    private boolean tempDown = true;
    private boolean tempUp1 = true;
    private boolean tempDown2 = true;


    @Override
    public void init() {
        robot = new Robot(this);
        elapsedTime = new ElapsedTime();
        AutoTransitioner.transitionOnStop(this, "Tele");//transition from auto to tele when auto ends
        robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
    }

    public void init_loop() {
        if (robot.stickyGamepad1.dpad_up == tempUp) {
            tempUp = !tempUp;
            waitTime = Range.clip(waitTime + 0.5, 0, 30);
        } else if (robot.stickyGamepad1.dpad_down == tempDown) {
            tempDown = !tempDown;
            waitTime = Range.clip(waitTime - 0.5, 0, 30);
        }

        if (robot.stickyGamepad1.dpad_left == tempUp1) {
            tempUp1 = !tempUp1;
            timeToDrive = Range.clip(timeToDrive + 0.2, 0, 30);
        } else if (robot.stickyGamepad1.dpad_right == tempDown2) {
            tempDown2 = !tempDown2;
            timeToDrive = Range.clip(timeToDrive - 0.2, 0, 30);
        }
        telemetry.addData("Wait time: ", waitTime);

        telemetry.addData("Drive time: ", timeToDrive);

        telemetry.update();
        elapsedTime.reset();

    }


    @Override
    public void loop() {
        if (elapsedTime.seconds() < waitTime) {

        }
        else if(elapsedTime.seconds() < waitTime + timeToDrive) {
            robot.mecanumDrive.setMecanum(Math.PI,0.4,0);
        } else {
            robot.mecanumDrive.setMecanum(0,0,0);
        }
    }
}
