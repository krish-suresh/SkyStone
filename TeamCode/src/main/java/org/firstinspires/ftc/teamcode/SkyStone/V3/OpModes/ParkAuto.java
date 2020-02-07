//V3

// TODO: make this based on RoadRunner paths

package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoTransitioner.AutoTransitioner;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;

@Autonomous (name = "ParkAuto")
public class ParkAuto extends OpMode {

    Robot robot;
    ElapsedTime time;

    double distance = 2;        // num of squares we move
    double waitTime = 0;        // seconds of wait before we move

    private boolean strafe = false;
    private Direction strafeDirection = Direction.LEFT;


    private boolean tempUp = true;
    private boolean tempDown = true;
    private boolean tempLeft = true;
    private boolean tempRight = true;
    private boolean tempA = true;
    private boolean tempB = true;


    private final double SQUARES_PER_SECOND = 2.2;

    private double strafeTime = 0.8;

    private double driveSpeed = 0.4;

    private States state = States.WAIT;


    @Override
    public void init() {
        robot = new Robot(this);
        time = new ElapsedTime();

        AutoTransitioner.transitionOnStop(this, "Tele");
    }

    @Override
    public void init_loop() {

        if (robot.stickyGamepad1.dpad_up == tempUp) {
            tempUp = !tempUp;
            waitTime = Range.clip(waitTime + 0.5, 0, 30);
        } else if (robot.stickyGamepad1.dpad_down == tempDown) {
            tempDown = !tempDown;
            waitTime = Range.clip(waitTime - 0.5, 0, 30);
        }

        if (robot.stickyGamepad1.dpad_left == tempLeft) {
            tempLeft = !tempLeft;
            waitTime = Range.clip(distance + 0.25, 0, 6);
        } else if (robot.stickyGamepad1.dpad_right == tempRight) {
            tempRight = !tempRight;
            waitTime = Range.clip(distance - 0.25, 0, 6);
        }

        if (robot.stickyGamepad1.a == tempA) {
            tempA = !tempA;
            strafe = !strafe;
        }

        if (robot.stickyGamepad1.b == tempB) {
            tempB = !tempB;
            strafeDirection = (strafeDirection == Direction.LEFT ? Direction.RIGHT : Direction.LEFT);
        }

        robot.telemetry.addData("Wait time: ", waitTime);
        robot.telemetry.addData("Distance (squares): ", distance);
        robot.telemetry.addData("Strafe? ", strafe);
        robot.telemetry.addData("Strafe Direction: ", strafeDirection);

        time.reset();
    }

    @Override
    public void loop() {

        switch (state) {

            case WAIT:
                if (time.seconds() > waitTime) {
                    state = States.DRIVE;
                    time.reset();
                }
                break;


            case DRIVE:
                robot.mecanumDrive.setMecanum(0, driveSpeed, 0);
                if (time.seconds() > squaresToSeconds(distance)) {
                    state = (strafe ? States.STRAFE : States.IDLE);
                }
                time.reset();
                break;


            case STRAFE:
                robot.mecanumDrive.setMecanum((strafeDirection == Direction.LEFT ? Math.PI / 2 : -Math.PI / 2), driveSpeed,0);
                if (time.seconds() > strafeTime) {
                    state = States.IDLE;
                }
                break;


            case IDLE:
                break;
        }

    }

    public double squaresToSeconds(double squares) {

        return squares / SQUARES_PER_SECOND;

    }

    public enum States {
        WAIT, DRIVE, STRAFE, IDLE
    }

    public enum Direction {
        LEFT, RIGHT
    }
}
