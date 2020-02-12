package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLibs.JServo;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

import java.util.Timer;
import java.util.TimerTask;

public class AutoGrab implements Subsystem {

    public GrabState grabState = GrabState.OPEN;
    public RotateState rotateState = RotateState.UP;
    private FoundationState foundationState = FoundationState.UP;

    public JServo rotate;
    public JServo grab;
    public JServo foundationGrab;

    public Robot robot;

    public OpMode opmode;

    public final double ROTATE_UP = 0.85;
    public final double ROTATE_DOWN = 0.445;
    public final double GRAB_GRABBED = 0.35;
    public final double GRAB_UNGRABBED = 0.8;

    public final double FOUNDATION_DOWN = 0.15;
    public final double FOUNDATION_UP = 0.85;
    public final double FOUDNATION_MID = 0.15;

    public final static double GRAB_DIFF_TIME = 0.15;       // difference in time btw grab and rotate servos starting for grabbing blocks effectively
    public final static double GRAB_TIME = 0.3;            // total time needed to go from up + open to down + grabbed on block
    public final static double PICK_UP_TIME = 0.4;         // total time needed to go from down + grabbed on block to up + grabbed
    public final static double PLACE_TIME = 0.4;           // total time needed to go from up + grabbed to down + open

    public boolean tempLeft = true;
    public boolean tempRight = true;

    ElapsedTime time;

    public AutoGrab(OpMode mode) {
        opmode = mode;
        rotate = new JServo(mode.hardwareMap, "Rotate");
        grab = new JServo(mode.hardwareMap, "Grab");
        foundationGrab = new JServo(mode.hardwareMap, "Foundation");
        robot = Robot.getInstance();
        time = new ElapsedTime();

    }

    @Override
    public void update() {

        // buttons for Tele
        // toggle grab
        if (robot.stickyGamepad1.left_bumper == tempLeft) {

            if (grabState == GrabState.GRAB) {
                grabState = GrabState.OPEN;
            } else {
                grabState = GrabState.GRAB;
            }
            tempLeft = !tempLeft;
        }

        //toggle rotate
        if (robot.stickyGamepad1.right_bumper == tempRight) {
            if (rotateState == RotateState.DOWN) {
                rotateState = RotateState.UP;
            } else {
                rotateState = RotateState.DOWN;
            }
            tempRight = !tempRight;
        }


        // set servos to appropriate positions based on current grabState and foundationState
        switch (grabState) {

            case OPEN:
                grab.setPosition(GRAB_UNGRABBED);
                break;


            case GRAB:
                grab.setPosition(GRAB_GRABBED);
                break;

        }

        switch (rotateState) {
            case UP:
                rotate.setPosition(ROTATE_UP);
                break;
            case DOWN:
                rotate.setPosition(ROTATE_DOWN);
                break;
        }


        // set foundation servo to appropriate place based on foundationState
        switch (foundationState) {

            case UP:
                foundationGrab.setPosition(FOUNDATION_UP);
                break;


            case DOWN:
                foundationGrab.setPosition(FOUNDATION_DOWN);
                break;


            case MID:
                foundationGrab.setPosition(FOUDNATION_MID);
                break;
        }

        robot.telemetry.addData("Grab State", grabState);

    }


    public void setGrabState(GrabState grabState) {
        this.grabState = grabState;
        this.update();
    }

    public void setRotateState(RotateState rotateState) {
        this.rotateState = rotateState;
        this.update();
    }

    public void setFoundationState(FoundationState foundationState) {
        this.foundationState = foundationState;
        this.update();
    }

    public enum GrabState {
        OPEN,
        GRAB
    }

    public enum RotateState {
        UP,
        DOWN
    }

    public enum FoundationState {
        UP,
        DOWN,
        MID
    }

}
