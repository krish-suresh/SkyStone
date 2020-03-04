package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLibs.JServo;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class AutoGrab implements Subsystem {

    public GrabState grabState = GrabState.GRAB;
    public RotateState rotateState = RotateState.UP;
    public TurnState turnState = TurnState.MIDDLE;

    public JServo grab;
    public JServo rotate;
    public JServo turn;

    public Robot robot;

    public OpMode opmode;

    public final double ROTATE_UP = 0.58;
    public final double ROTATE_MID = 0.4568209;
    public final double ROTATE_DOWN = 0.24;

    public final double GRAB_GRABBED = 0.32;
    public final double GRAB_UNGRABBED = 0.65;

    public final double TURN_LEFT = 0.25;
    public final double TURN_MIDDLE = 0.55;
    public final double TURN_RIGHT = 0.9;

    public final static double GRAB_DIFF_TIME = 0.3;      // difference in time btw grab and rotate servos starting for grabbing blocks effectively
    public final static double GRAB_TIME = 0.9;            // total time needed to go from up + open to down + grabbed on block
    public final static double PICK_UP_TIME = 0.4;         // total time needed to go from down + grabbed on block to up + grabbed
    public final static double PLACE_TIME = 0.5;           // total time needed to go from up + grabbed to down + open

    public boolean tempLeft = true;
    public boolean tempRight = true;
    public boolean tempUp = true;
    public boolean tempDown = true;

    ElapsedTime time;

    public AutoGrab(OpMode mode) {
        opmode = mode;

        grab = new JServo(mode.hardwareMap, "Grab");
        rotate = new JServo(mode.hardwareMap, "Rotate");
        turn = new JServo(mode.hardwareMap, "Turn");

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

        //toggle turn
        if (robot.stickyGamepad1.dpad_right == tempUp) {
            if (turnState == TurnState.LEFT) {
                turnState = TurnState.MIDDLE;
            } else {
                turnState = TurnState.RIGHT;
            }
            tempUp = !tempUp;
        } else if (robot.stickyGamepad1.dpad_left == tempDown) {
            if (turnState == TurnState.RIGHT) {
                turnState = TurnState.MIDDLE;
            } else {
                turnState = TurnState.LEFT;
            }
            tempDown = !tempDown;
        }


        // set servos to appropriate positions based on current grabState, rotateState, turnState and foundationState
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

            case MIDDLE:
                rotate.setPosition(ROTATE_MID);
                break;

            case DOWN:
                rotate.setPosition(ROTATE_DOWN);
                break;
        }

        switch (turnState) {
            case LEFT:
                turn.setPosition(TURN_LEFT);
                break;

            case RIGHT:
                turn.setPosition(TURN_RIGHT);
                break;

            case MIDDLE:
                turn.setPosition(TURN_MIDDLE);
                break;
        }

//        robot.telemetry.addData("Grab State", grabState);
//        robot.telemetry.addData("Rotate State", rotateState);
//        robot.telemetry.addData("Turn State", turnState);

    }


    public void setGrabState(GrabState grabState) {
        this.grabState = grabState;
        this.update();
    }

    public void setRotateState(RotateState rotateState) {
        this.rotateState = rotateState;
        this.update();
    }

    public void setTurnState(TurnState turnState) {
        this.turnState = turnState;
        this.update();
    }

    public enum GrabState {
        OPEN,
        GRAB
    }

    public enum RotateState {
        UP,
        MIDDLE,
        DOWN
    }

    public enum TurnState {
        LEFT,
        RIGHT,
        MIDDLE
    }

}
