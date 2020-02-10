package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.JServo;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class AutoGrab implements Subsystem {

    private GrabState grabState = GrabState.OPEN_UP;
    private FoundationState foundationState = FoundationState.UP;

    public JServo rotate;
    public JServo grab;
    public JServo foundationGrab;

    public Robot robot;

    public OpMode opmode;

    public final double ROTATE_UP = 0.85;
    public final double ROTATE_DOWN = 0.15;
    public final double GRAB_GRABBED = 0.85;
    public final double GRAB_UNGRABBED = 0.15;

    public final double FOUNDATION_DOWN = 0.15;
    public final double FOUNDATION_UP = 0.85;
    public final double FOUDNATION_MID = 0.15;

    public AutoGrab(OpMode mode) {
        opmode = mode;
        rotate = new JServo(mode.hardwareMap, "Rotate");
        grab = new JServo(mode.hardwareMap, "Grab");
        foundationGrab = new JServo(mode.hardwareMap, "Foundation");
        robot = Robot.getInstance();

    }

    @Override
    public void update() {

        // buttons for Tele
        // toggle grab
        if (robot.stickyGamepad1.left_stick_button) {
            if (grabState == GrabState.GRAB_DOWN) {
                grabState = GrabState.OPEN_DOWN;
            } else if (grabState == GrabState.OPEN_DOWN) {
                grabState = GrabState.GRAB_DOWN;
            } else if (grabState == GrabState.OPEN_UP) {
                grabState = GrabState.GRAB_UP;
            } else {
                grabState = GrabState.OPEN_UP;
            }
        }

        //toggle rotate
        if (robot.stickyGamepad1.right_stick_button) {
            if (grabState == GrabState.GRAB_DOWN) {
                grabState = GrabState.GRAB_UP;
            } else if (grabState == GrabState.OPEN_DOWN) {
                grabState = GrabState.OPEN_UP;
            } else if (grabState == GrabState.OPEN_UP) {
                grabState = GrabState.OPEN_DOWN;
            } else {
                grabState = GrabState.GRAB_DOWN;
            }
        }



        // set servos to appropriate positions based on current grabState and foundationState
        switch (grabState) {

            case OPEN_UP:
                rotate.setPosition(ROTATE_UP);
                grab.setPosition(GRAB_UNGRABBED);
                break;


            case OPEN_DOWN:
                rotate.setPosition(ROTATE_DOWN);
                grab.setPosition(GRAB_UNGRABBED);
                break;


            case GRAB_UP:
                rotate.setPosition(ROTATE_UP);
                grab.setPosition(GRAB_GRABBED);
                break;


            case GRAB_DOWN:
                rotate.setPosition(ROTATE_DOWN);
                grab.setPosition(GRAB_GRABBED);
                break;
        }

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

    }


    public void setGrabState(GrabState grabState) {
        this.grabState = grabState;
    }

    public void setFoundationState(FoundationState foundationState) {
        this.foundationState = foundationState;
    }

    public enum GrabState {
        OPEN_UP,
        OPEN_DOWN,
        GRAB_UP,
        GRAB_DOWN
    }

    public enum FoundationState {
        UP, DOWN, MID
    }

}
