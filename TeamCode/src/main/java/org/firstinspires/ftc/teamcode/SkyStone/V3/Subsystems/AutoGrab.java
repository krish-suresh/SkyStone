package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.JServo;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class AutoGrab implements Subsystem {

    private GrabState state = GrabState.OPEN_UP;

    public JServo rotate;
    public JServo grab;

    public Robot robot;

    public OpMode opmode;

    public final double ROTATE_UP = 0.85;
    public final double ROTATE_DOWN = 0.15;
    public final double GRAB_GRABBED = 0.85;
    public final double GRAB_UNGRABBED = 0.15;

    public AutoGrab(OpMode mode) {
        opmode = mode;
        rotate = new JServo(mode.hardwareMap, "Rotate");
        grab = new JServo(mode.hardwareMap, "Grab");
        robot = Robot.getInstance();

    }

    @Override
    public void update() {

        switch (state) {

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

    }

    public void setState(GrabState state) {
        this.state = state;
    }

    public enum GrabState {
        OPEN_UP,
        OPEN_DOWN,
        GRAB_UP,
        GRAB_DOWN
    }

}
