//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;

@Autonomous (name = "Auto")
public class Auto extends OpMode {

    // Start object/value creation
    Robot robot;

    boolean tempUp = true;      // used for waitTime setting
    boolean tempDown = true;

    int skystone;       // from Camera, which position (0, 1, 2) the skystone is

    AllianceColor allianceColor;

    double waitTime = 0;

    AutoStates state = AutoStates.WAIT;
    //End object/value creation



    ElapsedTime time;
    @Override
    public void init() {
        time = new ElapsedTime();

        // Initializations

    }

    @Override
    public void init_loop() {
        updateWaitTime();       // increase / decrease wait time with GP1's dpad up and dpad down

        allianceColor = (robot.stickyGamepad1.x ? AllianceColor.RED : AllianceColor.BLUE);      // toggle between alliance colors with GP1's x button

        // Vision

    }

    @Override
    public void loop() {

        /*
        *   Auto flow
        *
        *   Wait
        *   Stone 1
        *       Wall to close Skystone
        *       Grab stone
        *       Stone to foundation pos1
        *       Deposit block
        *       Grab foundation
        *       Move foundation closer to bridge
        *   Stone 2
        *       Foundation to next Skystone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block
        *   Stone 3
        *       Foundation to closest stone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block
        *   Stone 4
        *       Foundation to closest stone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block
        *   Stone 5
        *       Foundation to closest stone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block
        *   Stone 6
        *       Foundation to closest stone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block
        *       Grab foundation
        *       Pull back, turn, and push foundation into corner
        *       Scissor park
        *   Idle
        *
        * */

        switch(state) {

            case WAIT:
                break;
            case WALL_TO_FIRST_BLOCK:
                break;
            case GRAB:
                break;
            case FIRST_STONE_TO_FOUNDATION:
                break;
            case MOVE_FOUNDATION_1:
                break;
            case FOUNDATION_TO_STONES:
                break;
            case STONES_TO_FOUNDATION:
                break;
            case PLACE_STONE:
                break;
            case MOVE_FOUNDATION_2:
                break;
            case PARK:
                break;
            case IDLE:
                break;

        }


    }

    public void updateWaitTime() {
        if (robot.stickyGamepad1.dpad_up == tempUp) {
            waitTime += 0.5;
            tempUp = !tempUp;
        }
        if (robot.stickyGamepad1.dpad_down == tempDown) {
            waitTime -= 0.5;
            tempDown = !tempDown;
        }
    }

    public enum AutoStates {
        WAIT,                           // wait a specified time before starting Auto
        WALL_TO_FIRST_BLOCK,            // path from wall to the skystone detected closest to the bridge
        GRAB,                           // grab the block in front of the robot (no xy movement)
        FIRST_STONE_TO_FOUNDATION,      // path from the first stone to the foundation starting position
        MOVE_FOUNDATION_1,              // move the foundation from starting position to close to the bridge
        FOUNDATION_TO_STONES,           // path from foundation to next stone
        STONES_TO_FOUNDATION,           // path from stones to foundation by bridge
        PLACE_STONE,                    // place a stone on the foundation (no xy movement)
        MOVE_FOUNDATION_2,              // move the foundation from close to the bridge back, turn, and push against wall
        PARK,                           // activate scissor park once the foundation is in place
        IDLE                            // end phase once robot is parked - does nothing

    }

    public enum AllianceColor {
        RED, BLUE
    }
}
