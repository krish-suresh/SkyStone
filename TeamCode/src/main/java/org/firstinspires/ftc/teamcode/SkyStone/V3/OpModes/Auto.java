//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;

@Autonomous (name = "Auto")
public class Auto extends OpMode {

    // Start object/value creation
    Robot robot;
    Camera camera;

    boolean tempUp = true;      // used for waitTime setting
    boolean tempDown = true;

    int skystone;       // from Camera, which position (0, 1, 2) the skystone is

    boolean allianceColorIsRed;


    double waitTime = 0;

    AutoState oopState;

    ElapsedTime time;
    //End object/value creation



    @Override
    public void init() {
        camera = new Camera(this);
        time = new ElapsedTime();

        oopState = new Wait();
        // Initializations

    }

    @Override
    public void init_loop() {
        updateWaitTime();       // increase / decrease wait time with GP1's dpad up and dpad down
        updateAllianceColor();  // flip allianceColor based on gamepad1.x
        skystone = camera.getSkyPos(allianceColorIsRed);

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



        // OOP auto
        oopState = oopState.doLoop();
        robot.telemetry.addLine("Current State is " + oopState.getStateName());
        robot.telemetry.update();

    }



    // base class for all AutoState classes such as Wait, Grab
    private abstract class AutoState {

        /**
         * Does methods of subclasses
         * @return what state to run next loop
         */
        public abstract AutoState doLoop();

        /**
         * Used for telemetry of which state the robot is in
         * @return the name of the current state
         */
        public String getStateName() {
            return this.getClass().getName();
        }

    }


    // wait a specified time before starting Auto
    private class Wait extends AutoState {

        @Override
        public AutoState doLoop() {

            if (time.seconds() < waitTime) {
                return this;
            } else {
                return WALL_TO_FIRST_BLOCK;
            }

        }
    }


    private WallToFirstBlock WALL_TO_FIRST_BLOCK = new WallToFirstBlock();
    private class WallToFirstBlock extends AutoState {

        private WallToFirstBlock() {
            robot.mecanumDrive.follower.followTrajectory(getTrajectory());
        }

        @Override
        public AutoState doLoop() {
            if (hasArrived()) {
                return GRAB;
            } else {
                return this;
            }
        }

        private Trajectory getTrajectory() {
            return null;
        }

    }


    private Grab GRAB = new Grab();
    private class Grab extends AutoState {

        private double grabTime = 0.6;

        @Override
        public AutoState doLoop() {
            if (time.seconds() < grabTime) {
                grabBlock();
                return this;
            } else {
                return STONES_TO_FOUNDATION_1;
            }
        }

        private void grabBlock() {

        }

    }


    private StonesToFoundation1 STONES_TO_FOUNDATION_1 = new StonesToFoundation1();
    private class StonesToFoundation1 extends AutoState {

        public StonesToFoundation1() {
            robot.mecanumDrive.follower.followTrajectory(getTrajectory());
        }

        @Override
        public AutoState doLoop() {
            if (hasArrived()) {
                return MOVE_FOUNDATION_1;
            } else {
                return this;
            }
        }

        public Trajectory getTrajectory() {
            return null;
        }
    }


    private MoveFoundation1 MOVE_FOUNDATION_1 = new MoveFoundation1();
    private class MoveFoundation1 extends AutoState {

        private MoveFoundation1() {
            robot.mecanumDrive.follower.followTrajectory(getTrajectory());
        }

        @Override
        public AutoState doLoop() {
            if (hasArrived()) {
                return null;
            } else {
                return this;
            }
        }

        private Trajectory getTrajectory() {
            return null;
        }
    }















    private PlaceStone PLACE_STONE = new PlaceStone();
    private class PlaceStone extends AutoState {

        @Override
        public AutoState doLoop() {
            return null;
        }
    }


    private boolean hasArrived() {
        return !robot.mecanumDrive.follower.isFollowing();
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

    // flip allianceColor based on gamepad1.x
    private void updateAllianceColor() {
        allianceColorIsRed = robot.stickyGamepad1.x;
    }


//    WAIT,                           // wait a specified time before starting Auto
//    WALL_TO_FIRST_BLOCK,            // path from wall to the skystone detected closest to the bridge
//    GRAB,                           // grab the block in front of the robot (no xy movement)
//    FIRST_STONE_TO_FOUNDATION,      // path from the first stone to the foundation starting position
//    MOVE_FOUNDATION_1,              // move the foundation from starting position to close to the bridge
//    FOUNDATION_TO_STONES,           // path from foundation to next stone
//    STONES_TO_FOUNDATION,           // path from stones to foundation by bridge
//    PLACE_STONE,                    // place a stone on the foundation (no xy movement)
//    MOVE_FOUNDATION_2,              // move the foundation from close to the bridge back, turn, and push against wall
//    PARK,                           // activate scissor park once the foundation is in place
//    IDLE                            // end phase once robot is parked - does nothing



}
