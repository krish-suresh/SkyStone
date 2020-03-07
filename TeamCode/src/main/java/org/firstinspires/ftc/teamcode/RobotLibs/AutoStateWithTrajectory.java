package org.firstinspires.ftc.teamcode.RobotLibs;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLibs.AutoState;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;

public abstract class AutoStateWithTrajectory extends AutoState {

    /**
     * Gets the trajectory for the current state
     * @return the trajectory to use in the follower
     */
    protected abstract Trajectory getTrajectory();

    /**
     * Gets the next state for oopState to become
     * Called once the robot arrives at the end of the Trajectory
     * @return the next state
     */
    public abstract AutoState getNextState();

    /**
     * Starts the follower with the provided path from getTrajectory
     */
    protected void initState(Robot robot) {
        inited = true;
        robot.mecanumDrive.follower.followTrajectory(getTrajectory());
    }

    /**
     * Moves oopState to the next state if the robot has arrived at the end of the trajectory
     * @return the AutoState for the next loop cycle
     */
    public AutoState doLoop(ElapsedTime time, Robot robot) {
        if (!inited) {
            initState(robot);
        }
        followTrajectory(robot);
        if (hasArrived(robot)) {
            time.reset();
            inited = false;
            return getNextState();
        } else {
            return this;
        }
    }

    public boolean hasArrived(Robot robot) {
        return !robot.mecanumDrive.follower.isFollowing();
    }


    public void followTrajectory(Robot robot) {
        robot.mecanumDrive.updateFollowingDrive();
    }

    boolean inited;     // used in doLoop() and init()

}
