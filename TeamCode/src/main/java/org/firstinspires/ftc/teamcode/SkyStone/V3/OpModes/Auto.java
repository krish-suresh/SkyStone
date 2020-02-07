package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous (name = "Auto")
public class Auto extends OpMode {


    @Override
    public void init() {
        // Initializations
    }

    @Override
    public void init_loop() {
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




    }

    public enum AutoStates {

        

    }
}
