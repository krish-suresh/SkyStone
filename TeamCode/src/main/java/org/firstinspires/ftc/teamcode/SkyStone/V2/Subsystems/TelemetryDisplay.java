package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class TelemetryDisplay implements Subsystem {
    public OpMode opMode;
    public Telemetry telemetry;
    char[][] display = new char[32][10];//Find Actual Dim
    FoundationDisplay foundation;


    public TelemetryDisplay(OpMode mode) {
        opMode = mode;
        telemetry = mode.telemetry;
        foundation = new FoundationDisplay();
    }

    public void updateDisplay() {
        //put foundation into display
        foundation.updateFoundation();
        addCharArraytoDisplay(foundation.returnArray(), 1, 1);

        for (char[] line : display) {
            telemetry.addLine(new String(line));
        }
    }
    public void addData(String key, String data){
        //TODO Impl
    }

    public void addCharArraytoDisplay(char[][] element, int x, int y) {
        if (element.length+y>display.length||element[0].length+x>display[0].length){telemetry.addLine("ERROR ELEMENT TOO BIG");return;}
        int rownum = 0;
        int colnum = 0;
        for (char[] row : element) {
            for (char val : row) {
                display[rownum+y][colnum+x] = val;
                colnum++;
            }
            rownum++;
        }
    }


    @Override
    public void update() {
        updateDisplay();
    }
    class FoundationDisplay{

        char[][] blankFoundation = {
                {'☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐'},
                {'☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐'},
                {'☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐'},
                {'☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐'}
        }; //4x16
        char[][] foundation = blankFoundation.clone();
        int[] blockPos = {0, 0};
        boolean isBlockVertical = false;
        public final char filledSquare = '■';
        public final char emptySquare = '☐';
        /*
         * ☐ ☐ ☐ ☐ ☐ ☐ ☐ ☐
         * ☐ ☐ ☐ ☐ ☐ ☐ ☐ ☐
         * ☐ ☐ ☐ ☐ ☐ ☐ ☐ ☐
         * ☐ ☐ ☐ ☐ ☐ ☐ ☐ ☐
         *
         *
         * */
        public FoundationDisplay(){
            //TODO Alliance color change pos of foundation
        }

        public void updateFoundation() {
            if (opMode.gamepad2.right_bumper) {
                isBlockVertical = !isBlockVertical;
            }
            blockPos[0] += (opMode.gamepad2.dpad_right ? 1 : (opMode.gamepad2.dpad_left ? -1 : 0));
            blockPos[1] += (opMode.gamepad2.dpad_up ? -1 : (opMode.gamepad2.dpad_down ? 1 : 0));
            com.qualcomm.robotcore.util.Range.clip(blockPos[0], 0, (isBlockVertical ? 7 : 6));
            com.qualcomm.robotcore.util.Range.clip(blockPos[1], 0, (isBlockVertical ? 2 : 3));
            foundation = addBlockPos(blockPos[0], blockPos[1], isBlockVertical);
        }

        /**
         * @param x          xPos from top left corner to left/top peg of stone 0-7
         * @param y          yPos from top left corner to left/top peg of stone 0-3
         * @param isVertical
         */
        public char[][] addBlockPos(int x, int y, boolean isVertical) {
            char[][] foundation = blankFoundation.clone();
            foundation[y][x * 2] = filledSquare;
            if (isVertical) {
                if (y != 3) {
                    y++;
                }
            } else {
                x++;
            }
            foundation[y][x * 2] = filledSquare;
            return foundation;
        }

        public char[][] returnArray() {
            return foundation.clone();
        }
    }
}
