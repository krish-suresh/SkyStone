package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotLibs.JMotor;
import org.firstinspires.ftc.teamcode.RobotLibs.JServo;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class DepositLift implements Subsystem {

    /**
     * Figure out why extend didn't happen sometimes
     * Tune PID
     * Timer for clicking a to bring extend in
     */

    private final double LIFTTIME = .25;
    private final double DROPTIME = .15;
    private double WAITTIME = 0.5;
    private final double LINKAGE_ARM_1 = 6.545;
    private final double LINKAGE_ARM_2 = 7.315;

    private final double SPOOL_DIAMETER = 1;
    private static final double TICKS_PER_REV = 44.4;

    private final double GRAB_CLOSE = 0.18;
    private final double GRAB_OPEN = 0.73;

    private JMotor liftMotorRight;
    private JMotor liftMotorLeft;
    private JServo grab;
    public JServo extendL;
    public JServo extendR;

    private OpMode opMode;
    Robot robot;

    private double liftHeight = 0;
    private double liftBottomCal = 0;
    private double liftPower = 0;
    private int targetLevel = 1;
    private double targetHeight;

    private boolean tempDown = true;
    private boolean tempUp = true;
    private boolean tempRight = true;
    private boolean tempLeft = true;

    public LiftControlStates liftState = LiftControlStates.MANUAL;
    private ExtendStates extendState = ExtendStates.TELE_GRAB;

    public static double kP = 0.25;
    public static double kI = 0.01;
    public static double kD = 0.008;
    public PIDFController pidAutonomous = new PIDFController(new PIDCoefficients(kP, kI, kD));

    private ElapsedTime time;

    private double liftStartCal;
    private double FOUNDATION_HEIGHT = 4;
    private boolean goingUp;

    public DepositLift(OpMode mode) {
        opMode = mode;

        liftMotorRight = new JMotor(mode.hardwareMap, "L.R");
        liftMotorLeft = new JMotor(mode.hardwareMap, "L.L");
        liftMotorRight.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftStartCal = getRelLiftHeight();

        grab = new JServo(mode.hardwareMap, "D.G");
        extendL = new JServo(mode.hardwareMap, "D.E1");
        extendR = new JServo(mode.hardwareMap, "D.E2");

        pidAutonomous.setOutputBounds(-1, 1);

        time = new ElapsedTime();

        grab.setPosition(GRAB_OPEN);

        robot = Robot.getInstance();
    }

    @Override
    public void update() {

        liftHeight = getAbsLiftHeight() - liftBottomCal;

        setTargetHeight();

        setLiftState();

        checkForLiftHeightReset();

        calibrateLiftBottom();

        setExtendState();

        switch (liftState) {

            case MANUAL:
                liftPower = (-opMode.gamepad2.right_stick_y) +0.23; // TODO TEST WHY LIFTPOWER IS NEG
                if (extendState == ExtendStates.STRAIGHT_PLACE && opMode.gamepad2.right_stick_y > 0) {
                    liftPower = -(opMode.gamepad2.right_stick_y / 3) + 0.23;
                }
//                if (liftHeight < 2 && opMode.gamepad2.right_stick_y <= 0.2) {       // don't do the feedforward constant if the lift is low
//                    liftPower -= 0.23;
//                }
                break;


            case HOLD:

                liftPower = 0.23;       // power to hold the lift in place
                setExtendState();       // set extendState based on autoPlaceState and GP2's right and left triggers
                break;


            case GRAB_BLOCK:
                double secondsGB = time.seconds();
                if (secondsGB < WAITTIME) {
                } else if (secondsGB < WAITTIME + LIFTTIME) {
                    liftPower = -0.6;
                } else if (secondsGB < WAITTIME + LIFTTIME + DROPTIME) {
                    liftPower = 0;
                    robot.stickyGamepad2.right_bumper = true;
                } else {
                    liftState = LiftControlStates.HOLD;
                    targetHeight = FOUNDATION_HEIGHT + (targetLevel * 4);
                }
                break;


            case START_AUTOLIFT:
                if (targetHeight > liftHeight) {
                    goingUp = true;
                } else {
                    goingUp = false;
                }
                time.reset();
                pidAutonomous.setTargetPosition(targetHeight);
                pidAutonomous.reset();
                liftState = LiftControlStates.AUTOLIFT;
                break;


            case AUTOLIFT:
                liftPower = pidAutonomous.update(liftHeight) + 0.2;
                if ((goingUp ? (liftHeight - targetHeight) : (targetHeight - liftHeight)) >= .2) {
                    liftState = LiftControlStates.HOLD;
                    pidAutonomous.reset();
                    extendState = ExtendStates.STRAIGHT_PLACE;
                }
                break;

        }

        setExtend(extendState);

        // prevent the lift from being shoved into the ground by ensuring positive liftPower when liftHeight is < 0
        updateLiftPower((liftHeight < 0 && !opMode.gamepad2.right_stick_button) ?
                Range.clip(liftPower, 0, 1) :
                liftPower);

        // open and close grab with right bumper on GP2
        grab.setPosition(robot.stickyGamepad2.right_bumper ?
                GRAB_CLOSE :
                GRAB_OPEN);

        robot.telemetry.addData("LIFT Target Level: ", targetLevel);
        robot.telemetry.addData("LIFT STATE", liftState);
        robot.telemetry.addData("LIFT Current Height", liftHeight);
        robot.telemetry.addData("LIFT Target Height", targetHeight);
        robot.telemetry.addData("Lift power", liftPower);

    }


    /*--------------------------------------------------------------------------------------------------------------------------*/
    /* Update Methods / Methods that check for Gamepad input to change robot state */
    /*--------------------------------------------------------------------------------------------------------------------------*/


    private void setTargetHeight() {
        //set target height from d pad
        if (robot.stickyGamepad2.dpad_up == tempUp) {
            tempUp = !tempUp;
            targetLevel++;
            targetLevel = targetLevel % 15;       // make sure the target level is in the range that we can place
            targetHeight = 3.5 + (targetLevel * 4);

        } else if (robot.stickyGamepad2.dpad_down == tempDown) {
            tempDown = !tempDown;
            targetLevel--;
            //makes sure the target level is in the range that we can place
            targetLevel = Range.clip(targetLevel, 0, 14);
            targetHeight = 3 + (targetLevel * 4);
        }
    }


    private void setLiftState() {
        //if x is pressed pid to target height if not gpad input
        if (opMode.gamepad2.x && liftState != LiftControlStates.AUTOLIFT) {
            liftState = LiftControlStates.START_AUTOLIFT;
        } else if (Math.abs(opMode.gamepad2.right_stick_y) > 0.05) {
            liftState = LiftControlStates.MANUAL;
        } else if (liftState == LiftControlStates.MANUAL) {
            liftState = LiftControlStates.HOLD;
        }
    }


    private void setExtendState() {
        if (opMode.gamepad2.left_trigger > 0.1) {
            extendState = ExtendStates.TELE_GRAB;
        } else if (opMode.gamepad2.right_trigger > 0.1) {
            extendState = ExtendStates.STRAIGHT_PLACE;
        }
    }


    private void checkForLiftHeightReset() {
        // if the a-button is pressed, zero the lift
        if (opMode.gamepad2.a) {
            targetHeight = 0;
            liftState = LiftControlStates.START_AUTOLIFT;
            extendState = ExtendStates.TELE_GRAB;
            robot.stickyGamepad2.right_bumper = false;
        }
    }


    private void calibrateLiftBottom() {
        if (opMode.gamepad2.left_stick_button) {
            liftBottomCal = getAbsLiftHeight();
        }
    }


    /*--------------------------------------------------------------------------------------------------------------------------*/
    /* Setter Methods / Methods that change the robot's physical state */
    /*--------------------------------------------------------------------------------------------------------------------------*/


    public void setExtend(ExtendStates extendState) {
        switch (extendState) {

            case TELE_GRAB:             // position for grab
                setExtendPos(0.19);
                break;

            case STRAIGHT_PLACE:        // normal straight place
                setExtendPos(0.58);
                break;

            case GRAB_AUTO:           // distance we grab the block from in auto
                setExtendPos(0.85);
                break;

            case FULL_EXTEND:         // where we bring the block into when we strafe in auto
                setExtendPos(0.65);
                break;
        }
    }

    public void setExtend(double inchesAbs) {
        double DISTANCE_FROM_CENTER = 4.23765;
        double inchesRel = inchesAbs - DISTANCE_FROM_CENTER;
        setExtendRel(inchesRel);
    }


    public void setExtendRel(double inchesRel) {
        //this is the angle at which the servo should be at in order to extend that specific number of inches
        double START_POS = .43;
        double TOTAL_RANGE = .85 - START_POS;
        double TOTAL_ANGLE = 80;//TODO FIND THIS
        double START_ANGLE = 12.18;//deg

        double degAbs = Math.acos((Math.pow(LINKAGE_ARM_1, 2) + Math.pow(inchesRel, 2) - Math.pow(LINKAGE_ARM_2, 2)) / (2 * inchesRel * LINKAGE_ARM_1));
        double degRel = START_ANGLE - degAbs;

        double pos = START_POS + TOTAL_RANGE * degRel / TOTAL_ANGLE;
        setExtendPos(pos);
    }


    public void setExtendPos(double pos) {
        extendL.setPosition(pos);
        extendR.setPosition(1 - pos);
    }


    public void setExtendPos(double pos1, double pos2) {
        extendL.setPosition(pos1);
        extendR.setPosition(pos2);
    }


    public void setTargetHeight(double height) {
        pidAutonomous.setTargetPosition(height);
    }


    public void grabStone() {
        grab.setPosition(GRAB_CLOSE);

    }


    public void releaseStone() {
        grab.setPosition(GRAB_OPEN);
    }


    public void updateLiftPower(double power) {
        liftMotorLeft.setPower(power);
        liftMotorRight.setPower(power);
    }


    /*--------------------------------------------------------------------------------------------------------------------------*/
    /* Getter Methods / Methods that return some aspect about the robot's physical state */
    /*--------------------------------------------------------------------------------------------------------------------------*/


    public double getRelLiftHeight() {

        return Math.PI * (SPOOL_DIAMETER / 2) * (liftMotorRight.getCurrentPosition() / TICKS_PER_REV);
    }


    public double getAbsLiftHeight() {

        return Math.PI * (SPOOL_DIAMETER / 2) * (liftMotorRight.getCurrentPosition() / TICKS_PER_REV) - liftStartCal;
    }


    public double getAbsExtend() {
        return 0;//TODO IMP
    }


    public enum LiftControlStates {
        MANUAL, HOLD, GRAB_BLOCK, START_AUTOLIFT, AUTOLIFT
    }


    //TODO ADD STATES FOR ROTATION & GRAB
    public enum ExtendStates {

        TELE_GRAB(0.25),
        STRAIGHT_PLACE(0.75),
        GRAB_AUTO(0.75),
        FULL_EXTEND(0.1);

        private double extendTime;

        ExtendStates(double time) {
            this.extendTime = time;
        }

        public double getExtendTime() {
            return extendTime;
        }
    }
}

/**
 * NO ROTATE
 * a - zero the lift - extend in, lower, open, increments targetHeight
 * x - extend to position and target height
 * right bumper - open grab
 * left bumper - drop capstone
 * left/right trigger - extend/retract
 * b - does nothing
 * y - does nothing
 * dpad up/down sets height
 */
