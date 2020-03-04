//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotLibs.JMotor;
import org.firstinspires.ftc.teamcode.RobotLibs.JServo;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

@Config
public class DepositLiftOld implements Subsystem {

    private final double LIFTTIME = .25;
    private final double DROPTIME = .15;
    private double WAITTIME = 0.5;
    private final double LINKAGE_ARM_1 = 6.545;     // TODO FILL THESE IN
    private final double LINKAGE_ARM_2 = 7.315;     // TODO FILL THESE IN

    private final double SPOOL_DIAMETER = 1.25;
    private static final double TICKS_PER_REV = 44.4;

    public final double ROTATION_ROTATED = 1;
    public final double ROTATION_STRAIGHT = 0.47;
    private final double GRAB_CLOSE = 0.18;
    private final double GRAB_OPEN = 0.6;
    private final double GRAB_OPEN_WIDE = 0.6;

    private JMotor liftMotorRight;
    private JMotor liftMotorLeft;
    private JServo grab;
    public JServo rotation;
    public JServo extendL;
    public JServo extendR;
    public DistanceSensor blockSensor;

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
    private ExtendStates extendState = ExtendStates.FULL_BACK;
    private AutoPlaceStates autoPlaceState;
    private int autoPlaceType = 0;

    public static double kP = 0.15;
    public static double kI = 0.01;
    public static double kD = 0.008;
    public PIDFController pidAutonomous = new PIDFController(new PIDCoefficients(kP, kI, kD));

    private ElapsedTime time;

    private double liftStartCal;
    private boolean autoPlaceStarted = false;
    private boolean isStoneGrabbed = false;

    private boolean extend2;

    public boolean isSlowMode;


    public DepositLiftOld(OpMode mode) {
        opMode = mode;

        liftMotorRight = new JMotor(mode.hardwareMap, "L.R");
        liftMotorLeft = new JMotor(mode.hardwareMap, "L.L");
        liftMotorRight.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftStartCal = getRelLiftHeight();

        grab = new JServo(mode.hardwareMap, "D.G");
        rotation = new JServo(mode.hardwareMap, "D.R");
        extendL = new JServo(mode.hardwareMap, "D.E1");
        extendR = new JServo(mode.hardwareMap, "D.E2");
        blockSensor = opMode.hardwareMap.get(DistanceSensor.class, "D.Tof");

        pidAutonomous.setOutputBounds(-1, 1);

        time = new ElapsedTime();

        grab.setPosition(GRAB_OPEN);
        rotation.setPosition(ROTATION_STRAIGHT);

        isSlowMode = false;

        robot = Robot.getInstance();
    }


    @Override
    public void update() {
        // TODO ADD MODES FOR AUTO PLACE AND AUTO ALIGN
//        double distanceToStone = robot.mecanumDrive.getDistanceToStone();
//        double stoneAngle = robot.mecanumDrive.angleToStone();
        liftHeight = getAbsLiftHeight() - liftBottomCal;

        setTargetHeight();

        setPlaceType();

        setLiftState();

        checkForLiftHeightReset();

        calibrateLiftBottom();


        switch (liftState) {

            case MANUAL:
                liftPower = (extendState == ExtendStates.FULL_BACK) ?
                                (-opMode.gamepad2.right_stick_y + 0.23) :
                                (-opMode.gamepad2.right_stick_y/3 + 0.23);      // TODO TEST WHY LIFTPOWER IS NEG

                extendState = (opMode.gamepad2.right_trigger > 0.1) ?
                                ExtendStates.PLACE_FAR_ROTATED :
                                (opMode.gamepad2.left_trigger > 0.1 ?
                                        ExtendStates.FULL_BACK :
                                        extendState);
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
                    targetHeight = 3 + (targetLevel * 4);
                }
                break;


            case START_AUTOLIFT:
                time.reset();
                pidAutonomous.setTargetPosition(targetHeight);
                pidAutonomous.reset();
                liftState = LiftControlStates.AUTOLIFT;
                break;


            case AUTOLIFT:
                liftPower = pidAutonomous.update(liftHeight) + 0.2;
                if (Math.abs(targetHeight - liftHeight) <= .5) {
                    liftState = LiftControlStates.HOLD;
                    pidAutonomous.reset();
                }
                break;


            case AUTOPLACE:
                if (!autoPlaceStarted) {
                    time.reset();
                    autoPlaceStarted = true;
                    extendState = (autoPlaceType == 0 ?
                                        ExtendStates.STRAIGHT_PLACE :
                                        (autoPlaceType == 1 ?
                                                ExtendStates.PLACE_FAR_ROTATED :
                                                ExtendStates.EXTEND_TO_TURN));
                    autoPlaceState = AutoPlaceStates.EXTEND;
                }

                switch (autoPlaceState) {

                    case EXTEND:
                        liftPower = 0.23;
                        if (autoPlaceType == 1 && time.seconds() > ExtendStates.EXTEND_TO_TURN.getExtendTime()) {
                            robot.stickyGamepad2.left_bumper = true;
                        }
                        if (time.seconds() > extendState.getExtendTime()) {
                            if (extendState == ExtendStates.FULL_BACK) {
                                targetHeight = 0;
                                liftState = LiftControlStates.START_AUTOLIFT;
                                isStoneGrabbed = false;
                                autoPlaceStarted = false;
                                robot.stickyGamepad2.left_bumper = false;
                                extend2 = false;
                                targetLevel++;
                                break;
                            }
                            if (!extend2 && autoPlaceType == 2) {
                                autoPlaceState = AutoPlaceStates.EXTEND;
                                extendState = ExtendStates.PLACE_CLOSE_ROTATED;
                                robot.stickyGamepad2.left_bumper = true;
                                time.reset();
                                extend2 = true;
                            } else {
                                autoPlaceState = AutoPlaceStates.LIFT;
                                liftPower = -0.3;
                                time.reset();
                            }

                        }
                        break;


                    case LIFT:
                        if (time.seconds() > LIFTTIME) {
                            if (robot.stickyGamepad2.right_bumper) {
                                autoPlaceState = AutoPlaceStates.RELEASE_BLOCK;
                                robot.stickyGamepad2.right_bumper = false;
                            } else {
                                autoPlaceState = AutoPlaceStates.EXTEND;
                                extendState = ExtendStates.FULL_BACK;
                            }
                            time.reset();
                        }
                        break;


                    case RELEASE_BLOCK:
                        if (time.seconds() > DROPTIME) {
                            autoPlaceState = AutoPlaceStates.LIFT;
                            liftPower = 0.8;
                            time.reset();
                        }
                        break;
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

        // toggle rotation of the grabbed block with left bumper on GP2
        rotation.setPosition(robot.stickyGamepad2.left_bumper ?
                                ROTATION_ROTATED :
                                ROTATION_STRAIGHT);

        robot.telemetry.addData("LIFT Target Level: ", targetLevel);
        robot.telemetry.addData("EXTEND Place Pos: ", autoPlaceType == 0 ?
                                                            "[STRAIGHT] ROT_FAR ROT_NEAR" :
                                                            (autoPlaceType == 1 ?
                                                                    "STRAIGHT [ROT_FAR] ROT_NEAR" :
                                                                    "STRAIGHT ROT_FAR [ROT_NEAR]"));
        robot.telemetry.addData("LIFT STATE", liftState);
        robot.telemetry.addData("LIFT Current Height", liftHeight);
        robot.telemetry.addData("LIFT Target Height", targetHeight);
        robot.telemetry.addData("STONESENSOR", blockSensor.getDistance(DistanceUnit.MM));
    }


/*--------------------------------------------------------------------------------------------------------------------------*/
    /* Update Methods with GPad Input */
/*--------------------------------------------------------------------------------------------------------------------------*/


    private void setTargetHeight() {
        //set target height from d pad
        if (robot.stickyGamepad2.dpad_up == tempUp) {
            tempUp = !tempUp;
            targetLevel++;
            targetLevel = Range.clip(targetLevel, 0, 14);       // make sure the target level is in the range that we can place
            targetHeight = 3.5 + (targetLevel * 4);

        } else if (robot.stickyGamepad2.dpad_down == tempDown) {
            tempDown = !tempDown;
            targetLevel--;
            //makes sure the target level is in the range that we can place
            targetLevel = Range.clip(targetLevel, 0, 14);
            targetHeight = 3 + (targetLevel * 4);
        }
    }


    private void setPlaceType() {
        if (robot.stickyGamepad2.dpad_right == tempRight) {
            tempRight = !tempRight;
            autoPlaceType++;
            autoPlaceType = Range.clip(autoPlaceType, 0, 2);
        } else if (robot.stickyGamepad2.dpad_left == tempLeft) {
            tempLeft = !tempLeft;
            autoPlaceType--;
            autoPlaceType = Range.clip(autoPlaceType, 0, 2);
        }
    }


    private void setLiftState() {
        //if a is pressed pid to target height if not gpad input
        if (opMode.gamepad1.a || opMode.gamepad2.a && liftState != LiftControlStates.AUTOLIFT) {
            liftState = LiftControlStates.START_AUTOLIFT;
        } else if (opMode.gamepad2.x || opMode.gamepad1.x) {
            liftState = LiftControlStates.AUTOPLACE;
        } else if (isStoneInBot() && !isStoneGrabbed) {
            liftState = LiftControlStates.GRAB_BLOCK;
            isStoneGrabbed = true;
            time.reset();
        } else if (Math.abs(opMode.gamepad2.right_stick_y) > 0.05) {
            liftState = LiftControlStates.MANUAL;
        } else if (liftState == LiftControlStates.MANUAL) {
            liftState = LiftControlStates.HOLD;
        }
    }


    private void checkForLiftHeightReset() {
        // if the y-button is pressed on any GP, pick the lift up to 5
        if (opMode.gamepad2.y || opMode.gamepad1.y) {
            targetHeight = 5;
            liftState = LiftControlStates.START_AUTOLIFT;
            robot.stickyGamepad2.right_bumper = false;
            isStoneGrabbed = false;
        }
    }


    private void calibrateLiftBottom() {
        if (opMode.gamepad2.left_stick_button) {
            liftBottomCal = getAbsLiftHeight();
        }
    }


    private void setExtendState() {
        // set extend state based on autoPlaceType
        ExtendStates temp = autoPlaceType == 0 ?
                ExtendStates.STRAIGHT_PLACE :
                (autoPlaceType == 1 ?
                        ExtendStates.PLACE_FAR_ROTATED :
                        ExtendStates.PLACE_CLOSE_ROTATED);
        // if GP2's right_trigger is pressed go to state based on autoPlaceType
        extendState = opMode.gamepad2.right_trigger > 0.1 ?
                temp :
                // if right_trigger isn't pressed and left_trigger is go all the way back
                (opMode.gamepad2.left_trigger > 0.1 ?
                        ExtendStates.FULL_BACK :
                        // if nothing is pressed stay in the same place
                        extendState);
    }


/*--------------------------------------------------------------------------------------------------------------------------*/
    /* Setter Methods / Methods that change the robot's physical state */
/*--------------------------------------------------------------------------------------------------------------------------*/


    public void setExtend(ExtendStates extendState) {
        switch (extendState) {
            case FULL_BACK:             // all the way back
                setExtendPos(0.45);
                break;
            case TELE_GRAB:             // position for grab
                setExtendPos(0.43);
                break;
            case PLACE_CLOSE_ROTATED:   // position to place close rotated block
                setExtendPos(0.7);
                break;
            case EXTEND_TO_TURN:        // position we need to extend to to rotate the block
                setExtendPos(0.77);
                break;
            case STRAIGHT_PLACE:        // normal straight place
                setExtendPos(0.79);
                break;
            case PLACE_FAR_ROTATED:     // furthest back rotated place
                setExtendPos(0.85);
                break;
            case GRAB_AUTO:           // distance we grab the block from in auto
                setExtendPos(0.85);
                break;
            case STRAFE_AUTO:         // where we bring the block into when we strafe in auto
                setExtendPos(0.68);
                break;
        }
    }


    public void setExtend(double inchesAbs){
        double DISTANCE_FROM_CENTER = 4.23765;
        double inchesRel = inchesAbs-DISTANCE_FROM_CENTER;
        setExtendRel(inchesRel);
    }


    public void setExtendRel(double inchesRel){
        //this is the angle at which the servo should be at in order to extend that specific number of inches
        double START_POS = .43;
        double TOTAL_RANGE = .85-START_POS;
        double TOTAL_ANGLE = 80;//TODO FIND THIS
        double START_ANGLE = 12.18;//deg

        double degAbs = Math.acos((Math.pow(LINKAGE_ARM_1,2)+Math.pow(inchesRel,2)-Math.pow(LINKAGE_ARM_2,2))/(2*inchesRel*LINKAGE_ARM_1));
        double degRel = START_ANGLE-degAbs;

        double pos = START_POS+TOTAL_RANGE*degRel/TOTAL_ANGLE;
        setExtendPos(pos);
    }


    public void setExtendPos(double pos) {
        extendL.setPosition(pos);
        extendR.setPosition(1 - pos);
    }


    public void setExtendPos(double pos, double addPos) {
        extendL.setPosition(pos);
        extendR.setPosition(1 - pos + addPos);
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


    public boolean isStoneInBot() {
        return blockSensor.getDistance(DistanceUnit.MM) < 60;
    }


    public double getAbsExtend() {
        return 0;//TODO IMP
    }


    public enum LiftControlStates {
        MANUAL, HOLD, GRAB_BLOCK, START_AUTOLIFT, AUTOLIFT, AUTOPLACE
    }


    public enum AutoPlaceStates {
        EXTEND, LIFT, RELEASE_BLOCK
    }


//TODO ADD STATES FOR ROTATION & GRAB
    public enum ExtendStates {

        FULL_BACK(0.25),
        TELE_GRAB(0.25),
        PLACE_CLOSE_ROTATED(0.4),
        EXTEND_TO_TURN(0.3),
        STRAIGHT_PLACE(0.75),
        PLACE_FAR_ROTATED(0.8),
        GRAB_AUTO(0.75),
        STRAFE_AUTO(0.1);

        private double extendTime;
        ExtendStates(double time) {
            this.extendTime = time;
        }

        public double getExtendTime() {
            return extendTime;
        }
    }
}


//Procedure for driver 2

/* From collection:
     Press Y (set target height to 1 inch above block collection)
     -block enters robot
     Pull down on Right Stick to lower the lift all of the way into the block
     Press Up/Down on DPad to set target height for deposit (as you drive under bridge to platform)
     Press Left/Right on DPad to set deposit type (Straight, Rotated-Close, or Rotated-Far)
     Press A once at platform to bring lift to lift height
     Press X to auto reach out, deposit, and contract back in
     Press Y to bring the lift back to 1 inch above block collection
     Drive back to collection and start again
*/

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