package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotLibs.JMotor;
import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

@Config
public class DepositLift implements Subsystem {

    private final double LIFTTIME = .35;
    private final double DROPTIME = .15;
    private double WAITTIME = 0.5;
    public static double MAX_LIFT_VEL = 50;
    public static double MAX_LIFT_ACCEL = 80;
    public static double MAX_LIFT_JERK = 0;

    private final double SPOOL_DIAMETER = 1.25;
    private final double ROTATION_DEFAULT = 0.4;
    private final double ROTATION_ROTATE = 0.9;
    private final double GRAB_CLOSE = 0.27;
    private final double GRAB_OPEN = 0;

    private JMotor liftMotorRight;
    private JMotor liftMotorLeft;
    private Servo grab;
    private Servo rotation;
    public Servo extendL;
    public Servo extendR;
    private Rev2mDistanceSensor blockSensor;

    //TODO Mag sensor for bottoming out lift
    private OpMode opMode;
    private double liftHeight = 0;
    private double liftBottomCal = 0;
    private double liftPower = 0;
    private int targetLevel;
    private boolean tempDown = true;
    private boolean tempUp = true;
    private boolean tempRight = true;
    private boolean tempLeft = true;
    private double targetHeight;
    public LiftControlStates liftState = LiftControlStates.MANUAL;
    public static double kP = 0.15;
    public static double kI = 0.01;
    public static double kD = 0.008;
    public PIDFController pidAutonomous = new PIDFController(new PIDCoefficients(kP, kI, kD));

    StickyGamepad stickyGamepad2;
    private ElapsedTime time;
    private static final double TICKS_PER_REV = 44.4;
    public double liftStartCal;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telemetry;
    private boolean autoPlaceStarted = false;
    private boolean isStoneGrabbed = false;
    private ExtendStates extendState = ExtendStates.RETRACTED;
    private AutoPlaceStates autoPlaceState;
    private int autoPlaceType = 0;
    private boolean extend2;


    public DepositLift(OpMode mode) {
        opMode = mode;
        stickyGamepad2 = new StickyGamepad(mode.gamepad2);
        liftMotorRight = new JMotor(mode.hardwareMap,"L.R");
        liftMotorLeft = new JMotor(mode.hardwareMap,"L.L");
        liftMotorRight.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftStartCal = getRelLiftHeight();
        grab = opMode.hardwareMap.get(Servo.class, "D.G");
        rotation = opMode.hardwareMap.get(Servo.class, "D.R");
        extendL = opMode.hardwareMap.get(Servo.class, "D.E1");
        extendR = opMode.hardwareMap.get(Servo.class, "D.E2");
        blockSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "D.Tof");
//        pid.setOutputBounds(-1, 1);
        pidAutonomous.setOutputBounds(-1, 1);
        time = new ElapsedTime();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
        grab.setPosition(GRAB_OPEN);
        rotation.setPosition(ROTATION_ROTATE);
    }

    @Override
    public void update() {

        stickyGamepad2.update();
        liftHeight = getAbsLiftHeight() - liftBottomCal;

        //set target height from d pad
        if (stickyGamepad2.dpad_up == tempUp) {
            tempUp = !tempUp;
            targetLevel++;
            //makes sure the target level is in the range that we can place
            targetLevel = Range.clip(targetLevel, 0, 8);
            targetHeight = 2 + (targetLevel * 4);
        } else if (stickyGamepad2.dpad_down == tempDown) {
            tempDown = !tempDown;
            targetLevel--;
            //makes sure the target level is in the range that we can place
            targetLevel = Range.clip(targetLevel, 0, 8);
            targetHeight = 3 + (targetLevel * 4);
        }
        if (stickyGamepad2.dpad_right == tempRight) {
            tempRight = !tempRight;
            autoPlaceType++;
            autoPlaceType = Range.clip(targetLevel, 0, 2);
        } else if (stickyGamepad2.dpad_left == tempLeft) {
            tempLeft = !tempLeft;
            autoPlaceType--;
            autoPlaceType = Range.clip(targetLevel, 0, 2);
        }
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
        if (opMode.gamepad2.y || opMode.gamepad1.y) {
            targetHeight = 5;
            liftState = LiftControlStates.START_AUTOLIFT;
            isStoneGrabbed = false;
        }
        if (opMode.gamepad2.left_stick_button) {
            liftBottomCal = getRelLiftHeight();
        }
        switch (liftState) {
            case MANUAL:
                liftPower = -opMode.gamepad2.right_stick_y + 0.23;//TODO TEST WHY LIFTPOWER IS NEG
                extendState = opMode.gamepad2.right_trigger > 0.1 ? ExtendStates.EXTEND_TURN_1 : (opMode.gamepad2.left_trigger > 0.1 ? ExtendStates.RETRACTED : extendState);
                break;
            case HOLD:
                liftPower = 0.23;
                extendState = opMode.gamepad2.right_trigger > 0.1 ? ExtendStates.EXTEND_TURN_1 : (opMode.gamepad2.left_trigger > 0.1 ? ExtendStates.RETRACTED : extendState);
                break;
            case GRAB_BLOCK:
                double secondsGB = time.seconds();
                if (secondsGB < WAITTIME) {
                } else if (secondsGB < WAITTIME + LIFTTIME) {

                    liftPower = -0.6;
                } else if (secondsGB < WAITTIME + LIFTTIME + DROPTIME) {
                    liftPower = 0;
                    stickyGamepad2.right_bumper = true;
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
                double seconds = time.seconds();
                if (!autoPlaceStarted) {
                    time.reset();
                    autoPlaceStarted = true;
                    extendState = (autoPlaceType == 0 ? ExtendStates.EXTEND_0 : (autoPlaceType == 1 ? ExtendStates.EXTEND_TURN_1 : ExtendStates.EXTEND_TURN));
                    autoPlaceState = AutoPlaceStates.EXTEND;
                }
                switch (autoPlaceState) {
                    case EXTEND:
                        liftPower = 0.23;
                        if (autoPlaceType == 1 && seconds > ExtendStates.EXTEND_TURN.getTime()) {
                            stickyGamepad2.left_bumper = true;
                        }
                        if (seconds > extendState.getTime()) {
                            if (extendState == ExtendStates.RETRACTED) {
                                targetHeight = 0;
                                liftState = LiftControlStates.START_AUTOLIFT;
                                isStoneGrabbed = false;
                                autoPlaceStarted = false;
                                stickyGamepad2.left_bumper = false;
                                break;
                            }
                            if (!extend2 && autoPlaceType == 2) {
                                autoPlaceState = AutoPlaceStates.EXTEND;
                                extendState = ExtendStates.EXTEND_TURN_2;
                                stickyGamepad2.left_bumper = true;
                                time.reset();
                                extend2 = true;
                            } else {
                                autoPlaceState = AutoPlaceStates.LIFT;
                                liftPower = -0.2;
                                time.reset();
                            }

                        }
                        break;
                    case LIFT:
                        if (seconds > LIFTTIME) {
                            if (stickyGamepad2.right_bumper) {
                                autoPlaceState = AutoPlaceStates.RELEASE_BLOCK;
                                stickyGamepad2.right_bumper = false;
                            } else {
                                autoPlaceState = AutoPlaceStates.EXTEND;
                                extendState = ExtendStates.RETRACTED;
                            }
                        }
                        break;
                    case RELEASE_BLOCK:
                        if (seconds > DROPTIME) {
                            autoPlaceState = AutoPlaceStates.LIFT;
                            liftPower = 0.6;
                        }
                        break;
                }

                break;
        }
        setExtend(extendState);
        updateLiftPower((liftHeight < 0) ? Range.clip(liftPower, 0, 1) : liftPower);
        grab.setPosition(stickyGamepad2.right_bumper ? GRAB_CLOSE : GRAB_OPEN);
        rotation.setPosition(stickyGamepad2.left_bumper ? ROTATION_DEFAULT : ROTATION_ROTATE);
//        telemetry.addData("EXTEND", extendState);
//        telemetry.addData("LIFT POWER", liftPower);
        telemetry.addData("LIFT STATE", liftState);
//        telemetry.addData("LIFT Current Height", liftHeight);
        telemetry.addData("LIFT Target Level: ", targetLevel);
        telemetry.addData("EXTEND Place Pos: ",autoPlaceType==0?"[STRAIGHT] ROT_FAR ROT_NEAR":(autoPlaceType==1?"STRAIGHT [ROT_FAR] ROT_NEAR":"STRAIGHT ROT_FAR [ROT_NEAR]"));
//        telemetry.addData("LIFT Target Height", targetHeight);
        dashboard.getTelemetry().update();
    }

    private void setExtend(ExtendStates extendState) {
        //TODO FIX ALL THESE VALUES
        switch (extendState) {
            case RETRACTED:
                setExtendPos(0.4);
                break;
            case EXTEND_TURN_2:
                setExtendPos(0.5);
                break;
            case EXTEND_TURN:
                setExtendPos(0.6);
                break;
            case EXTEND_0:
                setExtendPos(0.77);
                break;
            case EXTEND_TURN_1:
                setExtendPos(0.85);
                break;
        }
    }

    public void setExtendPos(double pos) {
        extendL.setPosition(pos);
        extendR.setPosition(1 - pos);
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

    public double getRelLiftHeight() {

        return Math.PI * (SPOOL_DIAMETER / 2) * (liftMotorRight.getCurrentPosition() / TICKS_PER_REV);
    }

    public double getAbsLiftHeight() {

        return Math.PI * (SPOOL_DIAMETER / 2) * (liftMotorRight.getCurrentPosition() / TICKS_PER_REV) - liftStartCal;
    }

    public boolean isStoneInBot() {
        return blockSensor.getDistance(DistanceUnit.MM) < 50;
    }

    public void setTargetHeight(double height) {
        pidAutonomous.setTargetPosition(height);
    }

    public enum LiftControlStates {
        MANUAL, HOLD, GRAB_BLOCK, START_AUTOLIFT, AUTOLIFT, AUTOPLACE
    }

    public enum AutoPlaceStates {
        EXTEND, LIFT, RELEASE_BLOCK
    }

    public enum ExtendStates {

        RETRACTED(0.3), EXTEND_TURN_2(0.1), EXTEND_TURN(0.2), EXTEND_0(0.3), EXTEND_TURN_1(0.5);
        private double time;

        ExtendStates(double time) {
            this.time = time;
        }

        public double getTime() {
            return time;
        }
    }
}


//Procedure for driver 2

/*From collection:
    Press B (set target height to 1 inch above block collection)
    Press A (Move lift to target height(in this case 1 in))
    -block enters robot
    -auto detect block and go down and grab block
    Press Up/Down on DPad to set target height for deposit (as you drive under bridge to platform)
    Press A once at platform to bring lift to lift height
    Press X to auto reach out, deposit, and contract back in
    Press Y to set targetHeight back to 0
    Press A to go to target height (0)
    Drive back to collection and start again
*/