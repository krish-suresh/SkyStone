package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

@Config
public class DepositLift implements Subsystem {
    private static final double EXTEND_TIME = .7;
    private static final double LIFTTIME = .25;
    private static final double DROPTIME = .1;
    private final double SPOOL_DIAMETER = 1.25;
    private final double ROTATION_DEFAULT = 0.4;
    private final double ROTATION_ROTATE = 0.9;
    private final double GRAB_CLOSE = 0.27;
    private final double GRAB_OPEN = 0;

    private DcMotorEx liftMotorRight;
    private DcMotorEx liftMotorLeft;
    private Servo grab;
    private Servo rotation;
    private CRServo extension1;
    private CRServo extension2;
    private Rev2mDistanceSensor blockSensor;
    //TODO Mag sensor for bottoming out lift
    private OpMode opMode;
    private double liftHeight = 0;
    private double liftBottomCal = 0;
    private double liftPower = 0;
    private int targetLevel;
    private boolean tempDown = true;
    private boolean tempUp = true;
    private double targetHeight;
    public LiftControlStates liftState = LiftControlStates.MANUAL;
    MotionProfile liftMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(0, 0, 0),
            25,
            40,
            100
    );
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.03;
    public static double kA = 0.000;
    public PIDFController errorPID = new PIDFController(new PIDCoefficients(kP, kI, kD));
    public PIDFController pid = new PIDFController(new PIDCoefficients(0, 0, 0), kV, kA);  //idk why this isn't working gonna try and sep the FB and FF
    public PIDFController pidAutonomous = new PIDFController(new PIDCoefficients(0.05, 0, 0.0));  //TODO Calibrate PID

    StickyGamepad stickyGamepad2;
    private ElapsedTime time;
    private static final double TICKS_PER_REV = 44.4;
    private double liftStartCal;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telemetry;
    private boolean autoPlaceStarted = false;
    private double extendPower = 0;
    MotionState state = new MotionState(0, 0, 0);
    private boolean isStoneGrabbed = false;

    public DepositLift(OpMode mode) {
        opMode = mode;
        stickyGamepad2 = new StickyGamepad(mode.gamepad2);
        liftMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "L.R");
        liftMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "L.L");
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftStartCal = getRelLiftHeight();
        grab = opMode.hardwareMap.get(Servo.class, "D.G");
        rotation = opMode.hardwareMap.get(Servo.class, "D.R");
        extension1 = opMode.hardwareMap.get(CRServo.class, "D.E1");
        extension2 = opMode.hardwareMap.get(CRServo.class, "D.E2");
        blockSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "D.Tof");
        pid.setOutputBounds(-1, 1);
        errorPID.setOutputBounds(-1, 1);
        pidAutonomous.setOutputBounds(-0.5, 0.5);
        time = new ElapsedTime();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void update() {//TODO AUTO GRABBLOCK
        //TODO AUTO EXTEND DROP PLACE LIFT RETRACT
        //get lift height
        stickyGamepad2.update();
        liftHeight = getRelLiftHeight() - liftBottomCal - liftStartCal;

        //set target height from d pad
        if (stickyGamepad2.dpad_up == tempUp) {
            tempUp = !tempUp;
            targetLevel++;
            //makes sure the target level is in the range that we can place
            targetLevel = Range.clip(targetLevel, 0, 7);//target level is the number of blocks underneath
            targetHeight = 2 + (targetLevel * 4);
        } else if (stickyGamepad2.dpad_down == tempDown) {
            tempDown = !tempDown;
            targetLevel--;
            //makes sure the target level is in the range that we can place
            targetLevel = Range.clip(targetLevel, 0, 7);//target level is the number of blocks underneath
            targetHeight = 2 + (targetLevel * 4);
        }

        //if a is pressed pid to target height if not gpad input
        if (opMode.gamepad2.a && liftState != LiftControlStates.AUTOLIFT) {
            liftState = LiftControlStates.STARTAUTOLIFT;
        } else if (opMode.gamepad2.x) {
            liftState = LiftControlStates.AUTOPLACE;
        } else if (isStoneInBot() && !isStoneGrabbed) {
            liftState = LiftControlStates.GRABBLOCK;
            isStoneGrabbed = true;
            time.reset();
        } else if (Math.abs(opMode.gamepad2.right_stick_y) > 0.05) {
            liftState = LiftControlStates.MANUAL;
        } else if (liftState == LiftControlStates.MANUAL) {
            liftState = LiftControlStates.HOLD;
        }
        if (opMode.gamepad2.y) {
            targetHeight = 1.5;
        }
        if (opMode.gamepad2.right_stick_button) {
            liftBottomCal = getRelLiftHeight();
        }
        switch (liftState) {
            case MANUAL:
                liftPower = -Math.pow(opMode.gamepad2.right_stick_y, 3) + 0.2;//TODO TEST WHY LIFTPOWER IS NEG

                break;
            case HOLD:
                liftPower = 0.2;
                break;
            case GRABBLOCK:
                double secondsGB = time.seconds();
                if (secondsGB < LIFTTIME) {

                    liftPower = -0.2;
                } else if (secondsGB < LIFTTIME + DROPTIME) {
                    liftPower = 0;
                    stickyGamepad2.right_bumper = true;
                } else {
                    liftState = LiftControlStates.HOLD;
                }
                break;
            case STARTAUTOLIFT:
                time.reset();
                setLiftMotionProfile(targetHeight);
                liftState = LiftControlStates.AUTOLIFT;
                break;
            case AUTOLIFT:

                state = liftMotionProfile.get(time.seconds());

                liftPower = pid.update(liftHeight, state.getV(), state.getA()) + 0.2+errorPID.update(liftHeight);//the reason there is two controlers is cus the error correction part of the PIDF controler was not working correctly -\_(:))_/-
                if (liftMotionProfile.duration() <= time.seconds()) {
                    liftState = LiftControlStates.HOLD;
                }
                break;

            case AUTOPLACE:     //TODO pls add what each of these phases does
                double seconds = time.seconds();
                if (!autoPlaceStarted) {
                    time.reset();
                    autoPlaceStarted = true;
                } else if (seconds < EXTEND_TIME) {
                    extendPower = 1;
                } else if (seconds < EXTEND_TIME + LIFTTIME) {
                    extendPower = 0;
                    liftPower = -0.2;
                } else if (seconds < EXTEND_TIME + LIFTTIME + DROPTIME) {
                    liftPower = 0.2;
                    stickyGamepad2.right_bumper = false;
                } else if (seconds < EXTEND_TIME + LIFTTIME * 2 + DROPTIME) {
                    liftPower = 0.6;
                } else if (seconds < EXTEND_TIME * 2 + LIFTTIME * 2 + DROPTIME) {
                    liftPower = 0.2;
                    extendPower = -1;
                } else {
                    extendPower = 0;
                    targetHeight = 0;
                    liftState = LiftControlStates.STARTAUTOLIFT;
                    isStoneGrabbed = false;
                    autoPlaceStarted = false;
                }
                break;
        }
        updateLiftPower((liftHeight < 0) ? Range.clip(liftPower, 0, 1) : liftPower);
        grab.setPosition(stickyGamepad2.right_bumper ? GRAB_CLOSE : GRAB_OPEN);
        setExtensionPower(Range.clip(opMode.gamepad2.right_trigger - opMode.gamepad2.left_trigger + extendPower, -1, 1));
        rotation.setPosition(stickyGamepad2.left_bumper ? ROTATION_DEFAULT : ROTATION_ROTATE);
        telemetry.addData("LIFT POWER", liftPower);
        telemetry.addData("LIFT STATE", liftState);
        telemetry.addData("LIFT Current Height", liftHeight);
        telemetry.addData("LIFT Target Level", targetLevel);
        telemetry.addData("LIFT X", state.getX());
        telemetry.addData("LIFT V", state.getV());
        telemetry.addData("LIFT A", state.getA());
        telemetry.addData("AUTOLIFT ERROR", state.getX() - liftHeight);
        dashboard.getTelemetry().update();
    }

    public void setExtensionPower(double power) {
        extension1.setPower(power / 2);
        extension2.setPower(power / 2);
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

    public boolean isStoneInBot() {
        return blockSensor.getDistance(DistanceUnit.MM) < 50;
    }

    public void setLiftMotionProfile(double start, double end) {
        liftMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(start, 0, 0),
                new MotionState(end, 0, 0),
                40,
                50,
                100
        );
    }

    public void setLiftMotionProfile(double end) {
        setLiftMotionProfile(liftHeight, end);
    }

    public void setTargetHeight(double height) {
        targetHeight = height;
    }

    public enum LiftControlStates {
        MANUAL, HOLD, GRABBLOCK, STARTAUTOLIFT, AUTOLIFT, AUTOPLACE
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