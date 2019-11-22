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
    private final double EXTEND_TIME = .7;
    private final double LIFTTIME = .25;
    private final double DROPTIME = .15;
    private double WAITTIME = 0.5;
    public static  double MAX_LIFT_VEL = 50;
    public static  double MAX_LIFT_ACCEL = 80;
    public static  double MAX_LIFT_JERK = 0;

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
//    MotionProfile liftMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
//            new MotionState(0, 0, 0),
//            new MotionState(0, 0, 0),
//            25,
//            40,
//            100
//    );
    public static double kP = 0.15;
    public static double kI = 0.01;
    public static double kD = 0.008;
//    public static double kV = 0.03;
//    public static double kA = 0.000;
//    public PIDFController pid = new PIDFController(new PIDCoefficients(kP, kI, kD), kV, kA);  //idk why this isn't working gonna try and sep the FB and FF
    public PIDFController pidAutonomous = new PIDFController(new PIDCoefficients(kP, kI, kD));

    StickyGamepad stickyGamepad2;
    StickyGamepad stickyGamepad1;

    private ElapsedTime time;
    private static final double TICKS_PER_REV = 44.4;
    public double liftStartCal;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telemetry;
    private boolean autoPlaceStarted = false;
    private double extendPower = 0;
//    MotionState state = new MotionState(0, 0, 0);
    private boolean isStoneGrabbed = false;


    public DepositLift(OpMode mode) {
        opMode = mode;
        stickyGamepad2 = new StickyGamepad(mode.gamepad2);
        stickyGamepad1 = new StickyGamepad(mode.gamepad1);
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
//        pid.setOutputBounds(-1, 1);
        pidAutonomous.setOutputBounds(-1, 1);
        time = new ElapsedTime();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
        grab.setPosition(GRAB_OPEN);
        rotation.setPosition(ROTATION_ROTATE);
    }

    @Override
    public void update() {
        //get lift height

        stickyGamepad2.update();
        stickyGamepad1.update();
        liftHeight = getAbsLiftHeight() - liftBottomCal;

        //set target height from d pad
        if (stickyGamepad1.dpad_up == tempUp) {
            tempUp = !tempUp;
            targetLevel++;
            //makes sure the target level is in the range that we can place
            targetLevel = Range.clip(targetLevel, 0, 8);//target level is the number of blocks underneath
            targetHeight = 2 + (targetLevel * 4);
        } else if (stickyGamepad1.dpad_down == tempDown) {
            tempDown = !tempDown;
            targetLevel--;
            //makes sure the target level is in the range that we can place
            targetLevel = Range.clip(targetLevel, 0, 8);//target level is the number of blocks underneath
            targetHeight = 2 + (targetLevel * 4);
        }

        //if a is pressed pid to target height if not gpad input
        if (opMode.gamepad1.a||opMode.gamepad2.a && liftState != LiftControlStates.AUTOLIFT) {
            liftState = LiftControlStates.STARTAUTOLIFT;
        } else if (opMode.gamepad2.x||opMode.gamepad1.x) {
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
        if (opMode.gamepad2.y||opMode.gamepad1.y) {
            targetHeight = 3;
            liftState = LiftControlStates.STARTAUTOLIFT;
        }
        if (opMode.gamepad2.left_stick_button) {
            liftBottomCal = getRelLiftHeight();
        }
        switch (liftState) {
            case MANUAL:
                liftPower = (opMode.gamepad2.right_stick_button?0.5:1)*-Math.pow(opMode.gamepad2.right_stick_y, 3) + 0.2;//TODO TEST WHY LIFTPOWER IS NEG

                break;
            case HOLD:
                liftPower = 0.2;
                break;
            case GRABBLOCK:
                double secondsGB = time.seconds();
                if(secondsGB<WAITTIME){}
                else if (secondsGB < WAITTIME+LIFTTIME) {

                    liftPower = -0.4;
                } else if (secondsGB < WAITTIME+LIFTTIME + DROPTIME) {
                    liftPower = 0;
                    stickyGamepad2.right_bumper = true;
                } else {
                    liftState = LiftControlStates.HOLD;
                }
                break;
            case STARTAUTOLIFT:
                time.reset();
                pidAutonomous.setTargetPosition(targetHeight);
                pidAutonomous.reset();
                liftState = LiftControlStates.AUTOLIFT;
                break;
            case AUTOLIFT:

//                state = liftMotionProfile.get(time.seconds());
//                liftPower = pid.update(liftHeight, state.getV(), state.getA()) + 0.2;
                liftPower = pidAutonomous.update(liftHeight)+0.2;

                if (Math.abs(targetHeight-liftHeight) <= .5) {
                    liftState = LiftControlStates.HOLD;
                    pidAutonomous.reset();
                }
                break;

            case AUTOPLACE:
                double seconds = time.seconds();
                if (!autoPlaceStarted) {
                    time.reset();
                    autoPlaceStarted = true;
                } else if (seconds < EXTEND_TIME) {
                    extendPower = 1;
                } else if (seconds < EXTEND_TIME + LIFTTIME) {
                    extendPower = 0.75;
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
                    extendPower = -0.3;
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
        telemetry.addData("LIFT Target Height", targetHeight);
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
    public double getAbsLiftHeight() {

        return Math.PI * (SPOOL_DIAMETER / 2) * (liftMotorRight.getCurrentPosition() / TICKS_PER_REV)-liftStartCal;
    }
    public boolean isStoneInBot() {
        return blockSensor.getDistance(DistanceUnit.MM) < 50;
    }

//    public void setLiftMotionProfile(double start, double end) {
//        liftMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(start, 0, 0),
//                new MotionState(end, 0, 0),
//                MAX_LIFT_VEL,
//                MAX_LIFT_ACCEL,
//                MAX_LIFT_JERK
//        );
//    }
//
//    public void setLiftMotionProfile(double end) {
//        setLiftMotionProfile(liftHeight, end);
//    }

    public void setTargetHeight(double height) {
        pidAutonomous.setTargetPosition(height);
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