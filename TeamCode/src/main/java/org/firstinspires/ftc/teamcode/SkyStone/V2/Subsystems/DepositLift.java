package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.hardware.motors.NeveRest3_7GearmotorV1;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class DepositLift implements Subsystem {
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
    public LiftControlStates liftStates = LiftControlStates.MANUAL;
    MotionProfile liftMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(0, 0, 0),
            25,
            40,
            100
    );
    ;
    public PIDFController pid = new PIDFController(new PIDCoefficients(0.005, 0, 0.0), 0.05, 0.0005);  //TODO Calibrate PID
    public PIDFController pidAutonomous = new PIDFController(new PIDCoefficients(0.05, 0, 0.0));  //TODO Calibrate PID

    StickyGamepad stickyGamepad2;
    private ElapsedTime time;
    private PIDFController holdPID = new PIDFController(new PIDCoefficients(0.2, 0, 0.0));
    private boolean holdStarted = false;
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest3_7GearmotorV1.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public DepositLift(OpMode mode) {
        opMode = mode;
        stickyGamepad2 = new StickyGamepad(mode.gamepad2);
        liftMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "L.R");
        liftMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "L.L");
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        grab = opMode.hardwareMap.get(Servo.class, "D.G");
        rotation = opMode.hardwareMap.get(Servo.class, "D.R");
        extension1 = opMode.hardwareMap.get(CRServo.class, "D.E1");
        extension2 = opMode.hardwareMap.get(CRServo.class, "D.E2");
        blockSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "D.Tof");
        pid.reset();
        pid.setOutputBounds(-1, 1);
        holdPID.reset();
        holdPID.setOutputBounds(-1, 1);
        pidAutonomous.setOutputBounds(-0.5, 0.5);
        time = new ElapsedTime();

    }

    @Override
    public void update() {//TODO AUTO GRABBLOCK
        //TODO AUTO EXTEND DROP PLACE LIFT RETRACT
        //get lift height
        stickyGamepad2.update();
        liftHeight = getRelLiftHeight() - liftBottomCal;

        //set target height from d pad
        if (stickyGamepad2.dpad_up == tempUp) {
            tempUp = !tempUp;
            targetLevel++;
        } else if (stickyGamepad2.dpad_down == tempDown) {
            tempDown = !tempDown;
            targetLevel--;
        }
        //makes sure the target level is in the range that we can place
        targetLevel = Range.clip(targetLevel, 0, 7);//target level is the number of blocks underneath
        targetHeight = 2 + (targetLevel * 4);
        //if a is pressed pid to target height if not gpad input
        if (opMode.gamepad2.a && liftStates != LiftControlStates.AUTOPLACE) {
            liftStates = LiftControlStates.STARTAUTOPLACE;
        } else if (Math.abs(opMode.gamepad2.right_stick_y) > 0.05) {
            liftStates = LiftControlStates.MANUAL;
            holdStarted = false;
        } else if (!holdStarted && liftStates != LiftControlStates.AUTOPLACE) {
            liftStates = LiftControlStates.STARTHOLD;
            holdStarted = true;
        } else if (liftStates != LiftControlStates.AUTOPLACE) {
            liftStates = LiftControlStates.HOLD;
        }
        if (opMode.gamepad2.y) {
            targetLevel = 0;
        }
        if (opMode.gamepad2.x) {
            liftBottomCal = getRelLiftHeight();
            liftStates = LiftControlStates.STARTHOLD;
            holdStarted = true;
        }
        switch (liftStates) {
            case MANUAL:
                liftPower = -Math.pow(opMode.gamepad2.right_stick_y, 3);//TODO TEST WHY LIFTPOWER IS NEG
//                if (liftHeight < 0) {
//                    liftPower = Math.abs(liftPower);
//                }

                break;
            case STARTHOLD:
                holdPID.setTargetPosition(liftHeight);
                liftStates = LiftControlStates.HOLD;
                break;
            case HOLD:
                liftPower = 0.2;
                break;
            case STARTAUTOPLACE:
                time.reset();
                setLiftMotionProfile(targetHeight);
                liftStates = LiftControlStates.AUTOPLACE;
                break;
            case AUTOPLACE:
                MotionState state = liftMotionProfile.get(time.seconds());
                opMode.telemetry.addData("LIFT X", state.getX());
                opMode.telemetry.addData("LIFT V", state.getV());
                opMode.telemetry.addData("LIFT A", state.getA());
//                opMode.telemetry.addData("AUTOLIFT POWER",pid.update(state.getX()-liftHeight, state.getV(), state.getA()));
                opMode.telemetry.addData("AUTOLIFT ERROR", state.getX() - liftHeight);
                liftPower = pid.update(liftHeight, state.getV(), state.getA());
                if (liftMotionProfile.duration() <= time.seconds()) {
                    liftStates = LiftControlStates.STARTHOLD;
                    holdStarted = false;

                }
                break;

        }

        //calibrate the bottom pos


        opMode.telemetry.addData("LIFT POWER", liftPower);
        opMode.telemetry.addData("LIFT STATE", liftStates);
        updateLiftPower(liftPower);
        grab.setPosition(stickyGamepad2.right_bumper ? GRAB_CLOSE : GRAB_OPEN);
        setExtensionPower(opMode.gamepad2.right_trigger - opMode.gamepad2.left_trigger);
        rotation.setPosition(stickyGamepad2.left_bumper ? ROTATION_DEFAULT : ROTATION_ROTATE);
        opMode.telemetry.addData("DEPOSIT Current Height", liftHeight);
        opMode.telemetry.addData("DEPOSIT Target Level", targetLevel);
        opMode.telemetry.addData("DEPOSIT HoldPos", targetHeight);


    }

    public void setExtensionPower(double power) {
        extension1.setPower(-power / 2);
        extension2.setPower(-power / 2);
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
        MANUAL, HOLD, STARTAUTOPLACE, AUTOPLACE, STARTHOLD;
    }
}
