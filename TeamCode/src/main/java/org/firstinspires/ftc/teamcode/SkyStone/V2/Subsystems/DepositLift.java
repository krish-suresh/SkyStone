package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

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

public class DepositLift implements Subsystem {
    private final double SPOOL_DIAMETER = 1.25;
    private final double ROTATION_DEFAULT = 0.35;
    private final double ROTATION_ROTATE = 0.8;
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
    public PIDFController pid = new PIDFController(new PIDCoefficients(0.02, 0, 0.001),1,1);  //TODO Calibrate PID
    StickyGamepad stickyGamepad2;
    private ElapsedTime time;
    private PIDFController holdPID = new PIDFController(new PIDCoefficients(0.02, 0, 0));
    private boolean holdStarted = false;

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
        blockSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class,"D.Tof");
        pid.reset();
        pid.setOutputBounds(-1, 1);
        holdPID.reset();
        holdPID.setOutputBounds(-1, 1);
        time = new ElapsedTime();

    }

    @Override
    public void update() {
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
        if (stickyGamepad2.a&&liftStates!=LiftControlStates.AUTOPLACE) {
            liftStates = LiftControlStates.STARTAUTOPLACE;
        } else if (Math.abs(opMode.gamepad2.right_stick_y)>0.1){
            liftStates = LiftControlStates.MANUAL;
            holdStarted = false;
        } else if (!holdStarted){
            liftStates = LiftControlStates.STARTHOLD;
            holdStarted = true;
        } else {
            liftStates = LiftControlStates.HOLD;

        }
        switch (liftStates) {
            case MANUAL:
                liftPower = -Math.pow(opMode.gamepad2.right_stick_y, 3);//TODO TEST WHY LIFTPOWER IS NEG
                break;
            case STARTHOLD:
                holdPID.setTargetPosition(liftHeight);
                liftStates = LiftControlStates.HOLD;
                break;
            case HOLD:
                liftPower = holdPID.update(liftHeight);
                break;
            case STARTAUTOPLACE:
                time.reset();
                setLiftMotionProfile(targetHeight);
                liftStates = LiftControlStates.AUTOPLACE;
                break;
            case AUTOPLACE:
                MotionState state = liftMotionProfile.get(time.milliseconds());
                liftPower = pid.update(liftHeight, state.getV(), state.getA());
                if (liftMotionProfile.duration() <= time.milliseconds()) {
                    liftStates = LiftControlStates.STARTHOLD;
                    stickyGamepad2.a = false;
                }
                break;

        }


        //calibrate the bottom pos
        if (opMode.gamepad2.x) {
            liftBottomCal = getRelLiftHeight();
        }

        opMode.telemetry.addData("LIFT POWER", liftPower);
        opMode.telemetry.addData("LIFT STATE",liftStates);
        updateLiftPower(liftPower);
        grab.setPosition(stickyGamepad2.right_bumper ? GRAB_CLOSE : GRAB_OPEN);
        setExtensionPower(opMode.gamepad2.right_trigger - opMode.gamepad2.left_trigger);
        rotation.setPosition(stickyGamepad2.left_bumper ? ROTATION_DEFAULT : ROTATION_ROTATE);
        opMode.telemetry.addData("DEPOSIT Current Height", liftHeight);
        opMode.telemetry.addData("DEPOSIT Target Level", targetLevel);
        opMode.telemetry.addData("DEPOSIT Block In Bot?", isStoneInBot());
    }

    public void setExtensionPower(double power) {
        extension1.setPower(-power / 2);
        extension2.setPower(-power / 2);
    }
    public void grabStone(){
        grab.setPosition(GRAB_CLOSE);

    }
    public void releaseStone(){
        grab.setPosition(GRAB_OPEN);
    }
    public void updateLiftPower(double power) {
        liftMotorLeft.setPower(power);
        liftMotorRight.setPower(power);
    }

    public double getRelLiftHeight() {
        return Math.PI*(SPOOL_DIAMETER/2) * liftMotorRight.getCurrentPosition() / (3.7 * 28);
    }

    public boolean isStoneInBot() {
        return blockSensor.getDistance(DistanceUnit.MM)<50;
    }

    public void setLiftMotionProfile(double start, double end) {
        liftMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(start, 0, 0),
                new MotionState(end, 0, 0),
                25,
                40,
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
