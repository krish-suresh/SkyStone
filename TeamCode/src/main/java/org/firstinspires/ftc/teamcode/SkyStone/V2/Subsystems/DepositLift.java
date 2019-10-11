package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;

public class DepositLift implements Subsystem {
    private final double ROTATION_DEFAULT = 0.9;
    private final double ROTATION_ROTATE = 0.35;
    public DcMotorEx liftMotor;
    public Servo grab;
    public Servo rotation;
    public CRServo extention;
    //TODO auto encoder heights for lift
    //TODO Mag sensor for bottoming out lift
    public OpMode opMode;
    public double extensionPower = 0;

    public int liftHeight = 0;
    public final double GRAB_CLOSE = 0.2;
    public final double GRAB_OPEN = 0;
    public int liftBottomCal = 0;
    PIDFController pid = new PIDFController(new PIDCoefficients(0.0005, 0, 0));

    public DepositLift(OpMode mode) {
        opMode = mode;
        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "L");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        grab = opMode.hardwareMap.get(Servo.class, "D.G");
        rotation = opMode.hardwareMap.get(Servo.class, "D.R");
        extention = opMode.hardwareMap.get(CRServo.class, "D.E");
        pid.reset();
        pid.setOutputBounds(-1, 1);
        pid.setTargetPosition(0);
    }
//https://github.com/acmerobotics/relic-recovery/blob/master/TeamCode/src/main/java/com/acmerobotics/relicrecovery/opmodes/StickyGamepad.java
    @Override
    public void update() {
//        liftHeight = liftMotor.getCurrentPosition() - liftBottomCal;
//        if (opMode.gamepad2.a) {
//            liftBottomCal = liftMotor.getCurrentPosition();
//        }
//        if (!(liftHeight < 100)) {
            liftMotor.setPower((opMode.gamepad2.dpad_up ? 1 : (opMode.gamepad2.dpad_down ? -0.4 : 0) + 0.2));
//        } else {
//            liftMotor.setPower((opMode.gamepad2.dpad_up ? 1 : 0));
//        }


        if (opMode.gamepad2.right_bumper) grab.setPosition(GRAB_CLOSE);
        else if (opMode.gamepad2.left_bumper) grab.setPosition(GRAB_OPEN);


        extention.setPower((opMode.gamepad2.right_trigger-opMode.gamepad2.left_trigger)/2);

        if (opMode.gamepad2.dpad_right) {
            rotation.setPosition(ROTATION_DEFAULT);
        } else if (opMode.gamepad2.dpad_left) {
            rotation.setPosition(ROTATION_ROTATE);
        }
    }

}
