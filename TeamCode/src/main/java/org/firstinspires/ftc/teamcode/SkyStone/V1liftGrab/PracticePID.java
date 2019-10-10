//package org.firstinspires.ftc.teamcode.SkyStone.V1liftGrab;
//
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@TeleOp(name = "PID")
//@Disabled
//public class PracticePID extends OpMode {
//
//    PIDFController pid = new PIDFController(new PIDCoefficients(0.0005,0,0));
//    private DcMotor rightBack;
//
//    @Override
//    public void init() {
//        pid.reset();
//        rightBack = hardwareMap.get(DcMotor.class,"RB");
//
//        pid.setOutputBounds(-1,1);
//        pid.setTargetPosition(1500);
//
//    }
//
//    @Override
//    public void loop() {
//        double newPower = pid.update(rightBack.getCurrentPosition());
//        telemetry.addData("Power", newPower);
//        telemetry.addData("Pos", rightBack.getCurrentPosition());
//        rightBack.setPower(newPower);
//
//    }
//}
//
//
//
