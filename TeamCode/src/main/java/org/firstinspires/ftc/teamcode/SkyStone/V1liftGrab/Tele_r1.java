//package org.firstinspires.ftc.teamcode.SkyStone.V1liftGrab;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "tele_r1", group = "r1")
//@Disabled
//public class Tele_r1 extends Config_r1 {
//
//
//    @Override
//    public void init() {
//        config(this);
//    }
//
//    @Override
//    public void loop() {
//        robot.drive.mecanum.updateMecanum(gamepad1,1);
//        lift.liftPower(gamepad1.right_stick_y);
//        if (gamepad1.right_bumper){
//            lift.closeGrab();
//        } else if (gamepad1.left_bumper){
//            lift.openGrab();
//        }
//    }
//}
//
