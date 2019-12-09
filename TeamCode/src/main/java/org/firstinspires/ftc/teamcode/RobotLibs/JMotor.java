package org.firstinspires.ftc.teamcode.RobotLibs;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.openftc.revextensions2.ExpansionHubMotor;

public class JMotor {
    public ExpansionHubMotor motor;
    public double cachedPower;
    public JMotor(HardwareMap hwMap, String hwName){
        motor = hwMap.get(ExpansionHubMotor.class,hwName);
    }
    public void setPower(double power){
        if (cachedPower!=power){
            motor.setPower(power);
            cachedPower = power;
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients pidfCoefficients) {
        motor.setPIDFCoefficients(runMode, pidfCoefficients);
    }
}
