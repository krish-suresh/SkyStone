package org.firstinspires.ftc.teamcode.SkyStone.RobotLibs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class JServo {
    public ServoImplEx servo;
    private double cachedPosition=-1;

    public JServo(HardwareMap hwMap, String hwName) {
        servo = hwMap.get(ServoImplEx.class, hwName);
    }

    public void setPosition(double position) {
        if (position != cachedPosition) {
            servo.setPosition(position);
            cachedPosition = position;
        }
    }
    public void off(){
        servo.setPwmDisable();
    }
}
