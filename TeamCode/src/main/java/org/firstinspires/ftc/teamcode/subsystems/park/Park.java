package org.firstinspires.ftc.teamcode.subsystems.park;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Park {
    public Servo parkServo;

    public Park(HardwareMap hwMap) {
        parkServo = hwMap.get(Servo.class, "park");
    }

    public void tilt() {
        parkServo.setPosition(.6);
    }

    public void unTilt() {
        parkServo.setPosition(0);
    }

}
