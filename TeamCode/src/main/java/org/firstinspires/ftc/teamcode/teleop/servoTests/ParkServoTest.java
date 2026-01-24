package org.firstinspires.ftc.teamcode.teleop.servoTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.park.Park;

public class ParkServoTest extends OpMode {

    Park park;

    public static double position = 0;

    @Override
    public void init() {
        park = new Park(hardwareMap);
    }

    @Override
    public void loop() {
        park.parkServo.setPosition(position);
    }
}
