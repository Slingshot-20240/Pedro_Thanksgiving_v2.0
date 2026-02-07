package org.firstinspires.ftc.teamcode.teleop.servoTests;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.park.Park;

@Disabled
@TeleOp
@Config
public class ParkServoTest extends OpMode {

    Park park;

    public static double position = 0.5;

    @Override
    public void init() {
        park = new Park(hardwareMap);
    }

    @Override
    public void loop() {
        park.parkServo.setPosition(position);
    }


}
