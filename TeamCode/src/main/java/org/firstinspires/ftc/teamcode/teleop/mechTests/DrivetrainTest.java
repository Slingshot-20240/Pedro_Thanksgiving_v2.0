package org.firstinspires.ftc.teamcode.teleop.mechTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;

@Disabled
@TeleOp
public class DrivetrainTest extends OpMode {
    GamepadMapping controls;
    Robot robot;
    Drivetrain dt;
    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        dt = robot.drivetrain;
    }

    @Override
    public void loop() {
        controls.update();
        dt.update();
    }
}
