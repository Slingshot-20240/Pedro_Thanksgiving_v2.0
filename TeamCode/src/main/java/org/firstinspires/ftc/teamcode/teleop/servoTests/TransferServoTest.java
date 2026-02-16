package org.firstinspires.ftc.teamcode.teleop.servoTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;

@Disabled
@Config
@TeleOp(group = "transfer servo tests")
public class TransferServoTest extends OpMode {
    Robot robot;
    public static double servoPosF = 0;
    public static double servoPosB = 0;
    GamepadMapping controls;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
    }

    @Override
    public void loop() {
        robot.transfer.frontTransfer.setPower(servoPosF);
        robot.transfer.backTransfer.setPower(servoPosB);
    }
}
