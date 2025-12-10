package org.firstinspires.ftc.teamcode.teleop.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.logi;

@TeleOp
public class BeeBoopTest extends OpMode {
    private Robot robot;
    private GamepadMapping controls;
    private Telemetry dashboardTelemetry;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        robot.intake.intakeOn();
        robot.transfer.transferOn();

        // TODO: RUPAL - only uncomment the actions when you're certain the values from telemetry won't damage the bot.
        // TODO: if the hood position is greater than .6 or less than .05, there's something wrong with the math

//        double targetVelocity = robot.shooter.calculateShooterVel();
//        //robot.shooter.setShooterVelocity(targetVelocity);
//        double targetHoodPos = robot.shooter.calculateHoodAngle();
        //robot.shooter.setHoodAngle(targetHoodPos);

//        dashboardTelemetry.addData("Calculated Target Velocity:", targetVelocity); // in ticks per second
//        dashboardTelemetry.addData("Calculated Target Hood Angle:", targetHoodPos); // in value from 0-1
//        dashboardTelemetry.addData("AT Distance", robot.shooter.cam.getATdist()); // in value from 0-1

    }
}
