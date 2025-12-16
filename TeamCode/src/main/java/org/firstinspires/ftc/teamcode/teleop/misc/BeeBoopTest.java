package org.firstinspires.ftc.teamcode.teleop.misc;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@TeleOp
public class BeeBoopTest extends OpMode {
    private Robot robot;
    private GamepadMapping controls;
    private Telemetry dashboardTelemetry;

    public static double targetVelocity = 1293;
    public static double targetHoodPos = 0.26;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        robot.intake.intakeOn();

        if (controls.transfer.locked()) {
            robot.transfer.transferOn();
        } else {
            robot.transfer.hotDog();
        }

        robot.controls.update();
        robot.drivetrain.update();

        // TODO: RUPAL - only uncomment the actions when you're certain the values from telemetry won't damage the bot.
        // TODO: if the hood position is greater than .6 or less than .05, there's something wrong with the math

        double d = Robot.cam.getATdist();

//        double targetMPS = Shooter.calculateShooterMPS(d);
//        double targetVelocity = Shooter.convertMPSToRPM(targetMPS);
//
//        double targetHoodAngle = Shooter.calculateHoodAngle(d);
//        double targetHoodPos = Shooter.convertTargetAngleToHoodPos(targetHoodAngle);

        // TODO: ONCE FINAL, the above targetSomething declarations can be replaced with the following:
        // double targetVelocity = robot.shooter.calculateShooterRPM(d);
        // double targetHoodPos = robot.shooter.calculateHoodPos(d);

        robot.shooter.setShooterVelocity(-targetVelocity);
        robot.shooter.setHoodAngle(targetHoodPos);

//        dashboardTelemetry.addData("AT Distance (in)", d);
//        dashboardTelemetry.addData("Target Velocity (ms^-1):", targetMPS);
//        dashboardTelemetry.addData("Target Velocity (ticks•s^-1):", targetVelocity);
//        dashboardTelemetry.addData("Target Hood Angle (°):", Math.toDegrees(targetHoodAngle));
//        dashboardTelemetry.addData("Target Hood Position (0-1):", targetHoodPos);


        // why don't y'all use Log.d :(   -- jining
//        Log.d("SLINGSHOT_DEBUG_SHOOTER", "================================================================");
        Log.d("SLINGSHOT_DEBUG_SHOOTER", "AT Distance: " + String.format("%.3f", d) + " in");
//        Log.d("SLINGSHOT_DEBUG_SHOOTER", "Target Velocity: " + String.format("%.3f", targetMPS) + " ms^-1");
//        Log.d("SLINGSHOT_DEBUG_SHOOTER", "Target Velocity: " + String.format("%.3f", targetVelocity) + " ticks•s^-1");
//        Log.d("SLINGSHOT_DEBUG_SHOOTER", "Target Hood Angle: " + String.format("%.3f", Math.toDegrees(targetHoodAngle)) + "°");
//        Log.d("SLINGSHOT_DEBUG_SHOOTER", "Target Hood Pos: " + String.format("%.3f", targetHoodPos));
    }
}
