package org.firstinspires.ftc.teamcode.teleop.misc;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
@Disabled
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

        if (controls.transfer.locked()) {
            robot.transfer.transferOn();
        } else {
            robot.transfer.hotDog();
        }

        robot.controls.update();
        robot.drivetrain.update();

        // TODO: if the hood position is greater than .6 or less than .05, there's something wrong with the math

        double d = Robot.cam.getTargetArtifactTravelDistanceX();

        double targetVelocity = robot.shooter.calculateShooterRPM(d);
        double targetHoodPos = robot.shooter.calculateHoodPos(d);

        robot.shooter.setShooterVelocity(-targetVelocity);
        robot.shooter.setHoodAngle(targetHoodPos);

        dashboardTelemetry.addData("Target Artifact Travel Distance X (\")", d);
        dashboardTelemetry.addData("Target Velocity (ticks•s^-1):", targetVelocity);
        dashboardTelemetry.addData("Target Hood Position (0-1):", targetHoodPos);


        // why don't y'all use Log.d :(   -- jining
        Log.d("SLINGSHOT_DEBUG_SHOOTER", "================================================================");
        Log.d("SLINGSHOT_DEBUG_SHOOTER", "Target Artifact Travel Distance X: " + String.format("%.3f", d) + "\"");
        Log.d("SLINGSHOT_DEBUG_SHOOTER", "Target Velocity: " + String.format("%.3f", targetVelocity) + " ticks•s^-1");
        Log.d("SLINGSHOT_DEBUG_SHOOTER", "Target Hood Pos: " + String.format("%.3f", targetHoodPos));
    }
}
