package org.firstinspires.ftc.teamcode.teleop.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Disabled
@TeleOp
public class TurretPIDTuner extends OpMode {
    Turret turret;
    public static double targetAngle;
    private Telemetry dashboardTelemetry;
    public static double p, i, d, f;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        turret.runToPos(targetAngle);
        turret.changePIDF(p, i, d, f);

        // TODO check units on these, pretty sure getPos is in degrees
        dashboardTelemetry.addData("Target (angle/rad): ", targetAngle);
        dashboardTelemetry.addData("Actual (angle/rad): ", turret.getPosition());
        dashboardTelemetry.update();
    }
}
