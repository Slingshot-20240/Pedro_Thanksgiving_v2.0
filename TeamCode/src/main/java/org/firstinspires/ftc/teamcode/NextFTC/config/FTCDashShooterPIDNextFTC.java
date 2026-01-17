package org.firstinspires.ftc.teamcode.NextFTC.config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@TeleOp(name = "FTC Dash Shooter Config")
public class FTCDashShooterPIDNextFTC extends NextFTCOpMode {

    private Telemetry dashboardTelemetry;

    MotorEx outtake1, outtake2;
    MotorGroup shooter;

    public static double p = 3.5, i = 0.0, d = 0.0;
    public static double kV = 0, kA = 0, kS = 0;

    public static int targetShooterVel = 1100;

    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);
    public static BasicFeedforwardParameters feedforwardParameters = new BasicFeedforwardParameters(kV, kA, kS);

    ControlSystem shooterController = ControlSystem.builder()
            .velPid(coefficients)
            .basicFF(feedforwardParameters)
            .build();

    @Override
    public void onInit() {
        outtake1 = new MotorEx("outtake1");
        outtake2 = new MotorEx("outtake2");
        outtake2.reverse();

        shooter = new MotorGroup(outtake1, outtake2);

        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void onUpdate() {


        shooterController.setGoal(new KineticState(0, targetShooterVel));
        //TODO - figure out how to actually use the target shooter vel\ into this controller
        shooter.setPower(shooterController.calculate(shooter.getState()));


        dashboardTelemetry.addData("Target Velocity", targetShooterVel);
        dashboardTelemetry.addData("Current Velocity", shooter.getVelocity());
        dashboardTelemetry.update();
    }
}


