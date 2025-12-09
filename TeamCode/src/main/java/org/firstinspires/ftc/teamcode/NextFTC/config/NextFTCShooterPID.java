package org.firstinspires.ftc.teamcode.NextFTC.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@Disabled
@Configurable
@TeleOp
public class NextFTCShooterPID extends OpMode {

    MotorEx outtake1, outtake2;
    MotorGroup shooter;

    public static double p = 0.0005, i = 0.0, d = 0.0;
    public static double kV = 0, kA = 0, kS = 0;

    public static int shooterVel = 1100;

    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);
    //public static BasicFeedforwardParameters feedforwardParameters = new BasicFeedforwardParameters(kV, kA, kS);

    ControlSystem shooterController = ControlSystem.builder()
            .velPid(coefficients)
            //.basicFF(feedforwardParameters)
            .build();


    @Override
    public void init() {
        outtake1 = new MotorEx("outtake1");
        outtake2 = new MotorEx("outtake2");
        outtake2.reverse();

        shooter = new MotorGroup(outtake1, outtake2);
    }

    @Override
    public void loop() {
        shooter.setPower(
                shooterController.calculate(
                        new KineticState(0.0, shooter.getVelocity())
                )
        );


        telemetry.addData("Target Velocity", shooterVel);
        telemetry.addData("Current Velocity", shooter.getVelocity());
        telemetry.update();
    }
}


