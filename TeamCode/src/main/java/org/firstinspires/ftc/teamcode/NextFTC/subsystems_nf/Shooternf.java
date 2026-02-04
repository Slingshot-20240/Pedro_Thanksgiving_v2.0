package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Shooternf implements Subsystem {
    public static final Shooternf INSTANCE = new Shooternf();
    private Shooternf() { }

    public MotorEx outtake1, outtake2;
    public MotorGroup shooter;

    private final ControlSystem closeShooterController = ControlSystem.builder()
            .velPid(10, 0, 0.004)
            //old 3.5, 0.002, ff = 0.001
            .basicFF(0.006)
            .build();

    private final ControlSystem farShooterController = ControlSystem.builder()
            //extreme values
            .velPid(15, 0, 0.004)
            //old 5.5, 0.001, ff = 0.005
            .basicFF(0.008)
            .build();

    private boolean enabled = false;

    private enum ShooterControllerMode {
        CLOSE,
        FAR
    }

    private ShooterControllerMode currentControllerMode = ShooterControllerMode.CLOSE;


    public Command closeSide() {
        currentControllerMode = ShooterControllerMode.CLOSE;
        return new RunToVelocity(closeShooterController, -1210).requires(shooter);
    }

    public Command farSide() {
        currentControllerMode = ShooterControllerMode.FAR;
        return new RunToVelocity(farShooterController, -1450).requires(shooter);
    }

    public Command idle() {
        currentControllerMode = ShooterControllerMode.CLOSE;
        return new RunToVelocity(closeShooterController, 0).requires(shooter);
    }

    public Command setShooterVel(double shooterVel) {
        currentControllerMode = ShooterControllerMode.CLOSE;
        return new RunToVelocity(closeShooterController, shooterVel).requires(shooter);
    }

    public Command setShooterVel(double shooterVel, boolean farSide) {
        if (farSide) {
            currentControllerMode = ShooterControllerMode.FAR;
        } else {
            currentControllerMode = ShooterControllerMode.CLOSE;
        }

        return new RunToVelocity(
                farSide ? farShooterController : closeShooterController,
                shooterVel
        ).requires(shooter);
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        shooter.setPower(0);
    }

    @Override
    public void initialize() {
        outtake1 = new MotorEx("outtakeTop");
        outtake2 = new MotorEx("outtakeBot");
        outtake2.reverse();
        shooter = new MotorGroup(outtake1, outtake2);

        disable();
    }

    @Override
    public void periodic() {
        if (!enabled) {
            shooter.setPower(0);
            return;
        }

        ControlSystem controller;

        if (currentControllerMode == ShooterControllerMode.FAR) {
            controller = farShooterController;
        } else {
            controller = closeShooterController;
        }

        shooter.setPower(controller.calculate(shooter.getState()));
    }
}
