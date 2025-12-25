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

    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(0.342,0,0.001)
            //.velPid(1.52,0,0)
            .build();

    private boolean enabled = false;

    private enum shooterStates {
        IDLE (0),
        CLOSE_SIDE (-1167),
        FAR_SIDE (-1420);

        private final double shooterState;
        shooterStates(double state) { this.shooterState = state; }
        public double getState() { return shooterState; }
    }

    public Command closeSide() {
        return new RunToVelocity(shooterController, shooterStates.CLOSE_SIDE.getState());
    }
    public Command farSide() {
        return new RunToVelocity(shooterController, shooterStates.FAR_SIDE.getState());
    }
    public Command idle() {
        return new RunToVelocity(shooterController, shooterStates.IDLE.getState());
    }
    public Command setShooterVel(double shooterVel) {
        return new RunToVelocity(shooterController, shooterVel).requires(shooter);
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        shooter.setPower(0); //prevents shooter on in init
    }

    private boolean ball1detected, ball2detected, ball3detected;
    private boolean isRecovered = true;
    public boolean rpmDraw(double requestedVel, double dipThreshold) {
        double currentVel = shooter.getVelocity();
        double recoveryThreshold = requestedVel + 50;

        //check if the shooter has recovered speed since the last shot
        if (!isRecovered && currentVel <= recoveryThreshold) {
            isRecovered = true;
        }

        //only look for a dip if we have recovered from the previous shot
        if (isRecovered && currentVel > (requestedVel + dipThreshold)) {
            if (!ball1detected) {
                ball1detected = true;
            } else if (!ball2detected) {
                ball2detected = true;
            } else if (!ball3detected) {
                ball3detected = true;
            }

            isRecovered = false;
        }

        return ball3detected;
    }

    @Override
    public void initialize() {
        outtake1 = new MotorEx("outtake1");
        outtake2 = new MotorEx("outtake2");
        outtake2.reverse();
        shooter = new MotorGroup(outtake1, outtake2);

        disable(); //makes sure motor is off during init
    }

    @Override
    public void periodic() {

        if (!enabled) {
            shooter.setPower(0);
            return;
        }

        shooter.setPower(shooterController.calculate(shooter.getState()));
    }
}