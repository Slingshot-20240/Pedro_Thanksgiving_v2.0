package org.firstinspires.ftc.teamcode.NextFTC.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    public MotorEx outtake1, outtake2;

    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .build();

    private enum shooter_states {
        IDLE (0),
        CLOSE_SIDE (0.5),
        FAR_SIDE (0.7);

        private final double shooter_state;
        shooter_states(double state) {
            this.shooter_state = state;
        }
        public double getState() {
            return shooter_state;
        }
    }

    public Command closeSide() {
        return new RunToVelocity(shooterController, shooter_states.CLOSE_SIDE.getState());
    }
    public Command farSide() {
        return new RunToVelocity(shooterController, shooter_states.FAR_SIDE.getState());
    }
    public Command idle() {
        return new RunToVelocity(shooterController, shooter_states.IDLE.getState());
    }

    public Command setShooterVel(double shooterVel) {
        return new RunToVelocity(shooterController, shooterVel);
    }


    @Override
    public void initialize() {
        outtake1 = new MotorEx("outtake1");
        outtake2 = new MotorEx("outtake2");
        //outtake1.reverse();
        MotorGroup outtake = new MotorGroup(outtake1, outtake2);
    }
}
