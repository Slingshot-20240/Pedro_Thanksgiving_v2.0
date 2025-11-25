package org.firstinspires.ftc.teamcode.NextFTC.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToState;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    public MotorEx outtake1, outtake2;
    public MotorGroup shooter;

    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .build();

    private enum shooterStates {
        IDLE (0),
        CLOSE_SIDE (0.5),
        FAR_SIDE (0.7);

        private final double shooterState;
        shooterStates(double state) {
            this.shooterState = state;
        }
        public double getState() {
            return shooterState;
        }
    }

    public Command closeSide() {
        return new SetPower(shooter, shooterStates.CLOSE_SIDE.getState());
    }
    public Command farSide() {
        return new SetPower(shooter, shooterStates.FAR_SIDE.getState());
    }
    public Command idle() {
        return new SetPower(shooter, shooterStates.IDLE.getState());
    }

    public Command setShooterVel(double shooterVel) {
        return new SetPower(shooter, shooterVel);
    }


    @Override
    public void initialize() {
        outtake1 = new MotorEx("outtake1");
        outtake2 = new MotorEx("outtake2");
        //outtake1.reverse();
        new MotorGroup(outtake1, outtake2);
    }
    @Override
    public void periodic(){
        outtake1.setPower(shooterController.calculate(outtake1.getState()));
        outtake2.setPower(shooterController.calculate(outtake2.getState()));
    }
}
