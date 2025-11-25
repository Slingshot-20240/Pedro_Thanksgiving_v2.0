package org.firstinspires.ftc.teamcode.NextFTC.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    public MotorEx intake;

    private final ControlSystem intakeController = ControlSystem.builder()
            .velPid(0,0,0)
            .build();
    private enum intakeStates {
        IN (1.0),
        IDLE (0),
        OUT (-1.0);

        private final double intakeState;
        intakeStates(double state) {
            this.intakeState = state;
        }
        public double getState() {
            return intakeState;
        }
    }

    public Command in() {
        return new SetPower(intake, Intake.intakeStates.IN.getState());
    }
    public Command idle() {
        return new SetPower(intake, Intake.intakeStates.IDLE.getState());
    }
    public Command out() {
        return new SetPower(intake, Intake.intakeStates.OUT.getState());
    }

    @Override
    public void initialize() {
        intake = new MotorEx("intake");
    }

    @Override
    public void periodic(){
        intake.setPower(intakeController.calculate(intake.getState()));
    }
}
