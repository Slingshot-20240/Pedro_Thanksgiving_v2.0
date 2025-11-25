package org.firstinspires.ftc.teamcode.NextFTC.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    public MotorEx intake;

    private enum intake_states {
        IN (1.0),
        IDLE (0),
        OUT (-1.0);

        private final double intake_state;
        intake_states(double state) {
            this.intake_state = state;
        }
        public double getState() {
            return intake_state;
        }
    }

    public Command in = new SetPower(intake, intake_states.IN.getState());
    public Command idle = new SetPower(intake, intake_states.IDLE.getState());
    public Command out = new SetPower(intake, intake_states.OUT.getState());



    @Override
    public void initialize() {
        intake = new MotorEx("intake");
    }
}
