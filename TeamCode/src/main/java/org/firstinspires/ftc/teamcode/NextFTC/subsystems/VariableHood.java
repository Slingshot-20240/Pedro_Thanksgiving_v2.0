package org.firstinspires.ftc.teamcode.NextFTC.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class VariableHood implements Subsystem {
    public static final VariableHood INSTANCE = new VariableHood();
    private VariableHood() {}

    private final ServoEx variableHood = new ServoEx("variableHood");

    public Command closeSide() {
        return new SetPosition(variableHood, 0.5).requires(this);
    }
    public Command farSide() {
        return new SetPosition(variableHood, 0.3).requires(this);
    }

    public Command setHoodPos(double hoodPosition) {
        return new SetPosition(variableHood, hoodPosition).requires(this);
    }

}