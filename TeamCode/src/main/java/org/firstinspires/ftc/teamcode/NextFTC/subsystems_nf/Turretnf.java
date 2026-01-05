package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Turretnf implements Subsystem {
    public static final Turretnf INSTANCE = new Turretnf();

    private Turretnf() {
    }

    public ServoEx turretServo1;
    public ServoEx turretServo2;

    public Command farBlue() {
        return new ParallelGroup(
                new SetPosition(turretServo1, 0),
                new SetPosition(turretServo2, 0)
        );
    }

    public Command closeBlue() {
        return new ParallelGroup(
                new SetPosition(turretServo1, 0),
                new SetPosition(turretServo2, 0)
        );
    }

    public Command farRed() {
        return new ParallelGroup(
                new SetPosition(turretServo1, 0),
                new SetPosition(turretServo2, 0)
        );
    }

    public Command closeRed() {
        return new ParallelGroup(
                new SetPosition(turretServo1, 0),
                new SetPosition(turretServo2, 0)
        );
    }



    @Override
    public void initialize() {
        turretServo1 = new ServoEx("turret1");
        turretServo2 = new ServoEx("turret2");
    }

    @Override
    public void periodic() {

    }
}
