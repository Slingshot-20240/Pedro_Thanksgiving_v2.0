package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfernf implements Subsystem {
    public static final Transfernf INSTANCE = new Transfernf();
    private Transfernf() {}

    public CRServoEx frontTransfer;
    public CRServoEx backTransfer;


    public Command on() {
        return new ParallelGroup(
                new SetPower(frontTransfer, -1.0),
                new SetPower(backTransfer, -1.0)
        );
    }

    public Command hotdog() {
        return new ParallelGroup(
                new SetPower(frontTransfer, -0.2),
                new SetPower(backTransfer, 1.0)
        );
    }


    public Command idle() {
        return new ParallelGroup(
                new SetPower(frontTransfer, -0.05),
                new SetPower(backTransfer, -1.0)
        );
    }



    @Override
    public void initialize() {
        frontTransfer = new CRServoEx("transferF");
        backTransfer = new CRServoEx("transferB");
    }

    @Override
    public void periodic(){

    }
}