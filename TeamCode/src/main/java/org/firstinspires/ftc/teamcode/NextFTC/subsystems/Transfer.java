package org.firstinspires.ftc.teamcode.NextFTC.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() {}

    private final CRServoEx frontTransfer = new CRServoEx("transferF");
    private final CRServoEx backTransfer = new CRServoEx("transferB");


    public Command on =
            new ParallelGroup(
                    new SetPower(frontTransfer, -1.0),
                    new SetPower(backTransfer, -1.0)
            );

    public Command hotdog =
            new ParallelGroup(
                    new SetPower(frontTransfer, -0.05),
                    new SetPower(backTransfer, -1.0)
            );

    public Command idle =
            new ParallelGroup(
                    new SetPower(frontTransfer, -0.05),
                    new SetPower(backTransfer, -1.0)
            );


}