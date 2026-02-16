package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class MTransfernf implements Subsystem {
    public static final MTransfernf INSTANCE = new MTransfernf();
    private MTransfernf() {}

    public MotorEx frontTransfer;
    public CRServoEx backTransfer;

    private final ControlSystem transferController = ControlSystem.builder()
            .velPid(10, 0, 0)
            .basicFF(26)
            .build();

    public Command on() {
        return new ParallelGroup(
//                new RunToVelocity(transferController, -2000),
                new SetPower(frontTransfer,-1.0),
                new SetPower(backTransfer, -1.0)
        ).addRequirements(frontTransfer, backTransfer);
    }
    public Command farOn() {
        return new ParallelGroup(
//                new RunToVelocity(transferController, -2000),
                new SetPower(frontTransfer,-0.8),
                new SetPower(backTransfer, -1.0)
        ).addRequirements(frontTransfer, backTransfer);
    }

    public Command hotdog() {
        return new ParallelGroup(
//                new RunToVelocity(transferController, -600),
                new SetPower(frontTransfer,-0.3),
                new SetPower(backTransfer, 1.0)
        ).addRequirements(frontTransfer, backTransfer);
    }
    //public Command onInstant() {
    //    return new ParallelGroup(
    //            new InstantCommand(() -> frontTransfer.setVelocity(-2000)),
    //            new InstantCommand(() -> backTransfer.setPower(1.0))
    //    );
    //}

    public Command idle() {
        return new ParallelGroup(
//                new RunToVelocity(transferController, 0),
                new SetPower(frontTransfer,0),
                new SetPower(backTransfer, 0)
        ).addRequirements(frontTransfer, backTransfer);
    }


    @Override
    public void initialize() {
        frontTransfer = new MotorEx("transferF");
        backTransfer = new CRServoEx("transferB");

//        frontTransfer.reversed();
//        frontTransfer.reverse();

    }

    @Override
    public void periodic() {}
}