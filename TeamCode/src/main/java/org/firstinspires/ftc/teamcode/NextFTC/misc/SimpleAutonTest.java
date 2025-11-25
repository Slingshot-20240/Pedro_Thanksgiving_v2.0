package org.firstinspires.ftc.teamcode.NextFTC.misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems.Intake;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.VariableHood;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class SimpleAutonTest extends NextFTCOpMode {
    public SimpleAutonTest() {
        addComponents(
                new SubsystemComponent(
                        Intake.INSTANCE, VariableHood.INSTANCE,
                        Shooter.INSTANCE, Transfer.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
            Intake.INSTANCE.in(),
            Transfer.INSTANCE.hotdog(),
            VariableHood.INSTANCE.farSide()

        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }
}