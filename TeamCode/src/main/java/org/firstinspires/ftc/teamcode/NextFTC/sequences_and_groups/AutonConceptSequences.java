package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;



import org.firstinspires.ftc.teamcode.NextFTC.subsystems.Intake;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.VariableHood;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class AutonConceptSequences extends SubsystemGroup {
    public static final AutonConceptSequences INSTANCE = new AutonConceptSequences();

    private AutonConceptSequences() {
        super(
                AutonConceptSequences.INSTANCE,
                AutonConceptHWS.INSTANCE,
                Intake.INSTANCE, Transfer.INSTANCE,
                Shooter.INSTANCE, VariableHood.INSTANCE
        );
    }

    /**
     * Runs the Intake
     * Then waits for @speedUpTime
     * Then transfers for @transferTime
     */
    public final Command scoreSet(double speedUpTime, double transferTime) {
        return new SequentialGroup(
                Intake.INSTANCE.in,
                new SequentialGroup(
                        new Delay(speedUpTime),
                        AutonConceptHWS.INSTANCE.transferUpFor(transferTime)
                )
        );
    }

    /**
     * Runs the Intake while hotdogging
     * Sets Shooter Power
     */
    public final Command intakeSet(double shooterPower) {
        return new ParallelGroup(
                Intake.INSTANCE.in,
                Transfer.INSTANCE.hotdog,
                Shooter.INSTANCE.setShooterVel(shooterPower)
        );
    }


}