package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;



import org.firstinspires.ftc.teamcode.NextFTC.subsystems.Intake;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.VariableHood;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class AutonConceptHWS extends SubsystemGroup {
    public static final AutonConceptHWS INSTANCE = new AutonConceptHWS();

    private AutonConceptHWS() {
        super(
                AutonConceptHWS.INSTANCE,
                Intake.INSTANCE, Transfer.INSTANCE,
                Shooter.INSTANCE, VariableHood.INSTANCE
        );
    }


    public Command intakeInFor(double time) {
        return new SequentialGroup(
                Intake.INSTANCE.in(),
                new Delay(time),
                Intake.INSTANCE.idle()
        );
    }

    public Command transferUpFor(double time) {
        return new SequentialGroup(
                Transfer.INSTANCE.on(),
                new Delay(time),
                Transfer.INSTANCE.hotdog()
        );
    }

    public Command shootFor(double time, double vel) {
        return new SequentialGroup(
                Shooter.INSTANCE.setShooterVel(vel),
                new Delay(time),
                Intake.INSTANCE.idle()
        );
    }



}