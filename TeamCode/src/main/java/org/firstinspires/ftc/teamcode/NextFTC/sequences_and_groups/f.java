package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;



import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.extensions.pedro.FollowPath;

public class f extends SubsystemGroup {
    public static final f i = new f();

    private f() {
        super(
                Intakenf.INSTANCE, Transfernf.INSTANCE,
                Shooternf.INSTANCE, Hoodnf.INSTANCE,
                Lednf.INSTANCE
        );
    }

    public final Command follow(PathChain path) {
        return new SequentialGroup(
                new FollowPath(path)
        );
    }
    public final Command follow(PathChain path, String color) {
        return new ParallelGroup(
                new FollowPath(path),
                Lednf.INSTANCE.color(color)
        );
    }
    public final Command follow(PathChain path, String color, boolean holdEnd) {
        return new ParallelGroup(
                new FollowPath(path, holdEnd),
                Lednf.INSTANCE.color(color)
        );
    }
    public final Command follow(PathChain path, String color, boolean holdEnd, double maxPower) {
        return new ParallelGroup(
                new FollowPath(path, holdEnd, maxPower),
                Lednf.INSTANCE.color(color)
        );
    }
    public final Command follow(PathChain path, boolean holdEnd) {
        return new SequentialGroup(
                new FollowPath(path, holdEnd)
        );
    }
    public final Command follow(PathChain path, boolean holdEnd, double maxPower) {
        return new SequentialGroup(
                new FollowPath(path, holdEnd, maxPower)
        );
    }




}