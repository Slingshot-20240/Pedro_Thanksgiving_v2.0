package org.firstinspires.ftc.teamcode.NextFTC.autonomous.LC.far;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.asf;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.MTransfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "12 Blue Far Side Gate LC")
public class BlueFar12Gate extends NextFTCOpMode {

    public BlueFar12Gate() {
        addComponents(
                new SubsystemComponent(
                        asf.i, f.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, MTransfernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    private double mx(double x) { return 144 - x; }

    private double mh(double deg) {
        if (deg == 0) return 180;
        if (deg == 180) return 0;
        if (deg == 90 || deg == 268) return deg;
        return 180 - deg;
    }

    public PathChain scorePreloads, grabSet2, scoreSet2, grabSet3, scoreSet3, prepareHp, grabHp, scoreHp, park;

    public PathChain gate;

    Pose scorePose = new Pose(mx(88), 17);

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(mx(88), 8.2, Math.toRadians(mh(90))));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(mx(88), 8.2), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(mh(90)), Math.toRadians(mh(68.5)))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(mx(84), 65), new Pose(mx(132), 57)))
                .setLinearHeadingInterpolation(Math.toRadians(mh(68.5)), Math.toRadians(mh(0)))
                .build();


        gate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mx(132), 57.000),
                                new Pose(mx(89), 66.000),
                                new Pose(mx(127), 65)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(0)), Math.toRadians(mh(90)))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(new Pose(mx(127), 65), new Pose(mx(85), 60.5), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(mh(90)), Math.toRadians(mh(65)))
                .build();

        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(mx(87), 36), new Pose(mx(134), 32.8)))
                .setLinearHeadingInterpolation(Math.toRadians(mh(65)), Math.toRadians(mh(0)))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(mx(134), 32.8), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(mh(0)), Math.toRadians(mh(65)))
                .build();


        prepareHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(mx(132), 50)))
                .setLinearHeadingInterpolation(Math.toRadians(mh(65)), Math.toRadians(mh(290)))
                .build();

        grabHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(mx(132), 50), new Pose(mx(132), 11)))
                .setLinearHeadingInterpolation(Math.toRadians(mh(290)), Math.toRadians(mh(290)))
                .build();

        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(mx(134), 11), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(mh(290)), Math.toRadians(mh(65)))
                .build();

        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(mx(120), 65)))
                .setLinearHeadingInterpolation(Math.toRadians(mh(65)), Math.toRadians(mh(90)))
                .build();
    }

    private Command init_bot() {
        return Hoodnf.INSTANCE.setHoodPos(0.34);
    }

    private Command transferUpFor(double time) {
        return new ParallelGroup(
                MTransfernf.INSTANCE.on(),
                new Delay(time)
        );
    }

    private Command baseState() {
        return new ParallelGroup(
                MTransfernf.INSTANCE.hotdog(),
                Hoodnf.INSTANCE.setHoodPos(0.3)
        );
    }

    private Command autonomous() {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),
                new SequentialGroup(
                        // Preloads
                        new ParallelGroup(
                                f.i.follow(scorePreloads,"green"),
                                asf.i.baseState(-1520),
                                asf.i.transferSequenceDistance(scorePreloads,2.4,0,2.2)
                        ),

                        // SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet2),
                                        f.i.follow(gate),
                                        new Delay(1),
                                        f.i.follow(scoreSet2)
                                ),
                                asf.i.baseState(-1520),
                                asf.i.transferSequenceDistance(scoreSet2,2.4,0)

                        ),

                        // SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet3),
                                        f.i.follow(scoreSet3)
                                ),
                                asf.i.baseState(-1520),
                                asf.i.transferSequenceDistance(scoreSet3,2.4,0)
                        ),

                        // Human Player
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(prepareHp),
                                        //new Delay(0.3),
                                        f.i.follow(grabHp),
                                        f.i.follow(scoreHp, true)
                                ),
                                asf.i.baseState(-1520),
                                asf.i.transferSequenceDistance(scoreHp,2.4,0)
                        ),

                        f.i.follow(park)
                )
        );
    }

    @Override
    public void onInit() {
        buildPaths();
        init_bot().schedule();
        Shooternf.INSTANCE.disable();
    }

    @Override
    public void onStartButtonPressed() {
        autonomous().schedule();
        Shooternf.INSTANCE.enable();
    }
}
