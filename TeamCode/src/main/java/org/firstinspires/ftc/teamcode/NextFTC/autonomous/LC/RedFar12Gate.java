package org.firstinspires.ftc.teamcode.NextFTC.autonomous.LC;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.asf;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "RedFar12Gate LC")
public class RedFar12Gate extends NextFTCOpMode {

    public RedFar12Gate() {
        addComponents(
                new SubsystemComponent(
                        f.i,
                        asf.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE,
                        Lednf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads, grabSet2, gate, scoreSet2, grabSet3, scoreSet3, prepareHp, grabHp, scoreHp, park;

    Pose scorePose = new Pose(88, 17); // Red Far scoring position (144 - 56)

    public void buildPaths() {
        // Starting pose
        PedroComponent.follower().setStartingPose(new Pose(88, 8.2, Math.toRadians(90)));

        // Score preloads
        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(88, 8.2), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68)) // 180-112
                .build();

        // Grab Set 2
        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(86, 38), new Pose(132, 35))) // mirrored x
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();

        // Gate (optional extra path)
        gate = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(132, 57),
                        new Pose(89, 66),
                        new Pose(130, 67)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        // Score Set 2
        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(new Pose(12, 67), new Pose(59, 60.5), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                .build();

        // Grab Set 3
        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(57, 36), new Pose(10, 32.8)))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(180))
                .build();

        // Score Set 3
        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(10, 32.8), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(68))
                .build();

        // Human Player
        prepareHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(132, 50)))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(-60))
                .build();

        grabHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(132, 50), new Pose(132, 11)))
                .setLinearHeadingInterpolation(Math.toRadians(-70), Math.toRadians(-70))
                .build();

        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(134, 11), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(-70), Math.toRadians(68))
                .build();

        // Park
        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(132, 68)))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(90))
                .build();
    }

    private Command init_bot() {
        return Hoodnf.INSTANCE.setHoodPos(0.34);
    }

    private Command autonomous() {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),
                new SequentialGroup(
                        // Preloads
                        new ParallelGroup(
                                f.i.follow(scorePreloads, "green", true),
                                asf.i.baseState(-1515),
                                asf.i.transferSequence(scorePreloads, 2.2, 0.3)
                        ),

                        // SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet2,"red"),
                                        f.i.follow(gate),
                                        f.i.follow(scoreSet2,"green", true)
                                ),
                                asf.i.baseState(-1515),
                                asf.i.transferSequence(scoreSet2, 2.6)
                        ),

                        // SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet3,"red"),
                                        f.i.follow(scoreSet3)
                                ),
                                asf.i.baseState(-1515),
                                asf.i.transferSequence(scoreSet3, 2.6)
                        ),

                        // Human Player
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(prepareHp,"red"),
                                        f.i.follow(grabHp,"red"),
                                        f.i.follow(scoreHp,"green", true)
                                ),
                                asf.i.baseState(-1515),
                                asf.i.transferSequence(scoreHp, 2.4)
                        ),

                        f.i.follow(park,"green", true)
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
        Shooternf.INSTANCE.enable();
        autonomous().schedule();
    }
}
