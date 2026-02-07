package org.firstinspires.ftc.teamcode.NextFTC.autonomous.alliance.far;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.MTransfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "Blue Cycle HP Back")
public class BlueHpCycleBack extends NextFTCOpMode {

    public BlueHpCycleBack() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, MTransfernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;
    public PathChain grabSet2;
    public PathChain backAssureSet2;
    public PathChain frontAssureSet2;
    public PathChain scoreSet2;

    public PathChain grabHp;
    public PathChain backAssure;
    public PathChain frontAssure;
    public PathChain scoreHp;
    public PathChain park;

    Pose scorePose = new Pose(88, 17).mirror();

    public void buildPaths() {
        follower().setStartingPose(new Pose(88, 8.2, Math.toRadians(90)).mirror());

        scorePreloads = follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(88, 8.2).mirror(), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180-90), Math.toRadians(180-69.3))
                .build();

        //SET 2 SPECIFIC ONES
        grabSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(125, 25).mirror(),
                                new Pose(134.000, 11.000).mirror()
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        backAssureSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 11.000).mirror(), new Pose(125.000, 11).mirror())
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-0))
                .build();

        frontAssureSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.000, 11).mirror(), new Pose(134.000, 11).mirror())
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-0))

                .addPath(
                        new BezierLine(new Pose(125.000, 11).mirror(), new Pose(134.000, 8.2).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-300))
//                .setConstantHeadingInterpolation(Math.toRadians(180-0))
                .build();

        scoreSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 8.2).mirror(), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-300), Math.toRadians(180-69.3))
                .build();


        //REGULAR ONES
        grabHp = follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(128, 18).mirror(),
                                new Pose(134.000, 15).mirror()
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        backAssure = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 15).mirror(), new Pose(125.000, 18).mirror())
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-0))
                .build();

        frontAssure = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.000, 18).mirror(), new Pose(134.000, 20).mirror())
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-0))
                .build();

        scoreHp = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 20).mirror(), new Pose(88.000, 17.000).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-69.3))
                .build();


        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(134,20).mirror(), new Pose(120, 30).mirror()))
                .setLinearHeadingInterpolation(Math.toRadians(180-69.3), Math.toRadians(180-90))
                .build();

    }

    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.34),
                MTransfernf.INSTANCE.idle()
        );
    }

    private Command transferUpFor(double time) {
        return new ParallelGroup(
                //MTransfernf.INSTANCE.on(),
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
                                f.i.follow(scorePreloads, true),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1540)
                        ),
                        new Delay(0.5),
                        transferUpFor(2.2),

                        // SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet2),
                                        new Delay(0.2),
                                        f.i.follow(backAssureSet2),
                                        f.i.follow(frontAssureSet2, true,0.7),
                                        new Delay(0.2),
                                        f.i.follow(scoreSet2)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),


                        // SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabHp),
                                        new Delay(0.2),
                                        f.i.follow(backAssure),
                                        f.i.follow(frontAssure, true,0.7),
                                        new Delay(0.2),
                                        f.i.follow(scoreHp)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),

                        // SET 4
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabHp),
                                        new Delay(0.2),
                                        f.i.follow(backAssure),
                                        f.i.follow(frontAssure, true,0.7),
                                        new Delay(0.2),
                                        f.i.follow(scoreHp)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),

                        // SET 5
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabHp),
                                        new Delay(0.2),
                                        f.i.follow(backAssure),
                                        f.i.follow(frontAssure, true,0.7),
                                        new Delay(0.2),
                                        f.i.follow(park)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        )



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
