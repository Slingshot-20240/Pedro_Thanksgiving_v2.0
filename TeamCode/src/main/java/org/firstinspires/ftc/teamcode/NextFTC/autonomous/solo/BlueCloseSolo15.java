package org.firstinspires.ftc.teamcode.NextFTC.autonomous.solo;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.asc;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@Autonomous(name = "Blue 15 Solo")
public class BlueCloseSolo15 extends NextFTCOpMode {
    public BlueCloseSolo15() {
        addComponents(
                new SubsystemComponent(
                        f.i,
                        asc.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE,
                        Lednf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;

    public PathChain grabMiddleSet, scoreMiddleSet;
    public PathChain grabGate, scoreGate;
    public PathChain backAssureGate, assureGate;
    public PathChain grabSet4, scoreSet4;

    public PathChain grabSet2, scoreSet2;
    public PathChain park;

    public Pose scorePose = new Pose(56, 88);

    public void buildPaths() {
        double gateHeading = 162;

        PedroComponent.follower().setStartingPose(
                new Pose(17.8, 119, Math.toRadians(144))
        );

        scorePreloads = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17.8, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
                .build();

        grabMiddleSet = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(56.24, 55.000),
                                new Pose(64.687, 57.000),
                                new Pose(11.6, 54.000)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(

                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.linear(Math.toRadians(135), Math.toRadians(180))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(180))
                                )
                        )
                )
                .build();

        scoreMiddleSet = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(11.6, 54.000),
                                new Pose(46.5, 60.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        grabGate = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(62.0, 58.000),
                                new Pose(12.5, 60.2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(162))

                .addPath(
                        new BezierCurve(
                                new Pose(12.5, 60.2),
                                new Pose(12.5, 60)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(162), Math.toRadians(158))
                .build();

        scoreGate = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.5, 60),
                                new Pose(50.0, 64.000),
                                scorePose
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(

                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.linear(Math.toRadians(158), Math.toRadians(180))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1.0,
                                        HeadingInterpolator.linear(Math.toRadians(180), Math.toRadians(135))
                                )
                        )
                )
                .build();

        grabSet4 = follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(56, 39),
                                new Pose(62, 31),
                                new Pose(13, 33.3)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreSet4 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13, 33.3),
                                scorePose
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(

                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.tangent.reverse()
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(135))
                                )
                        )
                )
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(51.708, 77),
                                new Pose(18, 80)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        scoreSet2 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18, 80), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(36, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();
    }
}

