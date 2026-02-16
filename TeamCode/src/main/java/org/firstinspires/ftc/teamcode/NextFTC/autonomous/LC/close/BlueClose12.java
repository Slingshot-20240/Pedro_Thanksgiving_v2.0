package org.firstinspires.ftc.teamcode.NextFTC.autonomous.LC.close;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.asc;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.MTransfernf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "Blue 12 Close")
public class BlueClose12 extends NextFTCOpMode {
    public BlueClose12() {
        addComponents(
                new SubsystemComponent(
                        f.i, asc.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, MTransfernf.INSTANCE,
                        Lednf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;

    public PathChain grabSet2, hitGate, scoreSet2;

    public PathChain set3;

    public PathChain set4;
    public PathChain grabHp;
    public PathChain park;

    public Pose scorePose = new Pose(89,89).mirror();

    public void buildPaths() {
        follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)).mirror ());

        scorePreloads = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119).mirror(), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-36), Math.toRadians(180-43))
//                .setTangentHeadingInterpolation().setReversed()
                .build();



        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77).mirror(),
                                new Pose(126.5, 83.4).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-43), Math.toRadians(180-0))
                .build();

        hitGate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(126.5, 83.4).mirror(),
                                new Pose(112, 77.000).mirror(),
                                new Pose(130, 71.000).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130, 71.000).mirror(), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-90), Math.toRadians(180-43))
                .build();




        set3 = follower()
                .pathBuilder()

                //Grab set 3
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.760, 55.000).mirror(),
                                new Pose(79.313, 57.000).mirror(),
                                new Pose(132.4, 54).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-43), Math.toRadians(180-0))

                .addPath(
                        new BezierCurve(
                                new Pose(132.4, 54).mirror(),
                                new Pose(91.262, 56.240).mirror(),
                                new Pose(95.176, 82.815).mirror(),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-43))



                .build();


        set4 = follower()
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(84, 41).mirror(),
                                new Pose(80, 37).mirror(),
                                new Pose(131, 33.3).mirror()
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(180-43), Math.toRadians(180-0))
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(new Pose(131, 33.3).mirror(), scorePose)
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
                                        HeadingInterpolator.constant(Math.toRadians(180-43))
                                )
                        )
                )

                .build();




        grabHp = follower()
                .pathBuilder()

                //grab
                .addPath(
                        new BezierCurve(scorePose, new Pose(120,25).mirror(), new Pose(126,13).mirror())
                )
                .setTangentHeadingInterpolation()


                //assure backup
                .addPath(
                        new BezierLine(new Pose(126,13).mirror(), new Pose(119,15).mirror())
                )
                .setTangentHeadingInterpolation().setReversed()

                .addPath(
                        new BezierLine(new Pose(119,15).mirror(), new Pose(126,13).mirror())
                )
                .setTangentHeadingInterpolation()

                .build();



        park = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126, 13).mirror(),
                                new Pose(110, 55).mirror()
                        )
                )
                .setTangentHeadingInterpolation().setReversed()


                .build();



    }


    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.35),
                MTransfernf.INSTANCE.idle()
        );

    }


    private Command autonomous() {
        return new SequentialGroup(

                //------------------------—------------------------—------------------------—
                new ParallelGroup(
                        f.i.follow(scorePreloads, "green"),
                        asc.i.baseState(-1260),
                        MTransfernf.INSTANCE.hotdog()
                ),
                Lednf.INSTANCE.color("yellow"),
                MTransfernf.INSTANCE.on(),
                new Delay(2),
//
                //------------------------—------------------------—------------------------—

                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(grabSet2, "green"),
                                f.i.follow(hitGate),
                                f.i.follow(scoreSet2)
                        ),
                        MTransfernf.INSTANCE.hotdog()
                ),
                Lednf.INSTANCE.color("yellow"),
                MTransfernf.INSTANCE.on(),
                new Delay(2),

                //------------------------—------------------------—------------------------—

                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(set3, "green")
                        ),
                        MTransfernf.INSTANCE.hotdog()
                ),
                Lednf.INSTANCE.color("yellow"),
                MTransfernf.INSTANCE.on(),
                new Delay(2),

                //------------------------—------------------------—------------------------—

                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(set4, "green")
                        ),
                        MTransfernf.INSTANCE.hotdog()
                ),
                Lednf.INSTANCE.color("yellow"),
                MTransfernf.INSTANCE.on(),
                new Delay(2),

                //------------------------—------------------------—------------------------—

                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(grabHp, "green"),
                                f.i.follow(park,"red")
                        ),
                        MTransfernf.INSTANCE.hotdog()
                ),
                Lednf.INSTANCE.color("yellow")

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

    @Override
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }
}