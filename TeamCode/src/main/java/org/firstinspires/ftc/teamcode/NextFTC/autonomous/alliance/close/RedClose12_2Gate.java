package org.firstinspires.ftc.teamcode.NextFTC.autonomous.alliance.close;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.field.Line;
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
@Autonomous(name = "Red 12 Close 2 Gate")
public class RedClose12_2Gate extends NextFTCOpMode {
    public RedClose12_2Gate() {
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

    public PathChain grabSet3, scoreSet3;

    public PathChain set4;
    public PathChain park;

    public Pose scorePose = new Pose(89,89);

    public void buildPaths() {
        follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(43))
//                .setTangentHeadingInterpolation().setReversed()
                .build();



        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77),
                                new Pose(126.5, 83.4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        hitGate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(126.5, 83.4),
                                new Pose(112, 77.000),
                                new Pose(130, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130, 71.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(43))
                .build();




        grabSet3 = follower()
                .pathBuilder()

                //Grab set 3
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.760, 55.000),
                                new Pose(79.313, 57.000),
                                new Pose(131, 54.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
//                .setTangentHeadingInterpolation()



                //Gate 2
                .addPath(
                        new BezierCurve(
                                new Pose(131, 54.000),
                                new Pose(118, 54.000),
                                new Pose(110.000, 69.000),
                                new Pose(127.4, 70)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))



                .build();

        scoreSet3 = follower().pathBuilder()
                //Score set 3
                .addPath(
                        new BezierLine(new Pose(127.4, 70), scorePose)
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        0.5,
//                                        //HeadingInterpolator.tangent.reverse()
//                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        1.0,
                                        //TODO - tune the y value to make ball go in center due to newtons first law
                                        HeadingInterpolator.facingPoint(new Pose(144,142))
                                )
                        )
                )
                .build();





        set4 = follower()
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(84, 41),
                                new Pose(80, 37),
                                new Pose(131, 33.3)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(new Pose(131, 33.3), scorePose)
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
                                        HeadingInterpolator.constant(Math.toRadians(43))
                                )
                        )
                )

                .build();




        park = follower()
                .pathBuilder()

                .addPath(
                        new BezierLine(scorePose, new Pose(110,70))
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(90))

                .build();





    }


    //TODO - figure out the max and min pos of servo! Does increasing bring hood up or down?
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
                                f.i.follow(grabSet2, "red"),
                                f.i.follow(hitGate, "yellow"),
                                f.i.follow(scoreSet2, "green")
                        ),
                        MTransfernf.INSTANCE.hotdog()
                ),
                Lednf.INSTANCE.color("yellow"),
                MTransfernf.INSTANCE.on(),
                new Delay(2),

                //------------------------—------------------------—------------------------—

                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(grabSet3, "red"),
                                f.i.follow(scoreSet3,"green")
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

               f.i.follow(park)

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