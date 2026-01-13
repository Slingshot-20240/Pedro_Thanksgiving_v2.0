package org.firstinspires.ftc.teamcode.NextFTC.autonomous.alliance;

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
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
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

import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Config
@Autonomous(name = "Red Pushbot 12")
public class RedPushBot12 extends NextFTCOpMode {
    public RedPushBot12() {
        addComponents(
                new SubsystemComponent(
                        f.i, asc.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE,
                        Lednf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;

    public PathChain grabSet2;
    public PathChain scoreSet2;
    public PathChain gate;

    public PathChain grabSet3;
    public PathChain scoreSet3;

    public PathChain grabSet4;
    public PathChain scoreSet4;

    public PathChain pushBot;
    public PathChain park;

    public Pose scorePose = new Pose(89,89);

    public void buildPaths() {
        follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
//                .setTangentHeadingInterpolation().setReversed()
                .build();



        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292,77),
                                new Pose(127, 79)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        gate = PedroComponent.follower()
                .pathBuilder()
                //Gate 1
                .addPath(
                        new BezierCurve(
                                new Pose(127, 79),
                                new Pose(109, 76),
                                new Pose(128, 72)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();



        scoreSet2 = follower().pathBuilder()
                //Score Set 2
                .addPath(
                        new BezierLine(new Pose(128, 72), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();



        grabSet3 = follower()
                .pathBuilder()

                //Grab set 3
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.760, 55.000),
                                new Pose(79.313, 57.000),
                                new Pose(132.4, 54)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        scoreSet3 = follower().pathBuilder()
                //Score set 3
                .addPath(
                        new BezierLine(new Pose(132.4, 54), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();


        grabSet4 = follower()
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88, 39),
                                new Pose(82, 31),
                                new Pose(131.3, 33.3)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .setTangentHeadingInterpolation()

                .build();


        scoreSet4 = follower()
                .pathBuilder()

                .addPath(
                        new BezierLine(new Pose(131.3, 33.3), scorePose)
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
                                        //HeadingInterpolator.constant(Math.toRadians(45))
                                        //TODO - tune this value!
                                        HeadingInterpolator.facingPoint(new Pose(144,144))
                                )
                        )
                )
                .build();

        pushBot = follower().
                pathBuilder()
                .addPath(
                    new BezierLine(
                            scorePose,
                            new Pose(83, 10)
                    )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                )
                        )
                )

                .addPath(
                        new BezierLine(
                                new Pose(83, 10),
                                new Pose(126, 10)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();


        park = follower().
                pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126, 10),
                                new Pose(112, 70)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();




    }


    //TODO - figure out the max and min pos of servo! Does increasing bring hood up or down?
    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.35),
                Transfernf.INSTANCE.idle()
        );

    }


    private Command autonomous() {
        return new SequentialGroup(


                new ParallelGroup(
                        f.i.follow(scorePreloads, "green"),
                        asc.i.baseState(-1240),
                        Transfernf.INSTANCE.hotdog()
                ),
                asc.i.transferUpFor(2),


                //SET 2
                new ParallelGroup(
                        new SequentialGroup(
                                new ParallelGroup(
                                        f.i.follow(grabSet2, "red"),
                                        Transfernf.INSTANCE.pickup(grabSet2,2)
                                ),
                                f.i.follow(gate,"yellow"),
                                new Delay(0.4),
                                f.i.follow(scoreSet2,"green")
                        ),
                        asc.i.baseState(-1240)
                ),
                asc.i.transferUpFor(2),


                //SET 3
                new ParallelGroup(
                        new SequentialGroup(
                                new ParallelGroup(
                                        f.i.follow(grabSet3, "red"),
                                        Transfernf.INSTANCE.pickup(grabSet3,2)
                                ),
                                f.i.follow(scoreSet3,"green")
                        ),
                        asc.i.baseState(-1240)
                ),
                asc.i.transferUpFor(2),

                //SET 4
                new ParallelGroup(
                        new SequentialGroup(
                                new ParallelGroup(
                                        f.i.follow(grabSet4, "red"),
                                        Transfernf.INSTANCE.pickup(grabSet4,2)
                                ),
                                f.i.follow(scoreSet4,"green")

                        ),
                        asc.i.baseState(-1240)
                ),
                asc.i.transferUpFor(2),

                //PUSH BOT AND PARK
                f.i.follow(pushBot,"yellow"),
                f.i.follow(park,"green")

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