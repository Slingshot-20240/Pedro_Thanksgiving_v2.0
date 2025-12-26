package org.firstinspires.ftc.teamcode.NextFTC.autonomous.LC;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Config
@Autonomous(name = "Winter")
public class WinterRedClose extends NextFTCOpMode {
    public WinterRedClose() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;

    public PathChain grabSet2;
    public PathChain scoreSet2;

    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain set4;
    public PathChain grabHp;
    public PathChain scoreHp;

    public Pose scorePose = new Pose(88,88);

    public void buildPaths() {
        follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(44))
                .build();



        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77),
                                new Pose(126, 83.4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0))


                //Gate 1
                .addPath(
                        new BezierCurve(
                                new Pose(126, 83.4),
                                new Pose(112, 77.000),
                                new Pose(129, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();


        scoreSet2 = follower().pathBuilder()
                //Score Set 2
                .addPath(
                        new BezierLine(new Pose(130, 71.000), scorePose)
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
                                new Pose(132.4, 54.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))



                //Gate 2
                .addPath(
                        new BezierCurve(
                                new Pose(132.4, 54.000),
                                new Pose(120.000, 54.000),
                                new Pose(106.000, 74.000),
                                new Pose(129, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))



                .build();

        scoreSet3 = follower().pathBuilder()
                //Score set 3
                .addPath(
                        new BezierLine(new Pose(129, 71.000), scorePose)
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.linear(Math.toRadians(90), Math.toRadians(69))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1.0,
                                        //TODO - tune the y value to make ball go in center due to newtons first law
                                        HeadingInterpolator.facingPoint(new Pose(144,144))
                                )
                        )
                )
                .build();


        set4 = follower()
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88, 39),
                                new Pose(82, 31),
                                new Pose(127, 35)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))


                .addPath(
                        new BezierLine(new Pose(127, 35), scorePose)
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0.0,
//                                        0.1,
//                                        HeadingInterpolator.constant(Math.toRadians(289.1))
//                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent.reverse()
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1.0,
                                        HeadingInterpolator.linear(
                                                follower().getHeading(),
                                                Math.toRadians(45)
                                        )
                                )
                        )
                )
                .build();



        grabHp = follower()
                .pathBuilder()

                // prepareHp
                .addPath(
                        new BezierLine(scorePose, new Pose(127, 40.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(300))

                // grabHp
                .addPath(
                        new BezierLine(
                                new Pose(127, 40.000),
                                new Pose(127, 11.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(285), Math.toRadians(285))
//
                //IF YOU ACTUALLY DO THIS MAKE SURE TO USE TANGENT INSTEAD!!!!! may have to reverse it
//                // assure backUp
//                .addPath(
//                        new BezierLine(
//                                new Pose(127, 11.000),
//                                new Pose(125, 15.000)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(285), Math.toRadians(285))
//
//                //assure grab
//                .addPath(
//                        new BezierLine(
//                                new Pose(125, 15.000),
//                                new Pose(125, 11.000)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(285), Math.toRadians(300))

                .build();



        scoreHp = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125, 11.000),
                                new Pose(90.000, 110.000)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0.0,
//                                        0.1,
//                                        HeadingInterpolator.constant(Math.toRadians(289.1))
//                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent.reverse()
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1.0,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(289.1),
                                                Math.toRadians(32)
                                        )
                                )
                        )
                )
                .build();



    }


    //TODO - figure out the max and min pos of servo! Does increasing bring hood up or down?
    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.34)
        );

    }

    private Command transferUpFor(double time) {
        return new ParallelGroup(
                Transfernf.INSTANCE.on(),
                new Delay(time)
        );
    }

    private Command transferSequence(PathChain pathChain, double transferTime) {
        return new SequentialGroup(
                Transfernf.INSTANCE.hotdog(),
                new WaitUntil(() -> pathChain.lastPath().isAtParametricEnd()),
                transferUpFor(transferTime)
        );
    }

    private Command transferSequenceDistance(PathChain pathChain, double transferTime, double proximity) {
        return new SequentialGroup(
                Transfernf.INSTANCE.hotdog(),
                new WaitUntil(() -> pathChain.lastPath().getDistanceRemaining() < proximity),
                transferUpFor(transferTime)
        );
    }

    private Command baseState(double shooterVel) {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),
                Shooternf.INSTANCE.setShooterVel(shooterVel),
                Hoodnf.INSTANCE.setHoodPos(0.37)
        );
    }
    private Command baseState(double shooterVel, double hoodPos) {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),
                Shooternf.INSTANCE.setShooterVel(shooterVel),
                Hoodnf.INSTANCE.setHoodPos(hoodPos)
        );
    }

    private Command autonomous() {
        return new SequentialGroup(

                //Preloads
                new ParallelGroup(
                        new FollowPath(scorePreloads),
                        baseState(-1250),

//                            transferSequence(scorePreloads,1.4)
                        new SequentialGroup(
                                new WaitUntil(() ->
                                        scorePreloads.lastPath().getDistanceRemaining() < 1.0
                                ),
                                transferUpFor(1.4)
                        )
                ),
                new Delay(0.33),

                //SET 2
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabSet2),
                                new Delay(0.1),
                                new FollowPath(scoreSet2)

                        ),
                        baseState(-1215),

                        transferSequence(scoreSet2,1.3)
                ),


                //SET 3
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabSet3),
                                new Delay(0.5),
                                new FollowPath(scoreSet3)

                        ),
                        baseState(-1215),

                        transferSequence(scoreSet3,1.3)
                ),

                //SET 4
                new ParallelGroup(
                        new FollowPath(set4),
                        baseState(-1215),

                        transferSequence(set4,1.3)
                ),

//                    new ParallelRaceGroup(
//                            new WaitUntil(() -> Shooternf.INSTANCE.rpmDraw(-1215,50)),
//                            //TODO, tune time (max time for transfer)
//                            transferUpFor(4)
//                            //you can use .asDeadline() at end of command also
//                    ),

                //SET 5 - Human Player
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabHp),
                                new FollowPath(scoreHp)
                        ),
                        baseState(-1215,0.41),

                        transferSequenceDistance(scoreHp,5, 1.5)
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

    @Override
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }
}