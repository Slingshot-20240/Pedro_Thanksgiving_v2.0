package org.firstinspires.ftc.teamcode.NextFTC.autonomous.solo;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
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
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "0 18 Redo")
public class Solo18 extends NextFTCOpMode {
    public Solo18() {
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

    public PathChain grabMiddleSet, scoreMiddleSet;
    public PathChain grabGate, scoreGate;
    public PathChain grabGate2, scoreGate2;
    public PathChain grabGate3, scoreGate3;
    public PathChain grabSet4, scoreSet4;

    public PathChain grabSet2, scoreSet2;

    public Pose scorePose = new Pose(87,87);
    //TODO - try different types, public private regular etc.

    public void buildPaths() {
        double gateHeading = 16;

        PedroComponent.follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(43.5))
                .build();

        grabMiddleSet = PedroComponent.follower().pathBuilder()

                //Grab Middle Set
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.760, 55.000),
                                new Pose(79.313, 57.000),
                                new Pose(132.4, 54.000)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(

                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.linear(Math.toRadians(43.5), Math.toRadians(0))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1.0,
                                        HeadingInterpolator.constant(0)
                                )
                        )
                )


                .build();

        scoreMiddleSet = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.4, 54.000),
                                new Pose(97.500, 60.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43.5))
                .build();


        //----------- GATES ------------\\
        grabGate = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(82.000, 58.000),
                                new Pose(131.5, 60.38)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(gateHeading))
                .build();

        scoreGate = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(131.5, 60.38),
                                new Pose(94.000, 64.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(43.5))
                .build();

        grabGate2 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(82.000, 58.000),
                                new Pose(131.5, 60.38)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(gateHeading))
                .build();

        scoreGate2 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(131.5, 60.38),
                                new Pose(94.000, 64.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(43.5))
                .build();

        grabGate3 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(82.000, 58.000),
                                new Pose(131.5, 60.1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(gateHeading))
                .build();

        scoreGate3 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(131.5, 60.1),
                                new Pose(94.000, 64.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(43.5))
                .build();



        //----------- Done ------------\\


        grabSet4 = follower()
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88, 39),
                                new Pose(82, 31),
                                new Pose(131, 33.3)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(0))
                .setTangentHeadingInterpolation()


                .build();


        scoreSet4 = follower()
                .pathBuilder()

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
                                        HeadingInterpolator.constant(Math.toRadians(42))
                                )
                        )
                )
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77),
                                new Pose(126, 80)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(0))


                .build();


        scoreSet2 = PedroComponent.follower().pathBuilder()
                //Score Set 2
                .addPath(
                        new BezierLine(new Pose(126, 80), new Pose(90,110))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(28.4))
                .build();


    }


    //TODO - figure out the max and min pos of servo! Does increasing bring hood up or down?
    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.34),
                Transfernf.INSTANCE.idle()
        );

    }

    private Command transferUpFor(double time) {
        return new ParallelGroup(
                Transfernf.INSTANCE.stepOn(),
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
                Hoodnf.INSTANCE.setHoodPos(0.34)
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


                new ParallelGroup(
                        new FollowPath(scorePreloads),
                        baseState(-1240),
                        Transfernf.INSTANCE.hotdog()
                ),
                transferUpFor(1),


                //SET 2
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabMiddleSet),
//                                new Delay(0.14),
                                new FollowPath(scoreMiddleSet)

                        ),
                        baseState(-1240),

                        transferSequence(scoreMiddleSet,0.9)
                ),


                //SET 3
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabGate),
                                new Delay(1.3),
                                new FollowPath(scoreGate)

                        ),
                        baseState(-1240),

                        transferSequence(scoreGate,0.9)
                ),

                //SET
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabGate2),
                                new Delay(1.3),
                                new FollowPath(scoreGate2)

                        ),
                        baseState(-1240),

                        transferSequence(scoreGate2,0.9)
                ),

                //SET 4
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabSet4),
//                                new Delay(0.7),
                                new FollowPath(scoreSet4)

                        ),
                        baseState(-1240),

                        transferSequence(scoreSet4,0.9)
                ),



                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabSet2),
//                                new Delay(0.9),
                                new FollowPath(scoreSet2)
                        ),
                        baseState(-1200,0.38),

                        transferSequence(scoreSet2,5)
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
        PoseStorage.startingPose = PedroComponent.follower().getPose();
    }
}