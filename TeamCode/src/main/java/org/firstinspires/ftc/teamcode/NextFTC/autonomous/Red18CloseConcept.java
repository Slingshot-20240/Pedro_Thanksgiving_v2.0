package org.firstinspires.ftc.teamcode.NextFTC.autonomous;

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

import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Config
@Autonomous(name = "18 Concept")
public class Red18CloseConcept extends NextFTCOpMode {
    public Red18CloseConcept() {
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

    public PathChain grabMiddleSet;
    public PathChain scoreMiddleSet;
    public PathChain grabGate;
    public PathChain scoreGate;

    public PathChain grabGate2;
    public PathChain scoreGate2;

    public PathChain grabGate3;
    public PathChain scoreGate3;

    public PathChain grabSet2;
    public PathChain scoreSet2;


    public Pose scorePose = new Pose(89,89);
    public double gateHeading = 27;

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


        grabMiddleSet = follower()
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
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(

                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.linear(Math.toRadians(45), Math.toRadians(0))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1.0,
                                        HeadingInterpolator.constant(0)
                                )
                        )
                )


                .build();

        scoreMiddleSet = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.4, 54.000),
                                new Pose(97.500, 60.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        //----------- GATES ------------\\
        grabGate = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(82.000, 58.000),
                                new Pose(133.000, 60)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(gateHeading))
                .build();

        scoreGate = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.000, 60),
                                new Pose(94.000, 64.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(45))
                .build();

        grabGate2 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(82.000, 58.000),
                                new Pose(133.000, 60)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(gateHeading))
                .build();

        scoreGate2 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.000, 60),
                                new Pose(94.000, 64.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(45))
                .build();

        grabGate3 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(82.000, 58.000),
                                new Pose(133.000, 60)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(gateHeading))
                .build();

        scoreGate3 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.000, 60),
                                new Pose(94.000, 64.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(45))
                .build();



        //----------- Done ------------\\


        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77),
                                new Pose(126, 80)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))


                .build();


        scoreSet2 = follower().pathBuilder()
                //Score Set 2
                .addPath(
                        new BezierLine(new Pose(126, 80), new Pose(90,110))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(28.4))
                .build();





    }


    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.32),
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


                new ParallelGroup(
                        new FollowPath(scorePreloads),
                        baseState(-1270),
                        Transfernf.INSTANCE.on().afterTime(2)
                ),
                Transfernf.INSTANCE.stepOn(),
                Transfernf.INSTANCE.forceBackOn(),
                new Delay(0.9),
                Transfernf.INSTANCE.hotdog(),


                //SET 2
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabMiddleSet),
                                //new Delay(0.14),
                                new FollowPath(scoreMiddleSet)

                        ),
                        baseState(-1270),

                        transferSequence(scoreMiddleSet,0.97)
                ),


                //SET 3
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabGate),
                                new Delay(0.8),
                                new FollowPath(scoreGate)

                        ),
                        baseState(-1270),

                        transferSequence(scoreGate,0.97)
                ),

                //SET 4
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabGate2),
                                new Delay(0.8),
                                new FollowPath(scoreGate2)

                        ),
                        baseState(-1270),

                        transferSequence(scoreGate2,0.97)
                ),


                //SET 5
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabGate3),
                                new Delay(1),
                                new FollowPath(scoreGate3)

                        ),
                        baseState(-1270),

                        transferSequence(scoreGate3,0.97)
                ),


                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabSet2),
                                //new Delay(0.35),
                                new FollowPath(scoreSet2)
                        ),
                        baseState(-1200,0.37),

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
        PoseStorage.startingPose = follower().getPose();
    }
}