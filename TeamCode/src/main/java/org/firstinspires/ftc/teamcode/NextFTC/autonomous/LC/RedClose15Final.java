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
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Config
@Autonomous(name = "15 Final Red Close")
public class RedClose15Final extends NextFTCOpMode {
    public RedClose15Final() {
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

    public PathChain grabSet4;
    public PathChain scoreSet4;
    public PathChain grabHp;
    public PathChain scoreHp;

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
                                new Pose(92.292, 77),
                                new Pose(126, 80)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))


            //Gate 1
                .addPath(
                        new BezierCurve(
                                new Pose(126, 80),
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
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(44))
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
                .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0))
//                .setTangentHeadingInterpolation()



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
//                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
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




        grabHp = follower()
                .pathBuilder()

                //grab
                .addPath(
                        new BezierLine(scorePose, new Pose(126.5,13))
                )
                .setTangentHeadingInterpolation()


                //assure backup
                .addPath(
                        new BezierLine(new Pose(126.5,13), new Pose(122,15))
                )
                .setTangentHeadingInterpolation().setReversed()

                //assure pickup
                .addPath(
                        new BezierLine(new Pose(122,15), new Pose(126.5,13))
                )
                .setTangentHeadingInterpolation()

                .build();



        scoreHp = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126.5, 13),
                                new Pose(90.000, 110.000)
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
                                        HeadingInterpolator.linear(
                                                follower().getHeading(),
                                                Math.toRadians(28.4)
                                        )
                                )
                        )
                )
                .build();



    }


    //TODO - figure out the max and min pos of servo! Does increasing bring hood up or down?
    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.37),
                Transfernf.INSTANCE.idle()
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
                transferUpFor(transferTime).afterTime(0.07)
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
                            baseState(-1240),
                            Transfernf.INSTANCE.hotdog()
                    ),
                    transferUpFor(1.5),


                    //SET 2
                    new ParallelGroup(
                            new SequentialGroup(
                                    new FollowPath(grabSet2),
                                    new Delay(0.14),
                                    new FollowPath(scoreSet2)

                            ),
                            baseState(-1240),

                            transferSequence(scoreSet2,1.3)
                    ),


                    //SET 3
                    new ParallelGroup(
                            new SequentialGroup(
                                    new FollowPath(grabSet3),
                                    new Delay(0.7),
                                    new FollowPath(scoreSet3)

                            ),
                            baseState(-1240),

                            transferSequence(scoreSet3,1.3)
                    ),

                    //SET 4
                    new ParallelGroup(
                            new SequentialGroup(
                                    new FollowPath(grabSet4),
                                    new Delay(0.2),
                                    new FollowPath(scoreSet4)

                            ),
                            baseState(-1240),

                            transferSequence(scoreSet4,1.3)
                    ),



                    //SET 5 - Human Player
                    new ParallelGroup(
                            new SequentialGroup(
                                    new FollowPath(grabHp),
                                    new Delay(0.35),
                                    new FollowPath(scoreHp)
                            ),
                            baseState(-1200,0.37),

                            transferSequenceDistance(scoreHp,5, 2.6)
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