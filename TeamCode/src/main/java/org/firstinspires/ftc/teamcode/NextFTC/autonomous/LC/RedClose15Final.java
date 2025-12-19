package org.firstinspires.ftc.teamcode.NextFTC.autonomous.LC;

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
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

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
    public PathChain set2;
    public PathChain set3;
    public PathChain set4;
    public PathChain grabHp;
    public PathChain scoreHp;

    public Pose scorePose = new Pose(88,88);
    public static double proximityThreshold = 5;

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();



        set2 = PedroComponent.follower()
                .pathBuilder()

            //Grab set 2
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77),
                                new Pose(126, 83.4)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.3,
                                        HeadingInterpolator.linear(Math.toRadians(45), Math.toRadians(0))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.3,
                                        1.0,
                                        HeadingInterpolator.linear(Math.toRadians(0), Math.toRadians(0))
                                )
                        )
                )


            //Gate 1
                .addPath(
                        new BezierCurve(
                                new Pose(126, 83.4),
                                new Pose(112, 77.000),
                                new Pose(129, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))


            //Score Set 2
                .addPath(
                        new BezierLine(new Pose(129, 71.000), scorePose)
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.3,
                                        HeadingInterpolator.linear(Math.toRadians(90), Math.toRadians(69))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.3,
                                        1.0,
                                        //TODO - tune the y value to make ball go in center due to newtons first law
                                        HeadingInterpolator.facingPoint(new Pose(144,144))
                                )
                        )
                )
                .build();



        set3 = PedroComponent.follower()
                .pathBuilder()

            //Grab set 3
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.760, 55.000),
                                new Pose(79.313, 57.000),
                                new Pose(132, 54.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))



                //Gate 2
                .addPath(
                        new BezierCurve(
                                new Pose(132, 54.000),
                                new Pose(120.000, 54.000),
                                new Pose(106.000, 74.000),
                                new Pose(129.2, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

            //Score set 3
                .addPath(
                        new BezierLine(new Pose(129.2, 71.000), scorePose)
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


        set4 = PedroComponent.follower()
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88, 39),
                                new Pose(82, 31),
                                new Pose(133, 35)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))


                .addPath(
                        new BezierLine(new Pose(133, 35.000), scorePose)
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.15,
                                        HeadingInterpolator.linear(Math.toRadians(0), Math.toRadians(311))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.15,
                                        0.7,
                                        HeadingInterpolator.linear(Math.toRadians(311), Math.toRadians(311))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1.0,
                                        //TODO - tune the y value to make ball go in center due to newtons first law
                                        HeadingInterpolator.facingPoint(new Pose(144,144))
                                )
                        )
                )
                .build();



        grabHp = PedroComponent.follower()
                .pathBuilder()

            //prepareHp
                .addPath(
                        new BezierLine(scorePose, new Pose(127, 40.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(295))

            //grabHp
                .addPath(
                        new BezierLine(new Pose(127, 40.000), new Pose(127, 11.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(295))
                .build();


        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127, 11.000), new Pose(90.000, 110.000))
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        //more towards the bottom means decrease the heading
                                        HeadingInterpolator.linear(Math.toRadians(295), Math.toRadians(295))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1.0,
                                        //TODO - tune the y value to make ball go in center due to newtons first law
                                        HeadingInterpolator.facingPoint(new Pose(144,144))
                                )
                        )
                )
                .build();

        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127, 11.000), new Pose(90.000, 110.000))
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        //more towards the bottom means decrease the heading
                                        HeadingInterpolator.linear(Math.toRadians(295), Math.toRadians(295))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1.0,
                                        //TODO - tune the y value to make ball go in center due to newtons first law
                                        HeadingInterpolator.facingPoint(new Pose(144,144))
                                )
                        )
                )
                .build();



    }


    //TODO - figure out the max and min pos of servo! Does increasing bring hood up or down?
    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.4),
                Hoodnf.INSTANCE.setHoodPos(0.4)
        );

    }

    private Command transferUpFor(double time) {
        return new SequentialGroup(
                Transfernf.INSTANCE.on(),
                new Delay(time),
                Transfernf.INSTANCE.hotdog()
        );

    }

    private Command transferSequence(double proximity, double transferTime) {
        return new SequentialGroup(
                Transfernf.INSTANCE.hotdog(),
                new WaitUntil(() -> PedroComponent.follower().getCurrentPathChain().lastPath().getDistanceRemaining() < proximity),
//                                        //OR time based
//                                        new Delay(0.8),
//                                        //OR parametric end
//                                        new WaitUntil(() -> PedroComponent.follower().atParametricEnd()),
                transferUpFor(transferTime)
        );
    }

    private Command baseState(double shooterVel) {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),
                Shooternf.INSTANCE.setShooterVel(shooterVel),
                Hoodnf.INSTANCE.setHoodPos(0.4)
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
                            baseState(-1200),

                            transferSequence(proximityThreshold,1.3)
                    ),


                    //SET 2
                    new ParallelGroup(
                            new FollowPath(set2),
                            baseState(-1200),

                            transferSequence(proximityThreshold,1.3)
                    ),


                    //SET 3
                    new ParallelGroup(
                            new FollowPath(set3),
                            baseState(-1200),

                            transferSequence(proximityThreshold,1.3)
                    ),

                    //SET 4
                    new ParallelGroup(
                            new FollowPath(set4),
                            baseState(-1200)

                            //transferSequence(proximityThreshold,1.3)
                    ),

                    new ParallelRaceGroup(
                            new WaitUntil(() -> Shooternf.INSTANCE.rpmDraw(-1200,50)),
                            //TODO, tune time (max time for transfer)
                            transferUpFor(4)
                            //you can use .asDeadline() at end of command also
                    ),

                    //SET 5 - Human Player
                    new ParallelGroup(
                            new SequentialGroup(
                                    new FollowPath(grabHp),
                                    new FollowPath(scoreHp)
                            ),
                            baseState(-1200),

                            transferSequence(proximityThreshold,5)
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