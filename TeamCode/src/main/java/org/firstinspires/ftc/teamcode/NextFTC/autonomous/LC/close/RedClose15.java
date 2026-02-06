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
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "Red 15 Close FIXED")
public class RedClose15 extends NextFTCOpMode {
    public RedClose15() {
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
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(43))
//                .setTangentHeadingInterpolation().setReversed()
                .build();



        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292,77),
                                new Pose(127, 80)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();


        scoreSet2 = follower().pathBuilder()
                //Score Set 2
                .addPath(
                        new BezierLine(new Pose(128, 80), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                //was 0
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
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
//                .setTangentHeadingInterpolation()



                //Gate 2
                .addPath(
                        new BezierCurve(
                                new Pose(132.4, 54),
                                new Pose(100, 60),
                                new Pose(130, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))



                .build();

        scoreSet3 = follower().pathBuilder()
                //Score set 3
                .addPath(
                        new BezierLine(new Pose(130, 71), scorePose)
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
                                new Pose(84, 41),
                                new Pose(80, 37),
                                new Pose(131, 34)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
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
                                        HeadingInterpolator.constant(Math.toRadians(43))
                                )
                        )
                )
                .build();




        grabHp = follower()
                .pathBuilder()

                //grab
                .addPath(
                        new BezierCurve(scorePose, new Pose(120,25), new Pose(126,13))
                )
                .setTangentHeadingInterpolation()


                //assure backup
                .addPath(
                        new BezierLine(new Pose(126,13), new Pose(119,15))
                )
                .setTangentHeadingInterpolation().setReversed()

                .addPath(
                        new BezierLine(new Pose(119,15), new Pose(126,13))
                )
                .setTangentHeadingInterpolation()

                .build();



        scoreHp = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126, 13),
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
                Hoodnf.INSTANCE.setHoodPos(0.35),
                MTransfernf.INSTANCE.idle()
        );

    }


    private Command autonomous() {
        return new SequentialGroup(


                    new ParallelGroup(
                            f.i.follow(scorePreloads,"green"),
                            asc.i.baseState(-1270)
                    ),
                    asc.i.transferUpFor(1),

//TODO - TRY THIS IF ABOVE DOESN'T WORK
//                    new SequentialGroup(
//                            asc.i.baseState(-1260),
//                            f.i.follow(scorePreloads,"green"),
//                            asc.i.transferUpFor(1)
//                    ),





                //SET 2
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabSet2, "red")
                                    ),
                                    f.i.follow(scoreSet2,"green")

                            ),
                            asc.i.baseState(-1260),

                            asc.i.transferSequenceDistance(scoreSet2,1.1,2)
                    ),


                    //SET 3
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabSet3, "red")
                                    ),
                                    f.i.follow(scoreSet3,"green")

                            ),
                            asc.i.baseState(-1260),

                            asc.i.transferSequenceDistance(scoreSet3,1.1,2)
                    ),

                    //SET 4
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabSet4, "red")
                                    ),
                                    f.i.follow(scoreSet4,"green")

                            ),
                            asc.i.baseState(-1260),

                            asc.i.transferSequenceDistance(scoreSet4,1.1,2)
                    ),



                    //SET 5 - Human Player
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabHp, "red")
                                    ),
                                    f.i.follow(scoreHp,"green")
                            ),
                            asc.i.baseState(-1200,0.37),

                            //asc.i.transferSequenceDistance(scoreHp,5, 2.6),
                            asc.i.transferSequenceDistance(scoreHp,5,5)
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