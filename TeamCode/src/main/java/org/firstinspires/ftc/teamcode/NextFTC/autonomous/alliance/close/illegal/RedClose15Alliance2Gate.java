package org.firstinspires.ftc.teamcode.NextFTC.autonomous.alliance.close.illegal;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.asc;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Loginf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Config
@Autonomous(name = "15 Final Red Close")
public class RedClose15Alliance2Gate extends NextFTCOpMode {
    public RedClose15Alliance2Gate() {
        addComponents(
                new SubsystemComponent(
                        f.i, asc.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE,
                        Lednf.INSTANCE, Loginf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain adjust;

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
//                .addParametricCallback(0.95, () -> asc.i.transferUpFor(2))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

        adjust = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(follower()::getPose, follower()::getPose)
                )

                .setHeadingInterpolation(
                        HeadingInterpolator.lazy(() -> {
                            double start = follower().getHeading();
                            double end = start + Math.toRadians(Loginf.INSTANCE.getATangle());

                            return HeadingInterpolator.linear(start, end);
                        })
                )

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


            //Gate 1
                .addPath(
                        new BezierCurve(
                                new Pose(127, 79),
                                new Pose(109, 76),
                                new Pose(128, 73)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();


        scoreSet2 = follower().pathBuilder()
                //Score Set 2
                .addPath(
                        new BezierLine(new Pose(128, 73), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
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
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                .setTangentHeadingInterpolation()



                //Gate 2
                .addPath(
                        new BezierCurve(
                                new Pose(132.4, 54.000),
                                new Pose(120.000, 54.000),
                                new Pose(113.000, 69.000),
                                new Pose(129, 63)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))



                .build();

        scoreSet3 = follower().pathBuilder()
                //Score set 3
                .addPath(
                        new BezierLine(new Pose(129, 63), scorePose)
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
                                        HeadingInterpolator.constant(Math.toRadians(45))
                                )
                        )
                )
                .build();




        grabHp = follower()
                .pathBuilder()

                //grab
                .addPath(
                        new BezierCurve(scorePose, new Pose(126,25), new Pose(130,13))
                )
                .setTangentHeadingInterpolation()


                //assure backup
                .addPath(
                        new BezierLine(new Pose(130,13), new Pose(122,15))
                )
                .setTangentHeadingInterpolation().setReversed()

                //assure pickup
                .addPath(
                        new BezierLine(new Pose(122,15), new Pose(129,13))
                )
                .setTangentHeadingInterpolation()

                .build();



        scoreHp = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(129, 13),
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
                Hoodnf.INSTANCE.setHoodPos(0.41),
                Transfernf.INSTANCE.idle()
        );

    }


    private Command autonomous() {
        return new SequentialGroup(


                    new ParallelGroup(
                            f.i.follow(scorePreloads, "green"),
                            asc.i.baseState(-1200),
                            Transfernf.INSTANCE.hotdog()
                    ),
                    asc.i.transferUpFor(1),


                    //SET 2
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabSet2, "red"),
                                            Transfernf.INSTANCE.pickup(grabSet2,2)
                                    ),
                                    f.i.follow(scoreSet2,"green")

                            ),
                            asc.i.baseState(-1200),

                            asc.i.transferSequence(scoreSet2,1)
                    ),


                    //SET 3
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabSet3, "red"),
                                            Transfernf.INSTANCE.pickup(grabSet3,2)
                                    ),
                                    f.i.follow(scoreSet3,"green")

                            ),
                            asc.i.baseState(-1240),

                            asc.i.transferSequence(scoreSet3,1)
                    ),

                    //SET 4
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabSet4, "red"),
                                            Transfernf.INSTANCE.pickup(grabSet4,2)
                                    ),
                                    f.i.follow(scoreSet4,"green")

                            ),
                            asc.i.baseState(-1240),

                            asc.i.transferSequence(scoreSet4,1)
                    ),



                    //SET 5 - Human Player
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabHp, "red"),
                                            Transfernf.INSTANCE.pickup(grabHp,2)
                                    ),
                                    f.i.follow(scoreHp,"green")
                            ),
                            asc.i.baseState(-1200,0.37),

                            //asc.i.transferSequenceDistance(scoreHp,5, 2.6),
                            asc.i.transferSequence(scoreHp,1)
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