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
@Autonomous(name = "Blue 15 Close FIXED")
public class BlueClose15 extends NextFTCOpMode {

    private double mx(double x) { return 144 - x; }

    private double mh(double deg) {
        if (deg == 0) return 180;
        if (deg == 180) return 0;
        if (deg == 90 || deg == 268) return deg;
        return 180 - deg;
    }

    public BlueClose15() {
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

    public Pose scorePose = new Pose(55,89);

    public void buildPaths() {
        follower().setStartingPose(new Pose(17.8, 119, Math.toRadians(144)));

        scorePreloads = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17.8, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
                .build();



        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(51.708, 77),
                                new Pose(18, 80)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))


                .addPath(
                        new BezierCurve(
                                new Pose(18, 80),
                                new Pose(32, 77.000),
                                new Pose(15, 74)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();


        scoreSet2 = follower().pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15, 74), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();





        grabSet3 = follower()
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(56.240, 55.000),
                                new Pose(64.687, 57.000),
                                new Pose(11.6, 54.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))



                .addPath(
                        new BezierCurve(
                                new Pose(mx(126.5), 83.4),
                                new Pose(mx(112), 77.000),
                                new Pose(mx(130), 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))



                .build();

        scoreSet3 = follower().pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15, 66), scorePose)
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.linear(Math.toRadians(180), Math.toRadians(180))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1.0,
                                        HeadingInterpolator.facingPoint(new Pose(0,144))
                                )
                        )
                )
                .build();


        grabSet4 = follower()
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(56, 39),
                                new Pose(62, 31),
                                new Pose(13, 33.3)
                        )
                )
                .setTangentHeadingInterpolation()


                .build();


        scoreSet4 = follower()
                .pathBuilder()

                .addPath(
                        new BezierLine(new Pose(13, 33.3), scorePose)
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
                                        HeadingInterpolator.constant(Math.toRadians(138))
                                )
                        )
                )
                .build();




        grabHp = follower()
                .pathBuilder()

                .addPath(
                        new BezierLine(scorePose, new Pose(17.5,13))
                )
                .setTangentHeadingInterpolation()


                .addPath(
                        new BezierLine(new Pose(17.5,13), new Pose(22,15))
                )
                .setTangentHeadingInterpolation().setReversed()


                .addPath(
                        new BezierLine(new Pose(22,15), new Pose(17.5,13))
                )
                .setTangentHeadingInterpolation()

                .build();



        scoreHp = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(17.5, 13),
                                new Pose(54.000, 110.000)
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
                                                Math.toRadians(180),
                                                Math.toRadians(151.6)
                                        )
                                )
                        )
                )
                .build();



    }


    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.37),
                MTransfernf.INSTANCE.idle()
        );

    }


    private Command autonomous() {
        return new SequentialGroup(


                new ParallelGroup(
                        f.i.follow(scorePreloads, "green"),
                        asc.i.baseState(-1240),
                        MTransfernf.INSTANCE.hotdog()
                ),
                asc.i.transferUpFor(0.6),


                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(grabSet2, "red"),
                                new Delay(0.1),
                                f.i.follow(scoreSet2,"green")

                        ),
                        asc.i.baseState(-1240),

                        asc.i.transferSequence(scoreSet2,0.6)
                ),


                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(grabSet3,"red"),
                                new Delay(0.1),
                                f.i.follow(scoreSet3,"green")

                        ),
                        asc.i.baseState(-1240),

                        asc.i.transferSequence(scoreSet3,0.6)
                ),


                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(grabSet4,"red"),
                                new Delay(0.1),
                                f.i.follow(scoreSet4,"green")

                        ),
                        asc.i.baseState(-1240),

                        asc.i.transferSequence(scoreSet4,0.6)
                ),



                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(grabHp,"red"),
                                new Delay(0.1),
                                f.i.follow(scoreHp,"green")
                        ),
                        asc.i.baseState(-1200,0.37),

                        asc.i.transferSequenceDistance(scoreHp,5, 2.6)
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
