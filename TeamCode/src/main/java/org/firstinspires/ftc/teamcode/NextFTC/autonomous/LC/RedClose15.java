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
@Autonomous(name = "15 ball red close")
public class RedClose15 extends NextFTCOpMode {
    public RedClose15() {
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
    public PathChain gate1;
    public PathChain scoreSet2;
    public PathChain grabSet3;
    public PathChain gate2;
    public PathChain scoreSet3;
    public PathChain grabSet4;
    public PathChain scoreSet4;
    public PathChain prepareHp;
    public PathChain grabHp;
    public PathChain scoreHp;

    public Pose scorePose = new Pose(88,88);
    public static double proximityThreshold = 6;

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.200, 119.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
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
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))


                .addPath(
                        new BezierCurve(
                                new Pose(126.5, 83.4),
                                new Pose(112, 77.000),
                                new Pose(129.2, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))



                .addPath(
                        new BezierLine(new Pose(129.2, 71.000), scorePose)
                )
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                //TODO - may need to tune this. if you make y lower, it should account for robot movement for shooting
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(new Pose(144,144))
                )
                .addParametricCallback(0.8, transferUpFor(2))
                .build();

//        gate1 = PedroComponent.follower()
//                .pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(126.5, 83.4),
//                                new Pose(112, 77.000),
//                                new Pose(129.2, 71.000)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
//                .build();

//        scoreSet2 = PedroComponent.follower()
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(129.2, 71.000), scorePose)
//                )
////                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
//                //TODO - may need to tune this. if you make y lower, it should account for robot movement for shooting
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(new Pose(144,144))
//                )
//
//                .build();





        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.760, 55.000),
                                new Pose(79.313, 57.000),
                                new Pose(132, 54.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))




                .addPath(
                        new BezierCurve(
                                new Pose(132, 54.000),
                                new Pose(120.000, 54.000),
                                new Pose(106.000, 74.000),
                                new Pose(129.2, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))



                .build();

//        gate2 = PedroComponent.follower()
//                .pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(132, 54.000),
//                                new Pose(120.000, 54.000),
//                                new Pose(106.000, 74.000),
//                                new Pose(129.2, 71.000)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
//                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129.2, 71.000), scorePose)
                )
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(new Pose(144,144))
                )
                .build();

        grabSet4 = PedroComponent.follower()
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
                .build();


        scoreSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(133, 35.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        prepareHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(127, 40.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(295))
                .build();

        grabHp = PedroComponent.follower()
                .pathBuilder()
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
                .setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(34))
                .build();

        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127, 11.000), new Pose(90.000, 110.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(34))
                .build();



    }


    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.4)
        );

    }

    private Command transferUpFor(double time) {
        return new ParallelGroup(
                Transfernf.INSTANCE.on(),
                new Delay(time)
        );
    }

    private Command baseState() {
        return new ParallelGroup(
                Transfernf.INSTANCE.hotdog(),
                Hoodnf.INSTANCE.setHoodPos(0.4)
        );
    }

    private Command autonomous() {

        //Preloads, normal
        //Set 2, face pose interpolator + parametric, t value, callback
        //Set 3, wait until + distance remaining
        //Set 4, rpm draw race group
        //Hp, normal
        return new ParallelGroup(
                //INTAKE ALWAYS ON
                Intakenf.INSTANCE.in(),

                //MAIN SEQUENCE
                new SequentialGroup(

                        //Preloads
                        new ParallelGroup(
                                new FollowPath(scorePreloads, true),

                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1200)
                        ),
                        //Spin up time
                        //new Delay(0.6),
                        //ATalign(),
                        transferUpFor(1.3),


                        //SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet2)
//                                        //gate
//                                        new FollowPath(gate1),
//                                        //new Delay(0.2),
//                                        new FollowPath(scoreSet2),
//
//                                   )
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1200)
                        ),

                        //SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet3),
                                        //gate
                                        new FollowPath(gate2),
                                        //new Delay(0.2),
                                        new ParallelGroup(
                                                new FollowPath(scoreSet3),
                                                new SequentialGroup(
                                                        //TODO - tune this value
                                                        new WaitUntil(() -> PedroComponent.follower().getDistanceRemaining() < proximityThreshold),
                                                        transferUpFor(1.3)
                                                )

                                        )
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1200)
                        ),
                        transferUpFor(1.3),

                        //SET 4
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet4),
                                        new FollowPath(scoreSet4, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1200)
                        ),
                        new ParallelRaceGroup(
                                new WaitUntil(() -> Shooternf.INSTANCE.rpmDraw(-1200,50)),
                                //TODO, tune race group value (max time for transfer)
                                transferUpFor(5)
                                //you can use .asDeadline() at end of command also
                        ),

                        //SET 5 - Human Player
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(prepareHp),
                                        new FollowPath(grabHp),
                                        new FollowPath(scoreHp, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1200)
                        ),
                        transferUpFor(5)

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