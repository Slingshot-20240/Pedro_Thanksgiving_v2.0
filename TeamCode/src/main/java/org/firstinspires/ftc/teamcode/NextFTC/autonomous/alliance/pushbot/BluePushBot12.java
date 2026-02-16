package org.firstinspires.ftc.teamcode.NextFTC.autonomous.alliance.pushbot;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

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
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Blue Pushbot")
public class BluePushBot12 extends NextFTCOpMode {

    public BluePushBot12() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE,
                        Lednf.INSTANCE, asc.i
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;
    public PathChain set2;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain grabSet4;
    public PathChain scoreSet4;
    public PathChain park;
    public PathChain readyPush, pushBot;

    public Pose scorePose = new Pose(88,88).mirror();
    public Pose farScorePose = new Pose(86, 13).mirror();

    public static Pose startingPose = new Pose();

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)).mirror());

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119.000).mirror(), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-36), Math.toRadians(180-43))
                .build();

        set2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77).mirror(),
                                new Pose(126.5, 83.4).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-43), Math.toRadians(180-0))

                //gate 1
                .addPath(
                        new BezierCurve(
                                new Pose(126.5, 83.4).mirror(),
                                new Pose(112, 77.000).mirror(),
                                new Pose(129, 71.000).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-90))

                //score set 2
                .addPath(
                        new BezierLine(new Pose(130, 71.000).mirror(), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-90), Math.toRadians(180-43))
                .build();



        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.760, 55).mirror(),
                                new Pose(79.313, 57).mirror(),
                                new Pose(133, 54).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-43), Math.toRadians(180-0))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133, 54).mirror(),
                                new Pose(91.262, 56.240).mirror(),
                                new Pose(95.176, 82.815).mirror(),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-43))
                .build();


        grabSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88, 39).mirror(),
                                new Pose(82, 31).mirror(),
                                new Pose(132, 35).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-43), Math.toRadians(180-0))
                .build();

        readyPush = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.000, 35.000).mirror(),
                                new Pose(68, 55).mirror(),
                                new Pose(78, 13).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-90))

                .build();

        pushBot = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(78, 13).mirror(), new Pose(100, 13).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-90), Math.toRadians(180-90))
                .build();

        scoreSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100, 13).mirror(), farScorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-90), Math.toRadians(180-68))
                .build();

        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(farScorePose, new Pose(110,70).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-68), Math.toRadians(180-90))
                .build();


    }


    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.32)
        );

    }

    private Command autonomous() {
        return new ParallelGroup(
                //INTAKE ALWAYS ON
                Intakenf.INSTANCE.in(),

                //MAIN SEQUENCE
                new SequentialGroup(

                        new ParallelGroup(
                                f.i.follow(scorePreloads,"green"),
                                asc.i.baseState(-1240,0.32)
                        ),
                        asc.i.transferUpFor(2),


                        //SET 2
                        new ParallelGroup(
                                f.i.follow(set2,"green"),
                                asc.i.transferSequenceDistance(set2,2,0.5)

                        ),



                        //SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet3,"red"),
                                        f.i.follow(scoreSet3,"green")
                                ),
                                asc.i.transferSequenceDistance(scoreSet3,2,0.5)
                        ),

                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet4,"red"),
                                        f.i.follow(readyPush, "red"),
                                        f.i.follow(pushBot, "yellow"),
                                        f.i.follow(scoreSet4,"green"),
                                        new Delay(3),
                                        f.i.follow(park,"green")
                                ),

                                Shooternf.INSTANCE.setShooterVel(-1600),
                                asc.i.transferSequence(scoreSet4,2.5)
                        )

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