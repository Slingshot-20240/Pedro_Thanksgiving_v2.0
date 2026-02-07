package org.firstinspires.ftc.teamcode.NextFTC.autonomous.LC.far;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.MTransfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Disabled
@Autonomous(name = "Red Far 12 Gate LC")
public class RedFar12Gate extends NextFTCOpMode {

    public RedFar12Gate() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, MTransfernf.INSTANCE,
                        Lednf.INSTANCE,f.i
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }


    public PathChain scorePreloads, grabSet2, scoreSet2, grabSet3, scoreSet3, prepareHp, grabHp, scoreHp, park;

    public PathChain gate;

    Pose scorePose = new Pose(86, 15);

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(88, 8.2, Math.toRadians(90)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(88, 8.2), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68.4))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(84, 65), new Pose(132, 57)))
                .setLinearHeadingInterpolation(Math.toRadians(68.4), Math.toRadians(0))
                .build();


        gate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132, 57.000),
                                new Pose(89, 66),
                                new Pose(130, 69)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(new Pose(130, 69), new Pose(86, 60.5), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68.4))
                .build();

        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(86, 36), new Pose(134, 36)))
                .setLinearHeadingInterpolation(Math.toRadians(68.4), Math.toRadians(0))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(134, 36), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68.4))
                .build();


        prepareHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(132, 50)))
                .setLinearHeadingInterpolation(Math.toRadians(68.4), Math.toRadians(300))
                .build();

        grabHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(132, 50), new Pose(132, 11)))
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(290))
                .build();

        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(132, 11), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(68.4))
                .build();

        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(120, 68.4)))
                .setLinearHeadingInterpolation(Math.toRadians(68.4), Math.toRadians(90))
                .build();
    }

    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.34),
                MTransfernf.INSTANCE.idle()
        );
    }

    private Command transferUpFor(double time) {
        return new ParallelGroup(
                //MTransfernf.INSTANCE.on(),
                MTransfernf.INSTANCE.on(),
                Lednf.INSTANCE.yellow,
                new Delay(time)
        );
    }

    private Command baseState() {
        return new ParallelGroup(
                MTransfernf.INSTANCE.hotdog(),
                Hoodnf.INSTANCE.setHoodPos(0.3)
        );
    }

    private Command autonomous() {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),

                new SequentialGroup(
                        // Preloads
                        new ParallelGroup(
                                f.i.follow(scorePreloads, "green"),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1540)
                        ),
                        new Delay(0.5),
                        transferUpFor(2.2),

                        // SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet2, "red"),
                                        f.i.follow(gate,"yellow"),
                                        f.i.follow(scoreSet2, "green")
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1540)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),


                        // SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet3, "red"),
                                        f.i.follow(scoreSet3, "green")
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1540)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),

                        // SET 4
                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(prepareHp, "red"),
                                        new Delay(0.3),
                                        f.i.follow(grabHp, "red"),
                                        f.i.follow(scoreHp, "green")
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1540)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),

                        f.i.follow(park)



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
}
