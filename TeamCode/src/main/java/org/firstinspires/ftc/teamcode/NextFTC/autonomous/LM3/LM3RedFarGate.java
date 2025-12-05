package org.firstinspires.ftc.teamcode.NextFTC.autonomous.LM3;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
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

@Autonomous(name = "12 Red Far Side Gate")
public class LM3RedFarGate extends NextFTCOpMode {

    public LM3RedFarGate() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads, grabSet2, gate, scoreSet2, grabSet3, scoreSet3, prepareHp, backOutHp, grabHp, scoreHp, park;
    Pose scorePose = new Pose(88, 17);

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(88, 8.2, Math.toRadians(90)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(88, 8), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(84, 67.5), new Pose(135.5, 58.5)))
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                .build();

        gate = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(new Pose(135.5, 58.5), new Pose(102, 55), new Pose(128, 67.75)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(new Pose(128, 67.75), new Pose(85, 60.5), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                .build();

        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(89.75, 40), new Pose(135.5, 36)))
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(135.5, 36), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                .build();

        prepareHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(135.5, 36)))
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                .build();

        backOutHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(135.5, 36), new Pose(134, 36)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-80))
                .build();

        grabHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(134, 36), new Pose(134, 11)))
                .setLinearHeadingInterpolation(Math.toRadians(-80), Math.toRadians(-80))
                .build();

        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(134, 11), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(-80), Math.toRadians(70))
                .build();

        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(119, 70)))
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(90))
                .build();
    }

    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.34)
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
                Hoodnf.INSTANCE.setHoodPos(0.3)
        );
    }

    private Command autonomous() {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),
                new SequentialGroup(
                        // Preloads
                        new ParallelGroup(
                                new FollowPath(scorePreloads, true),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
                        new Delay(0.4),
                        transferUpFor(2.2),

                        // SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet2),
                                        new FollowPath(scoreSet2, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
                        new Delay(0.2),
                        transferUpFor(2.6),

                        // SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet3),
                                        new FollowPath(scoreSet3, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
                        new Delay(0.2),
                        transferUpFor(2.6),

                        // Human Player
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(prepareHp),
                                        new FollowPath(backOutHp),
                                        new FollowPath(scoreHp, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
                        new Delay(0.2),
                        transferUpFor(4)
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
