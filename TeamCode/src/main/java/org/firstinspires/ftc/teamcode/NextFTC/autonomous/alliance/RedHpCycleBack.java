package org.firstinspires.ftc.teamcode.NextFTC.autonomous.alliance;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Autonomous(name = "0 Red Cycle HP Back")
public class RedHpCycleBack extends NextFTCOpMode {

    public RedHpCycleBack() {
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
    public PathChain backAssureSet2;
    public PathChain frontAssureSet2;
    public PathChain scoreSet2;

    public PathChain grabHp;
    public PathChain backAssure;
    public PathChain frontAssure;
    public PathChain scoreHp;
    public PathChain park;

    Pose scorePose = new Pose(88, 17);

    public void buildPaths() {
        follower().setStartingPose(new Pose(88, 8.2, Math.toRadians(90)));

        scorePreloads = follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(88, 8.2), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(69.6))
                .build();

        //SET 2 SPECIFIC ONES
        grabSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(125, 25),
                                new Pose(134.000, 11.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        backAssureSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 11.000), new Pose(125.000, 11))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        frontAssureSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.000, 11), new Pose(134.000, 11))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                .addPath(
                        new BezierLine(new Pose(125.000, 11), new Pose(134.000, 8.2))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(300))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 8.2), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(69.6))
                .build();


        //REGULAR ONES
        grabHp = follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(128, 18),
                                new Pose(134.000, 15)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        backAssure = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 15), new Pose(125.000, 18))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        frontAssure = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.000, 18), new Pose(134.000, 20))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreHp = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 20), new Pose(88.000, 17.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69.6))
                .build();


        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(120, 30)))
                .setLinearHeadingInterpolation(Math.toRadians(69.6), Math.toRadians(90))
                .build();

    }

    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.34),
                Transfernf.INSTANCE.idle()
        );
    }

    private Command transferUpFor(double time) {
        return new ParallelGroup(
                //Transfernf.INSTANCE.on(),
                Transfernf.INSTANCE.stepOn(),
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
                                Shooternf.INSTANCE.setShooterVel(-1540)
                        ),
                        new Delay(0.5),
                        transferUpFor(2.2),

                        // SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet2),
                                        new Delay(0.2),
                                        new FollowPath(backAssureSet2),
                                        new FollowPath(frontAssureSet2, true,0.7),
                                        new Delay(0.2),
                                        new FollowPath(scoreSet2)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),


                        // SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabHp),
                                        new Delay(0.2),
                                        new FollowPath(backAssure),
                                        new FollowPath(frontAssure, true,0.7),
                                        new Delay(0.2),
                                        new FollowPath(scoreHp)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),

                        // SET 4
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabHp),
                                        new Delay(0.2),
                                        new FollowPath(backAssure),
                                        new FollowPath(frontAssure, true,0.7),
                                        new Delay(0.2),
                                        new FollowPath(scoreHp)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),

                        // SET 5
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabHp),
                                        new Delay(0.2),
                                        new FollowPath(backAssure),
                                        new FollowPath(frontAssure, true,0.7),
                                        new Delay(0.2),
                                        new FollowPath(scoreHp)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.1),
                        transferUpFor(1.5),


                        new FollowPath(park)
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
