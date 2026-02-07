package org.firstinspires.ftc.teamcode.NextFTC.autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Variable;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Disabled
@Autonomous(name = "SplineAutonTest")
public class SplineAutonTest extends NextFTCOpMode {
    public SplineAutonTest() {

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
    public PathChain hitGate;
    public PathChain scoreSet2;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain grabSet4;
    public PathChain scoreSet4;


    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(126, 118, Math.toRadians(36)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.200, 119.000), new Pose(87.000, 87.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.000, 87.000),
                                new Pose(92.292, 80.549),
                                new Pose(128, 83.800)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        hitGate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(128, 83.800),
                                new Pose(113.000, 77.000),
                                new Pose(128, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(128, 71.000), new Pose(87.000, 87.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.000, 87.000),
                                new Pose(87.760, 64),
                                new Pose(79.313, 55),
                                new Pose(133, 57)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133, 57),
                                new Pose(91.262, 56.240),
                                new Pose(95.176, 82.815),
                                new Pose(87,87)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();


        grabSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87, 87),
                                new Pose(92.498, 40.584),
                                new Pose(87.142, 33.579),
                                new Pose(137, 36)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scoreSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(137.000, 36.000), new Pose(90.000, 110.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();

    }

    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.45)
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
                Hoodnf.INSTANCE.setHoodPos(0.61)
        );
    }

    private Command autonomous() {
        return new ParallelGroup(
                //INTAKE ALWAYS ON
                Intakenf.INSTANCE.in(),

                //MAIN SEQUENCE
                new SequentialGroup(

                        new ParallelGroup(
                                new FollowPath(scorePreloads),

                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1255)
                        ),
                        //Spin up time
                        new Delay(1),
                        transferUpFor(2.2),



                        //SET 2
                        new ParallelGroup(
                                new FollowPath(grabSet2),

                                baseState(),
                                //SET 2 Shooter vel
                                Shooternf.INSTANCE.setShooterVel(-1255)
                        ),
                        new SequentialGroup(
                                new FollowPath(hitGate),
                                new Delay(1.5),
                                new FollowPath(scoreSet2)
                        ),
                        transferUpFor(2.5),


                        //SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet3),
                                        new FollowPath(scoreSet3)
                                ),
                                Transfernf.INSTANCE.hotdog(),
                                //SET 3 Shooter vel
                                Shooternf.INSTANCE.setShooterVel(-1255)
                        ),
                        transferUpFor(2.5),

                        //SET 4
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet4),
                                        new FollowPath(scoreSet4)
                                ),
                                Transfernf.INSTANCE.hotdog(),
                                //SET 4 Shooter vel
                                Shooternf.INSTANCE.setShooterVel(-1190)
                        ),
                        transferUpFor(2.5)


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