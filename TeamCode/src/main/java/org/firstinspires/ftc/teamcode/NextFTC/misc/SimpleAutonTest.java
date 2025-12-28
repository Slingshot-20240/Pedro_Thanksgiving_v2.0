package org.firstinspires.ftc.teamcode.NextFTC.misc;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.vision.logi;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "15 Logi Test")
public class SimpleAutonTest extends NextFTCOpMode {
    public SimpleAutonTest() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    public logi cam;
    double bearing;

    public PathChain scorePreloads;

    public PathChain grabMiddleSet, scoreMiddleSet;
    public PathChain grabGate, scoreGate;
    public PathChain grabGate2, scoreGate2;
    public PathChain grabGate3, scoreGate3;
    public PathChain grabSet4, scoreSet4;

    public PathChain grabSet2, scoreSet2;

    public Pose scorePose = new Pose(87,87);
    //TODO - try different types, public private regular etc.

    public void buildPaths() {
        double gateHeading = 18;

        PedroComponent.follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(50))
                .build();

        grabMiddleSet = PedroComponent.follower().pathBuilder()

                //Grab Middle Set
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.760, 55.000),
                                new Pose(79.313, 57.000),
                                new Pose(132.4, 54.000)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(

                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.linear(Math.toRadians(43.5), Math.toRadians(0))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1.0,
                                        HeadingInterpolator.constant(0)
                                )
                        )
                )


                .build();

        scoreMiddleSet = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.4, 54.000),
                                new Pose(97.500, 60.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43.5))
                .build();


        //----------- GATES ------------\\
        grabGate = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(82.000, 58.000),
                                new Pose(131.5, 60.2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(gateHeading))
                .build();

        scoreGate = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(131.5, 60.2),
                                new Pose(94.000, 64.000),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(43.5))
                .build();



        //----------- Done ------------\\


        grabSet4 = follower()
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88, 39),
                                new Pose(82, 31),
                                new Pose(131, 33.3)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(0))
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
                                        HeadingInterpolator.constant(Math.toRadians(42))
                                )
                        )
                )
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77),
                                new Pose(126, 80)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(0))


                .build();


        scoreSet2 = PedroComponent.follower().pathBuilder()
                //Score Set 2
                .addPath(
                        new BezierLine(new Pose(126, 80), new Pose(90,110))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(28.4))
                .build();


    }


    //TODO - figure out the max and min pos of servo! Does increasing bring hood up or down?
    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.33),
                Transfernf.INSTANCE.idle()
        );

    }

    private Command transferUpFor(double time) {
        return new ParallelGroup(
                Transfernf.INSTANCE.stepOn(),
                new Delay(time)
        );
    }

    private Command transferSequence(PathChain pathChain, double transferTime) {
        return new SequentialGroup(
                Transfernf.INSTANCE.hotdog(),
                new WaitUntil(() -> pathChain.lastPath().isAtParametricEnd()),
                transferUpFor(transferTime)
        );
    }

    private Command transferSequenceDistance(PathChain pathChain, double transferTime, double proximity) {
        return new SequentialGroup(
                Transfernf.INSTANCE.hotdog(),
                new WaitUntil(() -> pathChain.lastPath().getDistanceRemaining() < proximity),
                transferUpFor(transferTime)
        );
    }

    private Command baseState(double shooterVel) {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),
                Shooternf.INSTANCE.setShooterVel(shooterVel),
                Hoodnf.INSTANCE.setHoodPos(0.33)
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


                new ParallelGroup(
                        new FollowPath(scorePreloads),
                        baseState(-1240),
                        Transfernf.INSTANCE.hotdog()
                ),
                new Delay(1)


        );
    }
    private Command vision() {
        return new SequentialGroup(

                // Grab the angle at runtime
                new WaitUntil(() -> {
                    bearing = cam.getATangle();
                    return true;
                }),

                // Now use it
                new TurnBy(Angle.fromDeg(bearing)),

                transferUpFor(2)
        );
    }

    @Override
    public void onInit() {
        cam = new logi(hardwareMap);

        buildPaths();
        init_bot().schedule();
        Shooternf.INSTANCE.disable();
    }

    @Override
    public void onStartButtonPressed() {
        cam.enableAT();
        autonomous().schedule();
        Shooternf.INSTANCE.enable();
        //vision().schedule();

    }

    @Override
    public void onUpdate() {
        telemetry.addData("ATangle", cam.getATangle());
        telemetry.update();
        bearing = cam.getATangle();
        new SequentialGroup(
                new TurnBy(Angle.fromDeg(cam.getATangle()))
        ).schedule();
    }


    @Override
    public void onStop() {
        PoseStorage.startingPose = PedroComponent.follower().getPose();
    }


}