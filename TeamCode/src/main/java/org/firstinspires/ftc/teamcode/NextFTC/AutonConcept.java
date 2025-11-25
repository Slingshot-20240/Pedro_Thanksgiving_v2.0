package org.firstinspires.ftc.teamcode.NextFTC;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.AutonConceptSequences;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "CONCEPT Red Close Pedro")
public class AutonConcept extends NextFTCOpMode {
    public AutonConcept() {
        addComponents(
                new SubsystemComponent(
                        AutonConceptSequences.INSTANCE,
                        Intake.INSTANCE, Transfer.INSTANCE,
                        Shooter.INSTANCE, VariableHood.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)
        );
    }

    public PathChain scorePreloads;
    public PathChain prepareSet2;
    public PathChain grabSet2;
    public PathChain prepareGate;
    public PathChain hitGate;
    public PathChain scoreSet2;
    public PathChain prepareSet3;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain prepareSet4;
    public PathChain grabSet4;
    public PathChain scoreSet4;

    public void buildPaths() {
        scorePreloads = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.500, 118.300), new Pose(96.500, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

        prepareSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 96.000), new Pose(95.000, 83.800))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        grabSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(95.000, 83.800), new Pose(129.000, 83.800))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        prepareGate = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129.000, 83.800), new Pose(123.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        hitGate = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.000, 70.000), new Pose(130.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        scoreSet2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 70.000), new Pose(96.500, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        prepareSet3 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 96.000), new Pose(96.500, 59.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        grabSet3 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 59.500), new Pose(135.000, 59.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        //Backs out with a spline
        scoreSet3 = follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135.000, 59.500),
                                new Pose(91.262, 56.240),
                                new Pose(95.176, 82.815),
                                new Pose(96.500, 96.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        prepareSet4 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 96.000), new Pose(96.000, 35.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        grabSet4 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.000, 35.500), new Pose(135.000, 35.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scoreSet4 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(135.000, 35.500), new Pose(90.000, 110.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32))
                .build();

    }



    @Override
    public void onInit() {

        buildPaths();
        new SequentialGroup(
                VariableHood.INSTANCE.closeSide
        );
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(

            //--------Preloads--------\\
            //Shoot Preloads
            new ParallelGroup(
                    new FollowPath(scorePreloads),
                    //SHOOTER FIRST SET SPEED
                    AutonConceptSequences.INSTANCE.intakeSet(1060)
            ),
            AutonConceptSequences.INSTANCE.scoreSet(0.5,2.4),

            //--------Set 2--------\\
                    //Grab Set 2
            new ParallelGroup(
                    new SequentialGroup(
                            new FollowPath(prepareSet2),
                            new FollowPath(grabSet2),
                            new FollowPath(prepareGate),
                            new FollowPath(hitGate)
                    ),
                    //SHOOTER SECOND SET SPEED
                    AutonConceptSequences.INSTANCE.intakeSet(1080)
            ),

            //Shoot Set 2
            new SequentialGroup(
                    new FollowPath(scoreSet2),
                    AutonConceptSequences.INSTANCE.scoreSet(0,2.6)
            ),

            //--------Set 3--------\\
                    //Grab Set 3
            new ParallelGroup(
                    new SequentialGroup(
                            new FollowPath(prepareSet3),
                            new FollowPath(grabSet3)
                    ),
                    //SHOOTER THIRD SET SPEED
                    AutonConceptSequences.INSTANCE.intakeSet(1080)
            ),

            //Shoot Set 3
            new SequentialGroup(
                    new FollowPath(scoreSet3),
                    AutonConceptSequences.INSTANCE.scoreSet(0,2.6)
            ),

            //--------Set 4--------\\
            //Grab Set 4
            new ParallelGroup(
                    new SequentialGroup(
                            new FollowPath(prepareSet4),
                            new FollowPath(grabSet4),
                            VariableHood.INSTANCE.setHoodPos(0.52)
                    ),
                    //SHOOTER THIRD SET SPEED
                    AutonConceptSequences.INSTANCE.intakeSet(990)
            ),

            //Shoot Set 4
            new SequentialGroup(
                    new FollowPath(scoreSet4),
                    AutonConceptSequences.INSTANCE.scoreSet(0,2.6)
            )

        );
    }

}