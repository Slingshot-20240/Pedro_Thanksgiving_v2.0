package org.firstinspires.ftc.teamcode.NextFTC.autonomous.alliance;

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
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "blue pushbot")
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

    private double mx(double x) { return 144 - x; }

    private double mh(double deg) {
        if (deg == 0) return 180;
        if (deg == 180) return 0;
        if (deg == 90 || deg == 268) return deg;
        return 180 - deg;
    }

    public PathChain scorePreloads;
    public PathChain set2;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain grabSet4;
    public PathChain scoreSet4;
    public PathChain pushBot;

    public Pose scorePose = new Pose(mx(88),88);
    public Pose farScorePose = new Pose(mx(88), 17);

    public static Pose startingPose = new Pose();

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(mx(126.2), 119, Math.toRadians(mh(36))));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mx(126.2), 119.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(36)), Math.toRadians(mh(43)))
                .build();

        set2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(mx(92.292), 77),
                                new Pose(mx(126.5), 83.4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(43)), Math.toRadians(mh(0)))

                //gate 1
                .addPath(
                        new BezierCurve(
                                new Pose(mx(126.5), 83.4),
                                new Pose(mx(112), 77.000),
                                new Pose(mx(130), 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(0)), Math.toRadians(mh(90)))

                //score set 2
                .addPath(
                        new BezierLine(new Pose(mx(130), 71.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(90)), Math.toRadians(mh(43)))
                .build();



        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(mx(87.760), 55),
                                new Pose(mx(79.313), 57),
                                new Pose(mx(133), 54)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(43)), Math.toRadians(mh(0)))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mx(133), 54),
                                new Pose(mx(91.262), 56.240),
                                new Pose(mx(95.176), 82.815),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(90)), Math.toRadians(mh(43)))
                .build();


        grabSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(mx(88), 39),
                                new Pose(mx(82), 31),
                                new Pose(mx(132), 35)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(43)), Math.toRadians(mh(0)))
                .build();

        scoreSet4 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(132.000), 35.000),
                                new Pose(mx(79), 55),
                                new Pose(mx(82), 16)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(mh(0)), Math.toRadians(mh(67)))

                .build();

        pushBot = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mx(82), 16), new Pose(mx(100), 16))
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(67)), Math.toRadians(mh(90)))
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
                                new SequentialGroup(
//                                new ParallelGroup(
                                        f.i.follow(set2,"green")
                                        //Transfernf.INSTANCE.pickup(grabSet2,2)
//                                ),
                                ),
                                asc.i.baseState(-1240),
                                asc.i.transferSequenceDistance(set2,2,2)

                        ),



                        //SET 3
                        new ParallelGroup(
                                new SequentialGroup(
//                                new ParallelGroup(
                                        f.i.follow(grabSet3,"red"),
//                                        Transfernf.INSTANCE.pickup(grabSet3,2)
//                                        Transfernf.INSTANCE.hotdog()
//                                ),
                                        f.i.follow(scoreSet3,"green", true)

                                ),
                                asc.i.baseState(-1240),
                                asc.i.transferSequenceDistance(scoreSet3,2,2)
                        ),

                        new ParallelGroup(
                                new SequentialGroup(
                                        f.i.follow(grabSet4),
                                        f.i.follow(scoreSet4)
                                ),
                                asc.i.baseState(-1525, 0.33),
                                asc.i.transferSequenceDistance(scoreSet4,2.5,1)

                        ),

                        f.i.follow(pushBot)


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