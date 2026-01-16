package org.firstinspires.ftc.teamcode.NextFTC.misc;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.PathCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.DrawingNew;


import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Loginf;
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
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "zSimple Auton Test")
public class SimpleAutonTest extends NextFTCOpMode {
    public SimpleAutonTest() {
        addComponents(
                new SubsystemComponent(
                        f.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE,
                        Lednf.INSTANCE, Loginf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;

    public PathChain grabMiddleSet, scoreMiddleSet;
    public PathChain grabGate, scoreGate;
    public PathChain grabGate2, scoreGate2;
    public PathChain grabGate3, scoreGate3;
    public PathChain grabSet4, scoreSet4;

    public PathChain grabSet2, scoreSet2;

    public Pose scorePose = new Pose(87,87);
    //TODO - try different types, public private regular etc.
    double atErrorDeg;

    public void buildPaths() {

        follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = follower().pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(50))
                .build();



    }


    //TODO - figure out the max and min pos of servo! Does increasing bring hood up or down?
    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.33),
                Transfernf.INSTANCE.idle()
        );

    }


    private Command autonomous() {
        return new SequentialGroup(
                //new FollowPath(scorePreloads),

                new ParallelGroup(
                    Shooternf.INSTANCE.setShooterVel(-1240),
                    Transfernf.INSTANCE.on(),
                        Intakenf.INSTANCE.in()
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
        Shooternf.INSTANCE.enable();
        autonomous().schedule();


    }

    @Override
    public void onUpdate() {

        atErrorDeg = Loginf.INSTANCE.getATangle();
        telemetry.addData("atErrorDeg", atErrorDeg);
        telemetry.addData("ATangle", Loginf.INSTANCE.getATangle());
        telemetry.update();

    }


    @Override
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }


}