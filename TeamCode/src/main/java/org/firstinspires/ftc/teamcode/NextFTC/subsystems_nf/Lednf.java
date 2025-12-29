package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.ftc.ActiveOpMode;

public class Lednf implements Subsystem {
    public static final Lednf INSTANCE = new Lednf();
    private Lednf() { }

    public DigitalChannel ledBoard0;
    public DigitalChannel ledBoard1;


    public Command off = new InstantCommand(() -> {
        ledBoard0.setState(false);
        ledBoard1.setState(false);
    });

    public Command red = new InstantCommand(() -> {
        ledBoard0.setState(false);
        ledBoard1.setState(true);
    });

    public Command yellow = new InstantCommand(() -> {
        ledBoard0.setState(true);
        ledBoard1.setState(false);
    });

    public Command green = new InstantCommand(() -> {
        ledBoard0.setState(true);
        ledBoard1.setState(true);
    });

    @Override
    public void initialize() {
        ledBoard0 = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "ledBoard0");
        ledBoard0.setMode(DigitalChannel.Mode.OUTPUT);
        ledBoard1 = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "ledBoard1");
        ledBoard1.setMode(DigitalChannel.Mode.OUTPUT);
    }

    @Override
    public void periodic() {}
}
