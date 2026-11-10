package org.firstinspires.ftc.teamcode.subsystems.scrimbot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.command.Command;

import java.util.function.BooleanSupplier;

public class ShootCommand extends Command {
    ScrimIntakeSubsys intake;
    ScrimShooterSubsys shooter;
    double open;
    double closed;
    double targetRPM = 1000;
    BooleanSupplier requested;
    BooleanSupplier trimDown;
    BooleanSupplier trimUp;
    boolean lastTrimDown = false;
    boolean lastTrimUp = false;
    ElapsedTime timer = new ElapsedTime();

    public ShootCommand(ScrimIntakeSubsys intake,
                        ScrimShooterSubsys shooter,
                        double open,
                        double closed,
                        BooleanSupplier requested,
                        BooleanSupplier trimDown,
                        BooleanSupplier trimUp) {
        this.intake = intake;
        this.shooter = shooter;
        this.open = open;
        this.closed = closed;
        this.requested = requested;
        this.trimDown = trimDown;
        this.trimUp = trimUp;

        addRequirements(intake, shooter);
        withName("Shoot");
    }

    @Override
    public void initialize() {
        shooter.setGate(closed);
        shooter.setTPS(0);
        timer.reset();
    }

    @Override
    public void execute() {
        shooter.setTPS(targetRPM);
        trim();

        if (timer.milliseconds() > 3000) {
            shooter.setGate(open);
            intake.setIntake(1);
        } else {
            shooter.setGate(closed);
            intake.setIntake(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setGate(closed);
        shooter.setTPS(0);
    }

    private void trim() {
        if (trimUp.getAsBoolean() && !lastTrimUp) {
            targetRPM += 50;
        } else if (trimDown.getAsBoolean() && !lastTrimDown) {
            targetRPM -= 50;
        }
    }
}