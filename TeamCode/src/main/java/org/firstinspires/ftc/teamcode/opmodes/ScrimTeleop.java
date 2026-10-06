package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.lib.command.Commands;
import org.firstinspires.ftc.teamcode.lib.command.RunCommand;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.BatteryMonitor;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scrimbot.ScrimIntakeSubsys;
import org.firstinspires.ftc.teamcode.subsystems.scrimbot.ScrimShooterSubsys;

@TeleOp(name = "OLD")
@Disabled
public class ScrimTeleop extends CommandOpMode {
    ScrimIntakeSubsys intake;
    ScrimShooterSubsys shooter;
    DriveSubsystem drive;
    private static final Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
    private static final double slow = 0.4;
    private BatteryMonitor battery;
    private Alliance alliance = Alliance.RED;
    int shooterVelocity = 0;
    int targetVelocity = 2000;
    @Override
    public void configure() {
        intake = new ScrimIntakeSubsys(hardwareMap);
        shooter = new ScrimShooterSubsys(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
        battery = BatteryMonitor.from(hardwareMap);

        register(intake, shooter, drive);

        drive.getLocalization().setStartingPose(startPose);

        whenPressed(() -> gamepad1.b && opModeInInit(),
                Commands.runOnce(() -> alliance = Alliance.RED));
        whenPressed(() -> gamepad1.x && opModeInInit(),
                Commands.runOnce(() -> alliance = Alliance.BLUE));

        setDefaultCommand(drive, new RunCommand(() -> {
            double scale = gamepad1.right_bumper ? slow : 1.0;
            drive.driveDriverRelative(
                    -gamepad1.left_stick_y * scale,
                    -gamepad1.left_stick_x * scale,
                    -gamepad1.right_stick_x * scale,
                    alliance);
        }, drive).withName("DriverControl"));

        setDefaultCommand(intake, new RunCommand(() -> {
            double intakeSpeed = gamepad1.left_trigger > 0.5 ? 1.0 : 0;
            intake.setIntake(intakeSpeed);
        }, intake).withName("IntakePower"));

        setDefaultCommand(shooter, new RunCommand(() -> {
            whenPressed(() -> gamepad1.dpad_up, Commands.runOnce(() -> targetVelocity += 50));
            whenPressed(() -> gamepad1.dpad_down, Commands.runOnce(() -> targetVelocity -= 50));
            shooterVelocity = gamepad1.right_trigger > 0.5 ? targetVelocity : 500;
            shooter.setShooter(shooterVelocity);
        }, shooter).withName("ShooterVelocity"));
    }

    @Override
    public void onStart() {

    }
}
