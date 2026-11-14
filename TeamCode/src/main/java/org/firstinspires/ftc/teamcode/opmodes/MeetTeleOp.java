package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.lib.command.Commands;
import org.firstinspires.ftc.teamcode.lib.command.RunCommand;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.BatteryMonitor;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.localization.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scrimbot.ScrimIntakeSubsys;
import org.firstinspires.ftc.teamcode.subsystems.scrimbot.ScrimShooterSubsys;
import org.firstinspires.ftc.teamcode.subsystems.scrimbot.ShootCommand;

@TeleOp(name = "Meet 1 Teleop")
//@Disabled
public class MeetTeleOp extends CommandOpMode {
    ScrimIntakeSubsys intake;
    ScrimShooterSubsys shooter;
    DriveSubsystem drive;
    private static final Pose2d startPose = new Pose2d(0, 0, new Rotation2d(Math.toRadians(90)));
    private static final double slow = 0.4;
    private BatteryMonitor battery;
    private Alliance alliance = Alliance.RED;
    boolean absolute = false;
    double gateOpen = 0.21;
    double gateClosed = 0.27;
    double flowerUp = 0;
    double flowerDown = 0;
    boolean lastOptions = false;
    ShootCommand shootCommand;
    @Override
    public void configure() {
        intake = new ScrimIntakeSubsys(hardwareMap);
        shooter = new ScrimShooterSubsys(hardwareMap, telemetry);
        drive = new DriveSubsystem(hardwareMap);
        battery = BatteryMonitor.from(hardwareMap);

        register(intake, shooter, drive);

        drive.getLocalization().setStartingPose(startPose);

        setDefaultCommand(drive, new RunCommand(() -> {
            double scale = gamepad1.right_bumper ? slow : 1.0;

            whenPressed(() -> gamepad2.options,
                    Commands.runOnce(() -> drive.resetHeadingController()));

            if (absolute){
                drive.driveDriverRelative(
                        -gamepad1.left_stick_y * scale,
                        -gamepad1.left_stick_x * scale,
                        gamepad1.right_stick_x * scale,
                    alliance);}
            else {
                drive.driveRobotCentric(
                        gamepad1.left_stick_y * scale,
                        gamepad1.left_stick_x * scale,
                        gamepad1.right_stick_x * scale
                );
            }
        }, drive).withName("DriverControl"));

        setDefaultCommand(intake, new RunCommand(() -> {
            double intakeSpeed = gamepad1.left_trigger > 0.1 ? 1.0 : 0;
            intakeSpeed = gamepad2.left_trigger > 0.5 ? -1.0 : intakeSpeed;
            double flowerPos = gamepad2.options && !lastOptions ? flowerDown : flowerUp;
            lastOptions = gamepad2.options;
//            intake.setFlower(flowerPos);
            intake.setIntake(intakeSpeed);
        }, intake).withName("IntakeControl"));

        setDefaultCommand(shooter, new RunCommand(() -> {
            shooter.setTPS(0);
        }, shooter).withName("ShooterControl"));

        shootCommand = new ShootCommand(
                intake,
                shooter,
                gateOpen,
                gateClosed,
                () -> gamepad2.right_trigger > 0.25,
                () -> gamepad2.dpad_down,
                () -> gamepad2.dpad_up
        );
        whileHeld(() -> gamepad2.right_trigger > 0.25, shootCommand);
    }

    @Override
    public void onStart() {
        shooter.setGate(gateClosed);
//        intake.setFlower(flowerUp);
    }

    @Override
    public void periodic() {
        battery.update();

        if(opModeInInit()){
            whenPressed(() -> gamepad1.b,
                    Commands.runOnce(() -> alliance = Alliance.RED));
            whenPressed(() -> gamepad1.x,
                    Commands.runOnce(() -> alliance = Alliance.BLUE));

            whenPressed(() -> gamepad1.dpad_up && opModeInInit(),
                    Commands.runOnce(() -> absolute = false));
            whenPressed(() -> gamepad1.dpad_down && opModeInInit(),
                    Commands.runOnce(() -> absolute = true));


            telemetry.addLine("Select the Alliance: B for Red, X for Blue");
            telemetry.addData("Alliance", alliance);
            telemetry.addLine();
            telemetry.addLine("Select the Drive Mode");
            telemetry.addData("Drive", absolute ? "Field-Centric" : "Robot-Centric");
            telemetry.addLine();
            telemetry.addLine("Good Luck!");
        }
        shooter.addTelemetry(telemetry);
        drive.addTelemetry(telemetry);
        telemetry.addData("c", getScheduler().getScheduledCommands());
    }

    /*This is the stop*/
    @Override
    public void onStop(){
        intake.stop();
        shooter.stop();
        drive.stop();
        drive.getLocalization().stop();
    }
}
