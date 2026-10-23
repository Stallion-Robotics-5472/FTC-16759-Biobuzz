package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.lib.command.Commands;
import org.firstinspires.ftc.teamcode.lib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.lib.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.lib.command.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.TestCommandSubsystem;

@TeleOp(name = "CommandTest")
//@Disabled
public class CommandTest extends CommandOpMode {
    TestCommandSubsystem testSS;

    @Override
    public void configure() {
        testSS = new TestCommandSubsystem(hardwareMap);

        register(testSS);

        setDefaultCommand(testSS, new RunCommand(() -> {
            boolean requested = gamepad1.right_bumper;
            if (requested) {
                testSS.adjustPower();
            }
//            } else {
//                testSS.setPower(testSS.commandPower);
//            }
        }, testSS).withName("AdjustPower"));
    }

    @Override
    public void onStart() {
        telemetry.addLine("This is working");
    }

    @Override
    public void periodic() {
        testSS.addTelemetry(telemetry);
        telemetry.addData("currentCommand", getScheduler().getScheduledCommands());
    }

    @Override
    public void onStop() {
        testSS.stop();
    }
}
