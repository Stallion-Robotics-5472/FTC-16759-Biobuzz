package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.lib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.lib.command.Commands;
import org.firstinspires.ftc.teamcode.lib.command.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.TestCommandSubsystem;

@TeleOp(name = "CommandTest")
public class CommandTest extends CommandOpMode {
    TestCommandSubsystem testSS;

    @Override
    public void configure() {
        testSS = new TestCommandSubsystem(hardwareMap);

        register(testSS);

        setDefaultCommand(testSS, whenPressed(() -> gamepad1.a,
                Commands.runOnce(() -> testSS.setPower()));
    }

    @Override
    public void onStart() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void onStop() {

    }
}
