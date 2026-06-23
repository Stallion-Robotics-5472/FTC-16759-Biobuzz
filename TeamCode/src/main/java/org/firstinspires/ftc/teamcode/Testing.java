package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "TestingTele", group = "Buzz")
//@Disabled
public class Testing extends LinearOpMode {
    IntakeSubsystem intakeSS;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeSS = new IntakeSubsystem(gamepad1,hardwareMap,telemetry);

        waitForStart();

        while (opModeIsActive()) {
            intakeSS.runIntake();

            telemetry.update();
        }
    }
}
