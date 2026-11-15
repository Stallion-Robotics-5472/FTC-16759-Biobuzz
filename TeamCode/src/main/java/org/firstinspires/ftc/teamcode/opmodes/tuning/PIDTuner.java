package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.util.Flywheel;

@TeleOp(name="PIDTuner")
public class PIDTuner extends LinearOpMode {
    DcMotor intake;
    double target = 1130;
    double kP = 250;
    double kD = 0.00005;
    double kF = 10;
    double stopBoostRange = 200;
    Flywheel shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Flywheel(hardwareMap, "shooter", target, kP, kD, kF, stopBoostRange);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

//        gate.setPosition(closed);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.right_trigger > 0.2){ shooter.setTPS(target); } else { shooter.setTPS(0); }
            if (gamepad1.left_trigger > 0.2){ intake.setPower(1.0); } else { intake.setPower(0); }

            if (gamepad1.dpadUpWasPressed())            //up
                { shooter.updateParameters(0.1,0,0,0); }
            if (gamepad1.dpadDownWasPressed())          //down
                { shooter.updateParameters(-0.1,0,0,0); }
            if (gamepad1.dpadRightWasPressed())         //right
                { shooter.updateParameters(0,0.01,0,0); }
            if (gamepad1.dpadLeftWasPressed())          //left
                { shooter.updateParameters(0,-0.01,0,0); }
            if (gamepad1.yWasPressed())                 //y
                { shooter.updateParameters(0,0,10,0); }
            if (gamepad1.aWasPressed())                 //a
                { shooter.updateParameters(0,0,-10,0); }
            if (gamepad1.bWasPressed())                 //b
                { shooter.updateParameters(0,0,0,10); }
            if (gamepad1.xWasPressed())                 //x
                { shooter.updateParameters(0,0,0,-10); }

            shooter.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
