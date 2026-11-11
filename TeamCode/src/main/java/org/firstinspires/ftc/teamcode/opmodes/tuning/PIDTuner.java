package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="PIDTuner")
public class PIDTuner extends LinearOpMode {
    DcMotorEx shooter;
    DcMotor intake;
//    Servo gate;
    double closed = 0.27;
    double open = 0.21;
    double target = 1110;
    double kP = 550;
    double kD = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        gate = hardwareMap.get(Servo.class, "gate");
//        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, 0, kD, 0));

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

//        gate.setPosition(closed);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.right_bumper){ shooter.setVelocity(target); }
            if (gamepad1.dpadUpWasPressed()){ kP += 10; setCoefficients(); }
            if (gamepad1.dpadDownWasPressed()){ kP -= 10; setCoefficients(); }
            if (gamepad1.yWasPressed()){ kD += 0.01; setCoefficients(); }
            if (gamepad1.aWasPressed()){ kD -= 0.01; setCoefficients(); }
            if (gamepad1.left_trigger > 0.8){ intake.setPower(1); }
            if (gamepad1.dpadLeftWasPressed()){ target += 10; }
            if (gamepad1.dpadRightWasPressed()){ target -= 10; }

            telemetry.addData("velo", shooter.getVelocity());
            telemetry.addData("target", target);
            telemetry.addData("kP", kP);
            telemetry.addData("kD", kD);
            telemetry.update();
        }
    }

    void setCoefficients(){
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, 0, kD, 0));
    }
}
