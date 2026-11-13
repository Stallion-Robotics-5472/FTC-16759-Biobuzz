package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="LeavePark")
public class LeaveParkAuto extends LinearOpMode {
    DcMotor bl;
    DcMotor br;
    DcMotor fl;
    DcMotor fr;
    ElapsedTime timer;
    boolean farPark;

    @Override
    public void runOpMode() throws InterruptedException {
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeInInit()){
            telemetry.addLine("Select if Far or Close Park");
            telemetry.addLine("dPad left: Far | dPad right: Close");
            telemetry.addData("Park", farPark ? "far" : "close");
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()){
            move(0.5,0,0,
                    farPark ? 2000 : 3000);
        }
    }

    void mecanumDrive(double y, double x, double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }

    void move(double forward, double strafe, double rotate, double duration){
        timer.reset();

        if (timer.milliseconds() < duration) {
            mecanumDrive(forward, strafe, rotate);
        } else {
            mecanumDrive(0,0,0);
        }
    }
}
