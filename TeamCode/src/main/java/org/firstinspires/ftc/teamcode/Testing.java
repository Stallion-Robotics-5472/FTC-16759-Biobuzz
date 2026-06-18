package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TestingTele", group = "Buzz")
//@Disabled
public class Testing extends LinearOpMode {

    DcMotorEx motor;
    double kP;
    double kV;
    double kG;
    double maxVel;
    double maxAccel;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class,"motor");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(motor,maxVel,maxAccel,kP,kV,kG);

        waitForStart();

        while (opModeIsActive()) {



            telemetry.addData("MaxVel",maxVel);
            telemetry.addData("MaxAccel",maxAccel);
            telemetry.addData("kP",kP);
            telemetry.addData("kV",kV);
            telemetry.addData("kG",kG);
            telemetry.update();
        }
    }
}
