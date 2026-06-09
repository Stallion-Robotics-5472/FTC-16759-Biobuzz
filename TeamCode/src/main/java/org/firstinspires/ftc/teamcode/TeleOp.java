package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Buzz")
@Disabled
public class TeleOp extends LinearOpMode {
    DcMotor motor;
    CRServo servo;
    enum TestStates{
        IDLE,
        SERVO,
        MOTOR
    }
    TestStates curState;

    @Override
    public void runOpMode() throws InterruptedException {

        curState = TestStates.IDLE;

        motor = hardwareMap.get(DcMotor.class,"motor");
        servo = hardwareMap.get(CRServo.class,"servo");

        waitForStart();

        while (opModeIsActive()) {

            cycle();

            telemetry.update();
        }
    }

    void cycle(){
        switch (curState){
            case IDLE:
                motor.setPower(0);
                servo.setPower(0);
                if (gamepad1.aWasPressed()){
                    curState = TestStates.SERVO;
                }
                break;
            case SERVO:
                motor.setPower(0);
                servo.setPower(1);
                if (gamepad1.optionsWasPressed()){
                    curState = TestStates.MOTOR;
                }
            case MOTOR:
                motor.setPower(1);
                servo.setPower(0);
                if (gamepad1.dpadUpWasPressed()){
                    curState = TestStates.IDLE;
                }
        }
    }
}
