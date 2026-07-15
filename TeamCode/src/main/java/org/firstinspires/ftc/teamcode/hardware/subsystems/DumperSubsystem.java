package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DumperSubsystem extends Constants{
    final DcMotorEx leftElev;
    final DcMotorEx rightElev;
    final Servo outtake;
    final Servo intake;
    final CRServo roller1;
    final CRServo roller2;
    final Telemetry telemetry;
    final Elevator elevator;
    public DumperSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        leftElev = hardwareMap.get(DcMotorEx.class,"leftElev");
        rightElev = hardwareMap.get(DcMotorEx.class,"rightElev");
        outtake = hardwareMap.get(Servo.class, "outtake");
        intake = hardwareMap.get(Servo.class, "intake");
        roller1 = hardwareMap.get(CRServo.class, "roller1");
        roller2 = hardwareMap.get(CRServo.class, "roller2");

//        leftElev.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightElev.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator = new Elevator(leftElev, rightElev, elevkP, elevkF);

        this.telemetry = telemetry;
    } // initialization

    public void tuck(){
        setServos(intakeOpen, outtakeClosed, 0, 0);
        elevator.setSetpoint(tuckedExt);
    }

    public void raise(){
        setServos(intakeClosed, outtakeClosed, 0, 0);
        elevator.setSetpoint(highExt);
    }

    public void dump(){
        setServos(intakeClosed, outtakeOpen, 1, -1);
        elevator.setSetpoint(highExt);
    }

    public void setServos(double intakeAng, double outtakeAng, double roller1Pwr, double roller2Pwr){
        if (intake.getPosition() != intakeAng || outtake.getPosition() != outtakeAng || roller1.getPower() != roller1Pwr || roller2.getPower() != roller2Pwr){
            intake.setPosition(intakeAng);
            outtake.setPosition(outtakeAng);
            roller1.setPower(roller1Pwr);
            roller2.setPower(roller2Pwr);
        }
    }
}