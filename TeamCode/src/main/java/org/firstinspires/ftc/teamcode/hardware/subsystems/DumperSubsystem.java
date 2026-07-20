package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.Elevator;

public class DumperSubsystem extends Constants {
    final DcMotorEx leftElev;
    final DcMotorEx rightElev;
    final ServoImplEx outtake;
    final ServoImplEx intake;
    final AnalogInput outtakePos;
    final AnalogInput intakePos;
    final CRServo roller1;
    final CRServo roller2;
    final Telemetry telemetry;
    final Elevator elevator;
    public DumperSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        leftElev = hardwareMap.get(DcMotorEx.class,"leftElev");
        rightElev = hardwareMap.get(DcMotorEx.class,"rightElev");
        outtake = hardwareMap.get(ServoImplEx.class, "outtake");
        intake = hardwareMap.get(ServoImplEx.class, "intake");
        outtakePos = hardwareMap.get(AnalogInput.class, "outtakeEncoder");
        intakePos = hardwareMap.get(AnalogInput.class, "intakeEncoder");
        roller1 = hardwareMap.get(CRServo.class, "roller1");
        roller2 = hardwareMap.get(CRServo.class, "roller2");

//        leftElev.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightElev.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator = new Elevator(leftElev, rightElev, elevkP, elevkF);

        intake.setPwmRange(new PwmControl.PwmRange(500, 2500));
        outtake.setPwmRange(new PwmControl.PwmRange(500, 2500));

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
        if (getIntakePosition() != intakeAng || getOuttakePosition() != outtakeAng || roller1.getPower() != roller1Pwr || roller2.getPower() != roller2Pwr){
            intake.setPosition(intakeAng);
            outtake.setPosition(outtakeAng);
            roller1.setPower(roller1Pwr);
            roller2.setPower(roller2Pwr);
        }
    }

    double getIntakePosition(){
        return intakePos.getVoltage()/3.3;
    }

    double getOuttakePosition(){
        return outtakePos.getVoltage()/3.3;
    }
}