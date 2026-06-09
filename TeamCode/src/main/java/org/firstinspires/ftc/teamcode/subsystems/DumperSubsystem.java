package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DumperSubsystem {
    boolean elevRaise;
    final DcMotorEx leftElev;
    final DcMotorEx rightElev;
    final Telemetry telemetry;
    final Servo leftServo;
    final Servo rightServo;
    Constants constants = new Constants();
    Gamepad opCon;
    public DumperSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        leftElev = hardwareMap.get(DcMotorEx.class,"leftElev");
        rightElev = hardwareMap.get(DcMotorEx.class,"rightElev");
        leftServo = hardwareMap.get(Servo.class,"leftServo");
        rightServo = hardwareMap.get(Servo.class,"rightServo");

        leftElev.setDirection(DcMotorSimple.Direction.FORWARD);
        rightElev.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        leftElev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftElev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElev.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftServo.scaleRange(constants.levelAng,constants.tipAng);
        rightServo.scaleRange(constants.levelAng,constants.tipAng);

        leftElev.setTargetPositionTolerance(10);
        rightElev.setTargetPositionTolerance(10);

        this.opCon = opCon;

        this.telemetry = telemetry;
    } // initialization

    public void tuck(){
        goToPos(constants.tuckedExt);
        tip(0);
    }

    public void raise(){
        goToPos(constants.highExt);
        tip(0);
    }

    public void dump(){
        goToPos(constants.highExt);
        tip(1);
    }

    public void teleData(){
        telemetry.addLine("--DUMPER TELE--");
        telemetry.addData("IntakeOn", elevRaise);
        telemetry.addLine();
    }

    void goToPos(int pos){
        int error = Math.abs(pos-leftElev.getCurrentPosition());
        leftElev.setVelocity(error*constants.elevKP);
        rightElev.setVelocity(error*constants.elevKP);

        leftElev.setTargetPosition(pos);
        rightElev.setTargetPosition(pos);

        leftElev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void tip(double angle){
        leftServo.setPosition(angle);
        rightServo.setPosition(angle);
    }
}