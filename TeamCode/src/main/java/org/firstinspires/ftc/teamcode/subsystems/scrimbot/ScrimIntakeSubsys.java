package org.firstinspires.ftc.teamcode.subsystems.scrimbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.command.SubsystemBase;

public class ScrimIntakeSubsys extends SubsystemBase {
    DcMotor intake;
    public ScrimIntakeSubsys(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {

    }
    public void setIntake(double power) { intake.setPower(power); }
    public void addTelemetry(Telemetry telemetry){}
    public DcMotor getIntake() { return intake; }
    public void stop() { intake.setPower(0); }
}
