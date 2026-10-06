package org.firstinspires.ftc.teamcode.subsystems.scrimbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.command.SubsystemBase;

public class ScrimIntakeSubsys extends SubsystemBase {
    DcMotorEx intake;
    double speed = 0;
    public ScrimIntakeSubsys(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    @Override
    public void periodic() {
        speed = intake.getVelocity();
    }
    public void setIntake(double power) { intake.setPower(power); }
    public void stop() { intake.setPower(0); }
}
