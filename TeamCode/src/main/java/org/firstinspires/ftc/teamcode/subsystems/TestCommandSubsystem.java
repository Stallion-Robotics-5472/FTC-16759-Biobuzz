package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.command.SubsystemBase;

public class TestCommandSubsystem extends SubsystemBase {
    DcMotor motor;
    AnalogInput potentiometer;
    double commandPower;
    public TestCommandSubsystem(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotor.class, "motor");
        potentiometer = hardwareMap.get(AnalogInput.class, "poten");
    }

    public void setPower(){
        commandPower = potentiometer.getVoltage()/3.3;
        motor.setPower(commandPower);
    }

    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("potentiometer voltage", potentiometer.getVoltage()/3.3);
        telemetry.addData("motor power", motor.getPower());
        telemetry.addData("command power", commandPower);
    }

    public void stop(){
        motor.setPower(0);
    }
}
