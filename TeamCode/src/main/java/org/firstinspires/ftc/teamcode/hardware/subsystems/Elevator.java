package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    DcMotorEx motor1;
    DcMotorEx motor2;
    double kP = 0;
    double kF = 0;
    double setpoint = 0;
    double error = 0;
    int velocity = 0;

    public Elevator(DcMotorEx elev1, DcMotorEx elev2, double kP, double kF){
        motor1 = elev1;
        motor2 = elev2;

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetEncoders();

        setPID(kP, kF);
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;

        while (!isAtSetpoint()){
            error = 2 * setpoint - motor1.getCurrentPosition() - motor2.getCurrentPosition();
            velocity = (int) (error * kP + kF);

            setVelocity(velocity);
        }
    }

    public boolean isAtSetpoint(){
        return (motor1.getCurrentPosition() > setpoint-10 &&
                motor1.getCurrentPosition() < setpoint+10 &&
                motor2.getCurrentPosition() > setpoint-10 &&
                motor2.getCurrentPosition() < setpoint+10);
    }

    public void setPID(double kP, double kF){
        if (this.kP != kP || this.kF != kF){
            this.kP = kP;
            this.kF = kF;
        }
    }

    public void setVelocity(int velocity){
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }

    public void resetEncoders(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}