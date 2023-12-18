package org.firstinspires.ftc.teamcode.drive.opmode.auton;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControllerCustom {
    public double Kp, Ki, Kd;
    public ElapsedTime timer = new ElapsedTime();

    public double lastError = 0, integralSum = 0;

    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDControllerCustom(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void start() {
        timer.reset();
    }

    public void setPID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(double target, double state) {
        // PID logic and then return the output
        double error = target - state;

        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (this.Kp * error) + (this.Ki * this.integralSum) + (this.Kd * derivative);

        this.lastError = error;

        // reset the timer for next time
        timer.reset();

        return out;
    }
}
