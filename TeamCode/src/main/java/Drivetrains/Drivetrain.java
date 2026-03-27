package Drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.List;

/**
 * Abstract class implemented by all drivetrain classes
 * @author Sohum Arora
 */
public abstract class Drivetrain {

    public abstract void setPower(DcMotorEx motor,double power);
    public abstract void setPower(List<DcMotorEx> motors, double power);
    public abstract void setPower(double power);
    public abstract void initDrive(HardwareMap hardwareMap, String lfName, String rfName, String lrName, String rrName);

}
