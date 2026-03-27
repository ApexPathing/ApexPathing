package Drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorEx;
<<<<<<< HEAD
import com.qualcomm.robotcore.hardware.HardwareMap;

=======
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b

import java.util.List;

/**
 * Abstract class implemented by all drivetrain classes
 * @author Sohum Arora
 */
public abstract class Drivetrain {

<<<<<<< HEAD
    //set power methods
    public abstract void setPower(DcMotorEx motor,double power);
    public abstract void setPower(List<DcMotorEx> motors, double power);
    public abstract void setPower(double power);
    //drive train init method
    public abstract void initDrive(HardwareMap hardwareMap, String lfName, String rfName, String lrName, String rrName);

=======
    public abstract void setPower(DcMotorEx motor,double power);
    public abstract void setPower(List<DcMotorEx> motors, double power);
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
}
