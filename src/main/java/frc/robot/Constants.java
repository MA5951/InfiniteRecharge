/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Motors
    public static final int IntakeMotor = 12;
    public static final int TransportationMotor = 13;

    //Pnuematics
    public static final int IntakeSolenoidOne = 2;
    public static final int IntakeSolenoidTwo = 3;

    //Joysticks
    public static final int controlTransportationButton = 4;

    public static final int emitIntakeButton = 3;
    public static final int absorbIntakeButton = 2;
    public static final int openCloseIntakeButton = 1;
    
}
