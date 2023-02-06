package frc.robot;

import frc.robot.commands.ArmPickUp;
import frc.robot.commands.BalanceChargePad;

/**
 * Commands - Use this class to initialize and access commands globally.
 */
public class Commands {
  public static BalanceChargePad balanceChargePad = new BalanceChargePad();
  public static ArmPickUp armPickUp = new ArmPickUp();
}
