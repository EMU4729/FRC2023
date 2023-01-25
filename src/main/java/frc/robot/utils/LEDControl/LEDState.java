package frc.robot.utils.LEDControl;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.Constants;
import frc.robot.utils.logger.Logger;

/**
 * Holds the pattern of colours for some combination of zones at some instant.
 * Used only as a component of a pattern
 */
public class LEDState {
  private Constants cnst = Constants.getInstance();
  /** State Name */
  public final String name;
  /**
   * holds an array of all colours used in the form {r,g,b}
   * unsigned byte 0-255 access as
   * "Byte.toUnsignedInt(colours[col][r(0),g(1),b(2)])"
   */
  private final byte[][] colours;
  /**
   * repeating pattern for which colour is used, 0 is predefined as black
   * unsigned byte 0-255 access as "Byte.toUnsignedInt(LEDPattern[pat])"
   */
  private final byte[] drawSequence;

  private static List<LEDState> LEDStateList = new ArrayList<LEDState>() {
    {
      add(new LEDState("MidBlack", new byte[][] {}, new byte[] { 0 }));
      add(new LEDState("MidRed", new byte[][] { basicColor.RED }, new byte[] { 1 }));
      add(new LEDState("MidGreen", new byte[][] { basicColor.GREEN }, new byte[] { 1 }));
      add(new LEDState("MidBlue", new byte[][] { basicColor.BLUE }, new byte[] { 1 }));
      add(new LEDState("MidYellow", new byte[][] { basicColor.YELLOW }, new byte[] { 1 }));
      add(new LEDState("MidCyan", new byte[][] { basicColor.CYAN }, new byte[] { 1 }));
      add(new LEDState("MidMagenta", new byte[][] { basicColor.MAGENTA }, new byte[] { 1 }));
      add(new LEDState("MidWhite", new byte[][] { basicColor.WHITE }, new byte[] { 1 }));
    }
  };

  public static class basicColor {
    static byte[] RED = { (byte) 255, (byte) 0, (byte) 0 };
    static byte[] ORANGE = { (byte) 255, (byte) 127, (byte) 0 };
    static byte[] YELLOW = { (byte) 255, (byte) 255, (byte) 0 };
    static byte[] CHARTREUSE = { (byte) 127, (byte) 255, (byte) 0 };
    static byte[] GREEN = { (byte) 0, (byte) 255, (byte) 0 };
    static byte[] SPRING = { (byte) 0, (byte) 255, (byte) 127 };
    static byte[] CYAN = { (byte) 0, (byte) 255, (byte) 255 };
    static byte[] AZURE = { (byte) 0, (byte) 127, (byte) 255 };
    static byte[] BLUE = { (byte) 0, (byte) 0, (byte) 255 };
    static byte[] VIOLET = { (byte) 127, (byte) 0, (byte) 255 };
    static byte[] MAGENTA = { (byte) 255, (byte) 0, (byte) 255 };
    static byte[] ROSE = { (byte) 255, (byte) 0, (byte) 127 };
    static byte[] WHITE = { (byte) 255, (byte) 255, (byte) 255 };
    // black predefined
  }

  /**
   * Creates a new LEDState stored in LEDPattern,
   * Passing back a unique Name which may be used to find it in
   * {@code LEDStateList}
   * Accessed via {@code getLedState(String name)}
   * 
   * @param name       requested name for new State
   * @param colours    an array of all colours used of the form
   *                   [col-(0->255)][rgb-(0->2)]-(0->255)
   * @param LEDPattern repeating pattern for which colour is used, 0 is predefined
   *                   as black
   *                   [idx-(0->255)]-(0->255)
   * @return actual name in use (= Name if an identical LEDState is not found,
   *         = Name of existing identical LEDState if one is found)
   * @throws IllegalArgumentException no existing identical LEDState could be
   *                                  found and requested
   *                                  Name is already in use
   * @throws IllegalArgumentException more colours used in pattern than provided
   *                                  for
   */
  public static String newLEDState(String name, int[][] colours, int[] drawPattern) // make new LEDState
      throws IllegalArgumentException {
    // create empty coloursByte of size and load entries in colours after converison
    byte[][] coloursByte = new byte[colours.length][3];
    for (int i = 0; i < colours.length; i++) {
      for (int j = 0; j < 3; j++) {
        coloursByte[i][j] = (byte) colours[i][j];
      }
    }

    // create empty drawPatternByte of size and load entries in drawPattern after
    // converison
    byte[] drawPatternByte = new byte[drawPattern.length];
    for (int i = 0; i < drawPattern.length; i++)
      drawPatternByte[i] = (byte) drawPattern[i];

    // check for existing equivilant state or non-equivilant state with same name
    boolean dupName = false;
    for (LEDState existing : LEDStateList) {
      if (Arrays.deepEquals(coloursByte, existing.colours) && Arrays.equals(drawPatternByte, existing.drawSequence))
        return existing.name;
      if (name == existing.name)
        dupName = true;
    }
    if (dupName)
      throw new IllegalArgumentException("LedState : newLedState :" +
          " name already used for non-equivilant state");

    LEDState tmp = new LEDState(name, coloursByte, drawPatternByte);
    LEDStateList.add(tmp);
    return name;
  }

  private LEDState(String name, byte[][] colours, byte[] drawPattern) { // private constructor
    int maxColourUsed = -1;
    for (int i = 0; i < drawPattern.length; i++) {
      int tmpColNum = drawPattern[i];
      maxColourUsed = maxColourUsed > tmpColNum ? maxColourUsed : tmpColNum;
    }
    if (maxColourUsed > colours.length) {
      throw new IllegalArgumentException(
          "LEDState : New State : More colours used in pattern pattern than provided in array");
    } else if (maxColourUsed < colours.length) {
      Logger.warn("LEDState : New State : Last colour/s unused," +
          " Have you remembered 0 is already defined in the pattern as black : " +
          "Pattern-" + drawPattern);
    }

    this.name = name;
    this.colours = colours;
    this.drawSequence = drawPattern;
  }

  public int[][] setToBuff(int[][] colourBuff, short[][] LEDsRanges) {
    int patternIdx = 0;
    for (int i = 0; i < colourBuff.length; i++) {
      System.out.println(colourBuff[i]);
      if (colourBuff[i] != null)
        continue; // skip LEDs filled by higher priority Patterns
      for (short[] ledRng : LEDsRanges) {
        System.out.println(ledRng[0] + " " + ledRng[1]);
        if (i >= ledRng[0] && i <= ledRng[1]) { // only add colours if pattern covers this LED
          colourBuff[i] = getCol(drawSequence[patternIdx]); // get colour for this stage in the sequence
          break;
        }
      }
      patternIdx = patternIdx + 1 < drawSequence.length ? patternIdx + 1 : 0;
    }
    return colourBuff;
  }

  public int[] getCol(int col) {
    if (col == 0)
      return new int[] { 0, 0, 0 };
    return new int[] {
        Byte.toUnsignedInt(colours[col - 1][0]),
        Byte.toUnsignedInt(colours[col - 1][1]),
        Byte.toUnsignedInt(colours[col - 1][2])
    };
  }

  public static LEDState getLedState(String name) {
    for (LEDState ledState : LEDStateList) {
      if (ledState.name == name)
        return ledState;
    }
    if (name == "Black")
      throw new IllegalStateException("LEDState : getLEDState : default State " +
          "\"Black\" not found. check name has not changed");
    return getLedState("Black");
  }

  @Override
  public String toString() {
    String tmpCols = "[";
    for (byte[] col : colours) {
      if (tmpCols.length() > 1)
        tmpCols += ", ";
      tmpCols = tmpCols + "[" + col[0] + "," + col[1] + "," + col[2] + "]";
    }
    tmpCols += "]";

    String tmpSequ = "[";
    for (byte sequ : drawSequence) {
      if (tmpSequ.length() > 1)
        tmpSequ += ",";
      tmpSequ += sequ;
    }
    tmpSequ = "]";

    return "LEDState: {name-" + name + ", colours-" + tmpCols + ", sequence-" + tmpSequ + "}";
  }
}
