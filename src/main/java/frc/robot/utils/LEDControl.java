package frc.robot.utils;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import frc.robot.Constants;
import frc.robot.Variables;

public class LEDControl {
  private static Optional<LEDControl> inst = Optional.empty();

  public static LEDControl getInstance() {
    if (!inst.isPresent())
      inst = Optional.of(new LEDControl());
    return inst.get();
  }

  private Constants cnst = Constants.getInstance();
  private Variables vars = Variables.getInstance();

  private AddressableLED leds = new AddressableLED(cnst.LED_STRING_PORT);
  private AddressableLEDBuffer ledsBuff = new AddressableLEDBuffer(cnst.LED_STRING_LENGTH);
  /** lednum< queue< [r,g,b,endTime,priority]>> */
  private ArrayList<ArrayList<ledState>> ledStack = new ArrayList<ArrayList<ledState>>();

  private Optional<Instant> nextEndTime = Optional.empty();

  public static enum Colour {
    Black,
    Red,
    Green,
    Blue,
    Yellow,
    Cyan,
    Magenta,
    White
  }

  private class ledState {
    public int[] col;
    public Instant endTime;
    public int priority;

    public ledState(int[] col, Instant endTime, int priority) {
      this.col = col;
      this.endTime = endTime;
      this.priority = priority;
    }

    public boolean isFinished() {
      return Instant.now().isAfter(endTime);
    }

    public boolean finishAfter(ledState l) {
      return endTime.isAfter(l.endTime);
    }
  }

  AddressableLEDSim ledsSim = new AddressableLEDSim(leds);

  private LEDControl() {
    leds.setLength(ledsBuff.getLength());
    leds.setData(ledsBuff);
    leds.start();
    ledsSim.setInitialized(true);

    for (int i = 0; i < cnst.LED_STRING_LENGTH; i++) {
      ledStack.add(new ArrayList<ledState>());
    }
  }

  /**
   * 
   * @param ledsToSet range to act on [min, max] inclusive examp (0, 59)
   * @param colour    [r,g,b] 0-255 colour to set
   * @param duration  number of miliseconds a colour is held for (-1 -> inf)
   * @param piority   decides whether a existing colour will be overriden (x=x ->
   *                  new overrides old)
   *                  (1-10)
   */
  public void set(int[] ledsToSet, int[] colour, int duration, int priority) {
    ledState newState = new ledState(colour, Instant.MAX, priority);
    if (duration > 0)
      newState.endTime = Instant.now().plusMillis((long) duration);

    for (int i = ledsToSet[0]; i <= ledsToSet[1]; i++) {
      int j = 0;
      while (ledStack.get(i).size() > j) {
        if (newState.priority < ledStack.get(i).get(j).priority) {
          j++;
        } else {
          break;
        }
      }

      // would the old finish before the new which overrides it
      if (ledStack.get(i).size() > j && newState.finishAfter(ledStack.get(i).get(j))) {
        ledStack.get(i).set(j, newState); // replace
      } else {
        ledStack.get(i).add(j, newState); // insert
      }
    }
    update();
  }

  public void set(int[] ledsToSet, Colour col, int duration, int priority) {
    set(ledsToSet, colourToInt(col), duration, priority);
  }

  public void setAll(int[] colour, int duration, int priority) {
    set(new int[] { 0, cnst.LED_STRING_LENGTH - 1 }, colour, duration, priority);
  }

  public void setAll(Colour col, int duration, int priority) {
    set(new int[] { 0, cnst.LED_STRING_LENGTH - 1 }, colourToInt(col), duration, priority);
  }

  public void setFlash(int[] ledsToSet, int[] colour1, int[] colour2,
      int duration, int priority, int period) {
    if (period > duration)
      throw new IllegalArgumentException("period > duration");
    for (int i = duration; i > 0; i -= period) {
      set(ledsToSet, colour1, i, 4);
      set(ledsToSet, colour2, i - period / 2, 4);
    }
  }

  public void setFlash(int[] ledsToSet, Colour col1, Colour col2,
      int duration, int priority, int period) {
    setFlash(ledsToSet, colourToInt(col1), colourToInt(col2), duration, priority, period);
  }

  private int[] colourToInt(Colour col) {
    int[] setCol = { 0, 0, 0 };
    if (col == Colour.Black)
      return setCol;
    if (col == Colour.Red || col == Colour.Yellow || col == Colour.Magenta)
      setCol[0] = 255;
    if (col == Colour.Green || col == Colour.Yellow || col == Colour.Cyan)
      setCol[1] = 255;
    if (col == Colour.Blue || col == Colour.Cyan || col == Colour.Magenta)
      setCol[2] = 255;
    return setCol;
  }

  private void update() {
    Instant minEndTime = Instant.MAX;
    for (int i = 0; i < cnst.LED_STRING_LENGTH; i++) {
      if (ledStack.get(i).size() == 0) {
        ledsBuff.setRGB(i, 0, 0, 0);
        continue;
      }

      ledState tmp = ledStack.get(i).get(0);
      minEndTime = minEndTime.isBefore(tmp.endTime) ? minEndTime : tmp.endTime;

      ledsBuff.setRGB(i, tmp.col[0], tmp.col[1], tmp.col[2]);
    }
    nextEndTime = minEndTime.equals(Instant.MAX) ? Optional.empty() : Optional.of(minEndTime);
    leds.setData(ledsBuff);
  }

  public void updatePeriodic() {
    Instant timeNow = Instant.now();
    if (nextEndTime.isPresent() && timeNow.isAfter(nextEndTime.get())) {
      for (int i = 0; i < cnst.LED_STRING_LENGTH; i++) {
        while (ledStack.get(i).size() > 0 && timeNow.isAfter(ledStack.get(i).get(0).endTime)) {
          ledStack.get(i).remove(0);
        }
      }
      update();
    }
  }

  public void runDirectionLights() {
    int[] front = { 0, 2 };
    int[] back = { cnst.LED_STRING_LENGTH - 3, cnst.LED_STRING_LENGTH - 1 };
    set(vars.invertDriveDirection ? front : back, Colour.Green, -1, 5);
    set(vars.invertDriveDirection ? back : front, Colour.Red, -1, 5);
  }

  public void runCubeLights() {
    setFlash(new int[] { 3, 56 }, Colour.Magenta, Colour.Black, 3000, 4, 250);
  }

  public void runConeLights() {
    setFlash(new int[] { 3, 56 }, Colour.Yellow, Colour.Black, 3000, 4, 250);
  }
}