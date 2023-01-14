package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class LEDControl {
  private static Optional<LEDControl> inst = Optional.empty();
  public static LEDControl getInstance() {
    if(!inst.isPresent()) inst = Optional.of(new LEDControl());
    return inst.get();
  }

  private Constants cnst = Constants.getInstance();

  private AddressableLED leds = new AddressableLED(cnst.LED_STRING_PORT);
  private AddressableLEDBuffer ledsBuff = new AddressableLEDBuffer(cnst.LED_STRING_LENGTH);
  /** lednum< queue< [r,g,b,endTime,priority]>> */
  private ArrayList<ArrayList<int[]>> ledStack = new ArrayList<ArrayList<int[]>>();

  public static enum Colour {
    Red,
    Green,
    Blue,
    Yellow,
    Cyan,
    Magenta,
    White
  }

  private LEDControl(){
    leds.setLength(ledsBuff.getLength());
    leds.setData(ledsBuff);
    leds.start();
  }

  /**
   * 
   * @param ledsToSet range to act on [min, max] inclusive
   * @param colour [r,g,b] 0-255 colour to set
   * @param duration number of seconds a colour is held for (-1 -> inf)
   * @param piority decides whether a existing colour will be overriden (x=x -> new overrides old)
   * (1-10) 
   */
  public void set(int[] ledsToSet, int[] colour, double duration, int priority) {
    int[] ledValue = {colour[0], colour[1], colour[2], }
  }
  public void set(int[] ledsToSet, Colour col, double duration, int priority) {
    set(ledsToSet, colourToInt(col), duration, priority);
  }

  public void setAll(int[] colour, int duration, int priority) {
    set(new int[] {0,cnst.LED_STRING_LENGTH}, colour, duration, priority);
  }
  public void setAll(Colour col, int duration, int priority) {
    set(new int[] {0,cnst.LED_STRING_LENGTH}, colourToInt(col), duration, priority);
  }

  private int[] colourToInt(Colour col){
    int[] setCol = {0,0,0};
    if(col == Colour.Red   || col == Colour.Yellow || col == Colour.Magenta) setCol[0] = 255;
    if(col == Colour.Blue  || col == Colour.Yellow || col == Colour.Cyan)    setCol[1] = 255;
    if(col == Colour.Green || col == Colour.Cyan   || col == Colour.Magenta) setCol[2] = 255;
    return setCol;
  }
}