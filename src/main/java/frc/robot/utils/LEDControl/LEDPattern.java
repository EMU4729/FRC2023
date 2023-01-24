package frc.robot.utils.LEDControl;

import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;
import frc.robot.Variables;
import frc.robot.utils.logger.Logger;

public class LEDPattern {
  /** holds an arraylist of all States used */
  public final List<LEDState> LEDStates;
  /** repeating pattern for which state is used, 0 is all black */
  public final short[] LEDPattern;

  /** array of all LEDs controlled by the state in the form {start, end} (inclusive) */
  public final short[][] LEDsRanges;


  public final int duration; //ms
  public final int flashDuration; //ms
  public final int priority;

  public Instant lastFlash;
  public Instant endTime;
  public int patternStep;

  /**
   * @param LEDStates
   * @param LEDPattern
   * @param LEDToSet
   */
  public LEDPattern(String[] LEDStates, short[] LEDPattern, short[][] LEDsRanges,
        int duration, int flashDuration, int priority){
    int maxStateUsed = -1;
    for(short patternStep : LEDPattern){
      maxStateUsed = maxStateUsed > patternStep ? maxStateUsed : patternStep;
    }
    if(maxStateUsed > LEDStates.length){
      Logger.error
          ("LEDPattern : New Pattern : More colours used in pattern pattern than provided in array");
    } else if (maxStateUsed < LEDStates.length){
      Logger.warn("LEDPattern : New Pattern : Last colour/s unused,"+
          " Have you remembered 0 is already defined in the pattern as black : "+
          "Pattern-" + LEDPattern);
    }
    if(duration < 1000 / flashDuration) 
        Logger.error("LEDPattern : New Pattern : duration > flash length");

    this.LEDStates = new ArrayList<LEDState>();
    for(String ledState : LEDStates) this.LEDStates.add(LEDState.getLedState(ledState));
    this.LEDPattern = LEDPattern;
    this.LEDsRanges = LEDsRanges;
    this.duration = duration;
    this.flashDuration = flashDuration;
    this.priority = priority;
  }
  public LEDPattern(String LEDStates, short[][] LEDsRanges, int duration, int priority){
    this(new String[] {LEDStates}, new short[] {1}, LEDsRanges, duration, -1, priority);
  }

  public void start(){
    lastFlash = Instant.now();
    endTime = lastFlash.plusMillis(duration);
    patternStep = 0;
  }

  public int[][] update(int[][] colourBuff){
    if(duration >= 0 && Instant.now().isAfter(endTime)) return colourBuff;
    if(LEDPattern.length == 1) return LEDStates.get(0).setToBuff(colourBuff, LEDsRanges);

    int sinceFlash = (int) lastFlash.until(Instant.now(), ChronoUnit.MILLIS);
    double ShiftStage = sinceFlash/duration;

    if(ShiftStage < 0.8) return LEDStates.get(patternStep).setToBuff(colourBuff, LEDsRanges);

    ShiftStage = (ShiftStage-0.8) * 5;
    int[][] current = LEDStates.get(patternStep).setToBuff(new int[colourBuff.length][], LEDsRanges);
    int nextPatternStep = ++patternStep > LEDPattern.length ? 0 : patternStep; //iterate then check
    int[][] next = LEDStates.get(nextPatternStep).setToBuff(new int[colourBuff.length][], LEDsRanges);

    for(int i = 0; i < colourBuff.length; i++){
      if(colourBuff[i] != null) continue;
      colourBuff[i] = new int[3];
      colourBuff[i][0] = (int) (current[i][0] * ShiftStage + next[i][0] * (1 - ShiftStage)) / 2;
      colourBuff[i][1] = (int) (current[i][1] * ShiftStage + next[i][1] * (1 - ShiftStage)) / 2;
      colourBuff[i][2] = (int) (current[i][2] * ShiftStage + next[i][2] * (1 - ShiftStage)) / 2;
    }

    if(ShiftStage > 1) patternStep = nextPatternStep;
    return colourBuff;
  }

  public boolean hasExpired(){
    if(duration < 0) return false; 
    return Instant.now().isAfter(endTime);
  }

  public static void runDirLEDS(){
    Constants cnst = Constants.getInstance();
    Variables vars = Variables.getInstance();
    LEDPattern DirLEDNorm = new LEDPattern(
        LEDState.newLEDState("EndsFor", new int[][] {{255,0,0},{0,255,0}}, new int[] {1,1,1,2,2,2}),
        new short[][] {{(short) cnst.LED_ZONES[0], (short) (cnst.LED_ZONES[1]-1)}, 
                       {(short) cnst.LED_ZONES[2], (short) (cnst.LED_ZONES[3]-1)}},
        -1, 5);
    LEDPattern DirLEDRev = new LEDPattern(
        LEDState.newLEDState("EndsBac", new int[][] {{255,0,0},{0,255,0}}, new int[] {2,2,2,1,1,1}), 
        new short[][] {{(short) cnst.LED_ZONES[0], (short) (cnst.LED_ZONES[1]-1)}, 
                       {(short) cnst.LED_ZONES[2], (short) (cnst.LED_ZONES[3]-1)}},
        -1, 5);
    LEDControl.setPattern(vars.invertDriveDirection ? DirLEDNorm : DirLEDRev);
  }
}
