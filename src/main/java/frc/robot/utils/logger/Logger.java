package frc.robot.utils.logger;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;
import java.util.Date;
import java.util.Calendar;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;

import frc.robot.Constants;

public class Logger {
  private static Optional<Logger> inst = Optional.empty();
  private final Constants cnst = Constants.getInstance();

  private ArrayList<LogLine> logCache = new ArrayList<LogLine>();

  private final ReentrantLock cacheLock = new ReentrantLock();

  private Thread LogThread;

  private boolean logPause = false;
  private boolean logStop = false;

  private File logFile;
  private String filePath;
  private boolean filCrtFld = false;

  private Logger() {
    findFilePath();
    makeFile();
    runLoggerThread();
  }

  public static Logger getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new Logger());
    }
    return inst.get();
  }

  public static void header(String content) {
    LogLine logLine = new LogLine(content, LogLevel.HEADER);
    addLine(logLine);
  }

  public static void info(String content) {
    LogLine logLine = new LogLine(content, LogLevel.INFO);
    addLine(logLine);
  }

  public static void warn(String content) {
    LogLine logLine = new LogLine(content, LogLevel.WARN);
    addLine(logLine);
  }

  public static void error(String content) {
    LogLine logLine = new LogLine(content, LogLevel.ERROR);
    addLine(logLine);
  }

  private static void addLine(LogLine line) {
    Logger logger = getInstance();
    logger.cacheLock.lock();
    logger.logCache.add(line);
    logger.cacheLock.unlock();
  }

  private void findFilePath() {
    if (logPause) {
      return;
    }
    try {
      for (String tmpPath : cnst.PATH_USB) {
        if (new File(tmpPath).exists()) {
          filePath = tmpPath;
          System.out.println("Logger : File Path Found : " + filePath);
          return;
        }
      }
      filCrtFld = true;
      System.out.println("Logger : File Path Not Found : Logger printing to consol");
    } catch (SecurityException e) {
      filCrtFld = true;
      System.out.println("Logger : File Path Security Exception : Logger printing to consol");
    }
  }

  private void makeFile() {
    if (filCrtFld || logPause) {
      return;
    }

    Date date = Calendar.getInstance().getTime();
    DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd_hh-mm");
    logFile = new File(filePath + dateFormat.format(date) + ".txt");

    try {
      for (int i = 0; !logFile.createNewFile(); i++) {
        if (i > cnst.REPEAT_LIMIT_LOGGER_CREATION) {
          filCrtFld = true;
          System.out.println("Logger : File Creation Failed : Timed Out : Logger printing to consol");
          return;
        }
        logFile = new File(filePath + dateFormat.format(date) + "_(" + i + ").txt");
      }
    } catch (IOException e) {
      filCrtFld = true;
      System.out.println("Logger : File Creation Failed : IOException : " + e + " : Logger printing to consol");
    } catch (SecurityException e) {
      filCrtFld = true;
      System.out.println("Logger : File Creation Failed : Security Exception : " + e + " : Logger printing to consol");
    }
  }

  public void save() {
    if (logCache.isEmpty() || logPause) {
      return;
    }
    cacheLock.lock();
    if (filCrtFld) {
      while (!logCache.isEmpty()) {
        System.out.println(logCache.remove(0).toString());
      }
      cacheLock.unlock();
      return;
    }
    try {
      FileWriter logWriter = new FileWriter(logFile, true);
      while (!logCache.isEmpty()) {
        logWriter.write(logCache.remove(0).toString());
        logWriter.write(System.getProperty("line.separator"));
      }
      logWriter.close();
    } catch (IOException e) {
      System.out.println("Logger : File Save Failed : IOExeption : " + e);
    } catch (SecurityException e) {
      System.out.println("Logger : File Save Failed : Security Exeption : " + e);
    } finally {
      if (!logCache.isEmpty()) {
        System.out.println("Logger : Save Error : Dumping Cache to Consol");
      }
      while (!logCache.isEmpty()) {
        System.out.println(logCache.remove(0).toString());
      }
      cacheLock.unlock();
    }
    cacheLock.unlock();
  }

  /** reversibly stops operation of the logger */
  public void pause() {
    if (!logStop) {
      logPause = true;
    }
  }

  /** restarts the logger when paused */
  public void unpause() {
    if (!logStop) {
      logPause = false;
    }
  }

  /** irreversably kills the logger */
  public void stop() {
    if (!logStop) {
      pause();
      logStop = true;
    }
  }

  private void runLoggerThread() {
    if (logStop) {
      return;
    }
    LogThread = new Thread(() -> {
      try {
        while (!logStop) {
          save();
          Thread.sleep(1000 / cnst.LOGGER_SAVE_RATE);
        }
      } catch (InterruptedException e) {
        System.out.println("Logger : Save Thread Interupted : " + e);
      }
    });
    LogThread.start();
  }
}
