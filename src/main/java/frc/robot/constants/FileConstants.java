package frc.robot.constants;

public class FileConstants {
  protected FileConstants() {
  }

  /** file path header for files on usb storage */
  public final String[] PATH_USB = { "u//", "v//" };
  /** file path header for files on internal storage */
  public final String PATH_INTERNAL = System.getenv("HOME");
}
