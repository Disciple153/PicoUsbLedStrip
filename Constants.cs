// Version.cs
public static class Constants
{
    //build
    public static string ROM_ID = "DD";
    public static int LED_STRIP_LENGTH = 98;
    public static int DATA_LENGTH = LED_STRIP_LENGTH * 3;
    public static int SERIAL_PAGE_SIZE = 512; // If there are problems transmitting bytes, lower this.
    public static int PC_TO_PICO_TIMEOUT_MS = 100;
    public static int TRANSMISSION_TIMEOUT_MS = 50;
    public static int START_CODE = 0x53;
    public static string SUCCESS = "S!";
    public static string FAILURE = "F!";

    public enum DisplayMode : byte
    {
        Solid,
        Pulse,
        Stream,
        Fade,
        Scroll
    };
}
