public static class Constants
{
    //public static int LED_STRIP_LENGTH = 98;
    //public static int DATA_LENGTH = LED_STRIP_LENGTH * 3;
    public const string ROM_ID = "DD";
    public static int SERIAL_PAGE_SIZE = 512; // If there are problems transmitting bytes, lower this.
    public static int TRANSMISSION_TIMEOUT_MS = 50;
    public static int START_CODE = 'S';
    public const string SUCCESS = "S!";
    public const string FAILURE = "F!";
    public const string TIMEOUT = "T!";

    public enum DisplayMode : byte
    {
        Solid,
        Pulse,
        Stream,
        Fade,
        Scroll,
        SpectrumAnalyzer,
        Config,
        Identify
    };
}
