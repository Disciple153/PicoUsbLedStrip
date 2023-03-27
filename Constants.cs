public static class Constants
{
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
        Scroll,
        SpectrumAnalyzer,
        Config,
        GetConfig
    };

    public enum Config : byte
    {
        DeviceId,
        LedStripLength
    };
}
