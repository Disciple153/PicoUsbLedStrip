// See https://aka.ms/new-console-template for more information
using Microsoft.Win32;
using System.Drawing;
using System.IO.Ports;
using System.Text.RegularExpressions;

class PicoUsbLedStripClient
{
     const string MANUAL = "PicoUsbLedStripClient.exe: \n"
        + "    Send a command to the PicoUsbLedStripHost.\n"
        + "\n"
        + "\n"
        + "    List: PicoUsbLedStripClient.exe ls\n"
        + "\n"
        + "        Lists all available PicoUsbLedStripHosts in portid:deviceid format.\n"
        + "\n"
        + "\n"
        + "    Configure: PicoUsbLedStripClient.exe config [-p <Port>] [-d <DeviceId>] \n"
        + "               [-n <NumLeds>]\n"
        + "\n"
        + "        Applies configuration options to the PicoUsbLedStripHost. This must be\n"
        + "        run to set the number of leds on the led strip and the device id.\n"
        + "\n"
        + "        Options:\n"
        + "            -p  --portid    The name of the port of the PicoUsbLedStripHost to\n"
        + "                            be configured.\n"
        + "            -d  --deviceid  The device id to assign to the PicoUsbLedStripHost.\n"
        + "            -n  --numleds   A 16 bit unsigned integer representing the number of\n"
        + "                            LEDs on the LED strip.\n"
        + "\n"
        + "\n"
        + "    Display: PicoUsbLedStripClient.exe <DisplayMode> [-i <DeviceId>]\n"
        + "             [-c <Colors>] [-l <LoopTime>]\n"
        + "        DisplayModes:\n"
        + "            Solid:              Shows a static color or gradient.\n"
        + "            Pulse:              Shows colors while pulsing from minimum to\n"
        + "                                maximum brightness over looptime.\n"
        + "            Stream:             Shows a static color or gradient without saving\n"
        + "                                to flash memory. This is useful for streaming \n"
        + "                                data to the PicoUsbLedStripHost.\n"
        + "            Scroll:             Scrolls colors across the LED strip with a\n"
        + "                                period of looptime.\n"
        + "            SpectrumAnalyzer:   Displays an audio spectrum analysis as\n"
        + "                                brightness over the colors while the colors\n"
        + "                                scroll.\n"
        + "\n"
        + "        Options:\n"
        + "            -c  --colors    A comma delimited list of colors. Each color may be\n"
        + "                            in hexadecimal or plaintext format (ex: green). Each\n"
        + "                            color will fade into the next in a circular pattern.\n"
        + "            -l  --looptime  A 16 bit unsigned integer representing the number of\n"
        + "                            milliseconds in an animation cycle. May be in\n"
        + "                            decimal or hexadecimal format (ex 512: or 0x200).\n"
        + "            -d  --deviceid  The id assigned to the PicoUsbLedStripHost to which\n"
        + "                            the command will be sent.\n";

    const string VID = "2E8A";
    const string PID = "000A";
    const int MAX_RETRIES = 5;

    static void Main(string[] args)
    {
        Constants.DisplayMode displayMode = Constants.DisplayMode.Solid;
        Dictionary<string, Constants.DisplayMode> displayModes = new Dictionary<string, Constants.DisplayMode>();
        Dictionary<string, string> parameters;
        List<string> positionalArgs;
        string? result = null;

        // Build Dictionary mapping strings to DisplayModes
        foreach (Constants.DisplayMode mode in Enum.GetValues<Constants.DisplayMode>())
        {
            displayModes.Add(mode.ToString().ToLower(), mode);
        }

        // Define available options
        string[][] options = new string[][]
        {
            new string[] { "colors", "c" },
            new string[] { "looptime", "l" },
            new string[] { "help", "h" },
            new string[] { "numleds", "n" },
            new string[] { "deviceid", "d" },
            new string[] { "portid", "p"},
        };

        // Parse arguments
        (positionalArgs, parameters) = ParseArgs(args, options); // TODO use out

        //GetScreenColors();

        // PARSE PARAMETERS
        if (positionalArgs.Count > 0 )
            displayModes.TryGetValue(positionalArgs[0], out displayMode);

        // On help or no arguments, print the manual.
        if (parameters.ContainsKey("help") || positionalArgs[0].ToLower() == "help" || positionalArgs.Count < 1)
        {
            Console.WriteLine(MANUAL);
        }
        // List available PicoUsbLedStripHosts.
        else if (positionalArgs[0].ToLower() == "ls")
        {
            GetSerialPort();
        }
        // Configure PicoUsbLedStripHost.
        else if (displayMode == Constants.DisplayMode.Config)
        {
            result = Configure(parameters);
        }
        // Display animation.
        else
        {
            result = Display(displayMode, parameters);
        }

        // Print result.
        if (result != null)
        {
            PrintResult(result);
        }
    }

    /**************************************************************************/
    // COMMANDS
    /**************************************************************************/

    /// <summary>
    /// Displays an animation on the PicoUsbLedStripHost.
    /// </summary>
    /// <param name="displayMode">The animation to display.</param>
    /// <param name="parameters">A Dictionary of parameters which may be used. id, colors, and looptime may be used.</param>
    /// <returns>The result of the transmission.</returns>
    /// <exception cref="Exception"></exception>
    private static string Display(Constants.DisplayMode displayMode, Dictionary<string, string> parameters)
    {
        AssertParameters(parameters, new string[] { "deviceid", "colors" });

        Dictionary<Constants.Config, string> hostConfig;
        SerialPort? serialPort = null;
        List<byte> payload = new();
        List<Color> colors;
        string result;
        ushort loopTime, numLeds;

        try
        {
            // Get the PicoUsbLedStripHost serialPort
            serialPort = GetSerialPort(deviceId: parameters["deviceid"]);
            colors = GetColors(parameters["colors"]);
            hostConfig = GetHostConfig(serialPort);
            numLeds = ParseUShort(hostConfig[Constants.Config.LedStripLength]);

            // Add metadata to payload
            switch (displayMode)
            {
                case Constants.DisplayMode.Solid:
                case Constants.DisplayMode.Stream:
                    break;
                case Constants.DisplayMode.Scroll:
                case Constants.DisplayMode.Pulse:
                case Constants.DisplayMode.SpectrumAnalyzer:
                    AssertParameters(parameters, new string[] { "looptime" });

                    // Get loopTime
                    loopTime = ParseUShort(parameters["looptime"]);

                    payload.Add((byte)loopTime); // Pulse Speed Low
                    payload.Add((byte)(loopTime >> 8)); // Pulse Speed Low

                    break;
                default:
                    throw new Exception("How did you get here??");
            }

            // Add colors to payload
            foreach (Color color in FadeThroughColors(numLeds, colors.ToArray()))
            {
                payload.Add(color.R);
                payload.Add(color.G);
                payload.Add(color.B);
            }

            // Write data to Pico
            result = WriteToSerialPort(displayMode, serialPort, payload);

            // Receive any print statements
            Debug(serialPort);

        }
        finally
        {
            // Close serialPort
            serialPort?.Close();
        }

        return result;
    }

    /// <summary>
    /// Sends new configuration settings to the PicoUsbLedStripHost.
    /// </summary>
    /// <param name="parameters">Dictionary of parameters which may be used. id, port, and numleds will be used,</param>
    /// <returns>The result of the transmission.</returns>
    /// <exception cref="Exception"></exception>
    private static string Configure(Dictionary<string, string> parameters)
    {
        AssertParameters(parameters, new string[] { "numleds", "deviceid", "portid" });

        SerialPort? serialPort = null;
        string result;
        ushort numLeds;
        List<byte> payload;

        try
        {
            serialPort = GetSerialPort(portId: parameters["portid"]);
            numLeds = ParseUShort(parameters["numleds"]);

            // Set the number of leds
            payload = new List<byte>
            {
                (byte)numLeds,
                (byte)(numLeds >> 8),
            };

            if (numLeds < 1)
            {
                throw new Exception("numleds must beat least 1.");
            }

            // Set the device id
            foreach (char c in parameters["deviceid"])
            {
                payload.Add((byte)c);
            }

            payload.Add(0x00);

            // Send command
            result = WriteToSerialPort(Constants.DisplayMode.Config, serialPort, payload);

            Debug(serialPort);
        }
        finally
        {
            // Close serialPort
            serialPort?.Close();
        }

        return result;
    }


    /**************************************************************************/
    // SERIAL
    /**************************************************************************/

    /// <summary>
    /// Writes an array of bytes to the PicoUsbLedStripHost along with a displayMode.
    /// </summary>
    /// <param name="displayMode">The command to be sent to the PicoUsbLedStripHost.</param>
    /// <param name="serialPort">The serialPort referring to the PicoUsbLedStripHost.</param>
    /// <param name="payload">Any bytes of data to be sent with the command.</param>
    /// <param name="retries">The number of retries so far.</param>
    /// <returns>The result of the transmission.</returns>
    /// <exception cref="Exception"></exception>
    private static string WriteToSerialPort(Constants.DisplayMode displayMode, SerialPort serialPort, List<byte> payload, int retries = 0)
    {
        LinkedList<byte> rawPacket;
        List<byte> packet;
        int count, index;
        byte hash;
        string response = "E";
        ushort length;

        payload.Insert(0, (byte)displayMode);

        // Discard any garbage
        serialPort.DiscardInBuffer();

        // Ask to transmit
        serialPort.Write(new byte[] { (byte)Constants.START_CODE }, 0, 1);

        // Receive ready response
        serialPort.ReadTo(Constants.ROM_ID + "\r\n");

        index = 0;
        while (index < payload.Count())
        {
            // Extract a packet with a size <= Constants.SERIAL_PAGE_SIZE
            if (index + Constants.SERIAL_PAGE_SIZE > payload.Count())
                count = payload.Count() - index;
            else
                count = Constants.SERIAL_PAGE_SIZE;

            packet = payload.GetRange(index, count);

            // Build packet
            rawPacket = new(packet);

            // Build hash
            hash = 0;

            foreach (byte b in rawPacket)
            {
                hash += b;
            }

            // Append hash (length is not included)
            rawPacket.AddLast(hash);

            // If this is the first packet, prepend the total transmission length
            if (index == 0)
            {
                length = (ushort)payload.Count();

                rawPacket.AddFirst((byte)(length >> 8));
                rawPacket.AddFirst((byte)length);
            }

            // Write packet
            serialPort.Write(rawPacket.ToArray(), 0, rawPacket.Count);

            // Get response
            response = serialPort.ReadTo("!\r\n");

            switch (response[response.Length - 1] + "!")
            {
                // If Success, send next packet
                case Constants.SUCCESS:
                    index += Constants.SERIAL_PAGE_SIZE;
                    break;

                // If Failure, resend packet
                case Constants.FAILURE:
                    break;

                // If Timeout, retry from beginning
                case Constants.TIMEOUT:
                    if (retries < MAX_RETRIES)
                        return WriteToSerialPort(displayMode, serialPort, payload, retries + 1);
                    else
                        throw new Exception("Too many retries");

                // Otherwise, unknown transmission error.
                default:
                    throw new Exception("Unknown transmission error");
            }
        }

        return response[response.Length - 1] + "!";
    }

    // FIXME Windows only
    /// <summary>
    /// Returns a list of COM port names with the given VID and PID.
    /// </summary>
    /// <param name="VID">Vendor ID.</param>
    /// <param name="PID">Product ID.</param>
    /// <returns>A list of COM port names.</returns>
    static List<string> ComPortNames(String VID, String PID)
    {
        String pattern = String.Format("^VID_{0}.PID_{1}", VID, PID);
        Regex _rx = new(pattern, RegexOptions.IgnoreCase);
        List<string> comports = new();

        RegistryKey rk1 = Registry.LocalMachine;
        RegistryKey rk2 = rk1.OpenSubKey("SYSTEM\\CurrentControlSet\\Enum");

        foreach (String s3 in rk2.GetSubKeyNames())
        {
            RegistryKey rk3 = rk2.OpenSubKey(s3);
            foreach (String s in rk3.GetSubKeyNames())
            {
                if (_rx.Match(s).Success)
                {
                    RegistryKey rk4 = rk3.OpenSubKey(s);
                    foreach (String s2 in rk4.GetSubKeyNames())
                    {
                        RegistryKey rk5 = rk4.OpenSubKey(s2);
                        string location = (string)rk5.GetValue("LocationInformation");
                        RegistryKey rk6 = rk5.OpenSubKey("Device Parameters");
                        string portName = (string)rk6.GetValue("PortName");
                        if (!String.IsNullOrEmpty(portName) && SerialPort.GetPortNames().Contains(portName))
                            comports.Add((string)rk6.GetValue("PortName"));
                    }
                }
            }
        }
        return comports;
    }

    /// <summary>
    /// Returns the serial port with the matching deviceId or portId. If none are provided, all available device portIds and deviceIds are printed and null is returned.
    /// </summary>
    /// <param name="deviceId">The Id assigned to a PicoUsbLedStripHost.</param>
    /// <param name="portId">The com port id of a PicoUsbLedStripHost.</param>
    /// <returns>The desired SerialPort or null.</returns>
    /// <exception cref="Exception"></exception>
    private static SerialPort? GetSerialPort(string? deviceId = null, string? portId = null)
    {
        string receivedDeviceId;
        SerialPort? serialPort = null;

        foreach (string receivedPortId in ComPortNames(VID, PID))
        {
            serialPort = new SerialPort(receivedPortId)
            {
                ReadTimeout = 100,
                WriteTimeout = 1000,
                BaudRate = 115200,
                RtsEnable = true,
                DtrEnable = true
            };

            try
            {
                serialPort.Open();
            }
            catch (IOException)
            {
                continue;
            }

            try
            {
                receivedDeviceId = GetHostConfig(serialPort)[Constants.Config.DeviceId];

                // If neither id not port have been specified, list ports and ids.
                if (deviceId == null && portId == null)
                {
                    serialPort.Close();
                    serialPort = null;
                    Console.WriteLine(receivedPortId + ":" + receivedDeviceId);
                }

                // If port or id match, return the serialPort.
                else if ((deviceId != null && deviceId == receivedDeviceId) ||
                         (portId != null && portId == receivedPortId))
                {
                    break;
                }

                // Try the next serialPort.
                else
                {
                    serialPort.Close();
                    serialPort = null;
                }
            }
            catch (TimeoutException)
            {
                serialPort?.Close();
                continue;
            }
        }

        // Make sure serialPort was successfully found
        if (deviceId != null && portId != null && (serialPort == null || !serialPort.IsOpen))
        {
            if (portId != null)
            {
                throw new Exception("Device with PortId: " + portId + " not found.");
            }
            else
            {
                throw new Exception("Device with DeviceId: " + deviceId + " not found.");
            }
        }

        return serialPort;
    }


    /**************************************************************************/
    // ANIMATION
    /**************************************************************************/
    /// <summary>
    /// Returns a list of Colors which fades through a list of colors in a linear fashion as if the list is circular.
    /// </summary>
    /// <param name="resolution">The length of the resulting list of colors.</param>
    /// <param name="colors">The list of colors to fade through.</param>
    /// <returns>The resulting gradient.</returns>
    private static Color[] FadeThroughColors(int resolution, Color[] colors)
    {
        Color[] fade = new Color[resolution];
        Color color1;
        Color color2;
        float brightness;
        int colorIndex;

        for (int i = 0; i < resolution; i++)
        {
            brightness = (i % ((float)resolution / colors.Length)) / (resolution / colors.Length);
            colorIndex = (i * colors.Length) / resolution;
            color1 = colors[colorIndex];

            colorIndex++;

            if (colorIndex >= colors.Length)
            {
                colorIndex = 0;
            }

            color2 = colors[colorIndex];

            fade[i] = Color.FromArgb(
                (byte)((color1.R * (1 - brightness)) + (color2.R * brightness)),
                (byte)((color1.G * (1 - brightness)) + (color2.G * brightness)),
                (byte)((color1.B * (1 - brightness)) + (color2.B * brightness))
            );
        }

        return fade;
    }

    /// <summary>
    /// Returns a list of Colors which fades through a list of colors in a cosine fashion as if the list is circular.
    /// </summary>
    /// <param name="resolution">The length of the resulting list of colors.</param>
    /// <param name="colors">The list of colors to fade through.</param>
    /// <returns>The resulting gradient.</returns>
    private static Color[] FadeThroughColorsCos(int resolution, Color[] colors)
    {
        Color[] fade = new Color[resolution];
        Color color1;
        Color color2;
        float brightness;
        int colorIndex;

        for (int i = 0; i < resolution; i++)
        {
            brightness = (i % ((float)resolution / colors.Length)) / (resolution / colors.Length);
            colorIndex = (i * colors.Length) / resolution;
            color1 = colors[colorIndex];

            colorIndex++;

            if (colorIndex >= colors.Length)
            {
                colorIndex = 0;
            }

            color2 = colors[colorIndex];

            fade[i] = Color.FromArgb(
                (byte)((color1.R * (1 - ((Math.Cos(Math.PI * brightness) + 1) / -2)))
                     + (color2.R * ((Math.Cos(Math.PI * brightness) + 1) / -2))),

                (byte)((color1.G * (1 - ((Math.Cos(Math.PI * brightness) + 1) / -2)))
                     + (color2.G * ((Math.Cos(Math.PI * brightness) + 1) / -2))),

                (byte)((color1.B * (1 - ((Math.Cos(Math.PI * brightness) + 1) / -2)))
                     + (color2.B * ((Math.Cos(Math.PI * brightness) + 1) / -2)))
            );
        }

        return fade;
    }


    /**************************************************************************/
    // PARSING
    /**************************************************************************/
    /// <summary>
    /// Parses through command line input and returns a list and a dictionary of the result.
    /// </summary>
    /// <param name="args">The command line input.</param>
    /// <param name="options">The possible arguments.</param>
    /// <returns>A list of positional arguments, and a Dictionary of parameters.</returns>
    /// <exception cref="Exception"></exception>
    private static (List<string>, Dictionary<string, string>) ParseArgs(string[] args, string[][] options)
    {
        List<string> positionalArgs = new();
        Dictionary<string, string> parameters = new();
        int index = 0;
        string? parameterName;

        // Get all command line arguments
        while (index < args.Length)
        {
            string arg = args[index++];

            if (arg.StartsWith("-"))
            {
                parameterName = GetParameterName(arg, options);

                if (parameterName != null)
                {
                    switch (parameterName.ToLower())
                    {
                        default:
                            parameters.Add(parameterName.ToLower(), args[index++]);
                            break;
                        case ("help"):
                            parameters.Add(parameterName.ToLower(), "");
                            break;
                    }
                }
                else
                {
                    throw new Exception("Invalid option: " + arg);
                }
            }
            else
            {
                positionalArgs.Add(arg);
            }
        }

        return (positionalArgs, parameters);
    }

    /// <summary>
    /// Given an argument and a set op possible arguments, returns the standard name of the argument.
    /// </summary>
    /// <param name="argument">The argument to be analyzed.</param>
    /// <param name="options">The possible arguments.</param>
    /// <returns>The standard name of the argument.</returns>
    private static string? GetParameterName(string argument, string[][] options)
    {
        string? parameterName = null;
        int index = 0;
        argument = argument.TrimStart('-');

        while (index < options.Length && parameterName == null)
        {
            string[] parameter = options[index++];

            if (parameter.Contains(argument))
            {
                parameterName = parameter[0];
            }
        }

        return parameterName;
    }

    /// <summary>
    /// Gets the config data from the PicoUsbLedStripHost.
    /// </summary>
    /// <param name="serialPort">The serialPort referring to the PicoUsbLedStripHost.</param>
    /// <returns>A Dictionary which maps the config data</returns>
    private static Dictionary<Constants.Config, string> GetHostConfig(SerialPort serialPort)
    {
        Dictionary<Constants.Config, string> hostConfig = new();
        Dictionary<string, Constants.Config> configMap = new();

        foreach (Constants.Config mode in Enum.GetValues<Constants.Config>())
        {
            configMap.Add(((byte)mode).ToString(), mode);
        }

        string result = WriteToSerialPort(Constants.DisplayMode.GetConfig, serialPort, new List<byte>());

        if (result == Constants.SUCCESS)
        {
            foreach (string token in serialPort.ReadLine().TrimEnd(',', '\r').Split(','))
            {
                string[] keyval = token.Split('=');

                if (keyval.Length >= 2)
                {
                    hostConfig.Add(configMap[keyval[0]], keyval[1]);
                }
                else
                {
                    hostConfig.Add(configMap[keyval[0]], "");
                }
            }
        }

        return hostConfig;
    }


    /**************************************************************************/
    // HELPER METHODS
    /**************************************************************************/
    /// <summary>
    /// Prints a human readable result based on the result returned from WriteToSerialPort.
    /// </summary>
    /// <param name="result">The result returned from WriteToSerialPort.</param>
    private static void PrintResult(string? result)
    {
        // Print result
        switch (result)
        {
            case Constants.SUCCESS:
                Console.WriteLine("Success");
                break;

            case Constants.FAILURE:
                Console.WriteLine("Failure");
                break;

            case Constants.TIMEOUT:
                Console.WriteLine("Timeout");
                break;

            default:
                Console.WriteLine("Error");
                break;
        }
    }

    /// <summary>
    /// Parses a string formatted as either decimal or hexadecimal to a ushort.
    /// </summary>
    /// <param name="toParse">A decimal or hexadecimal integer.</param>
    /// <returns>A ushort.</returns>
    private static ushort ParseUShort(string toParse)
    {
        ushort loopTime;
        if (toParse.StartsWith("0x"))
        {
            loopTime = Convert.ToUInt16(toParse, 16);
        }
        else
        {
            loopTime = Convert.ToUInt16(toParse);
        }

        return loopTime;
    }

    /// <summary>
    /// Returns a list of Colors bases on a comma delimited string of English or hexadecimal colors.
    /// </summary>
    /// <param name="colorString">A comma delimited string of English or hexadecimal colors.</param>
    /// <returns>A list of Colors.</returns>
    private static List<Color> GetColors(string colorString)
    {
        List<Color> colors = new List<Color>();

        foreach (string color in colorString.Split(','))
        {
            if (char.IsDigit(color[0]))
            {
                colors.Add(Color.FromArgb(Convert.ToInt32("00" + color, 16)));
            }
            else
            {
                colors.Add(Color.FromName(color));
            }
        }

        return colors;
    }

    /// <summary>
    /// Accepts and prints stdin from the serialPort until the serialPort times out.
    /// </summary>
    /// <param name="serialPort">The serialPort referring to the PicoUsbLedStripHost.</param>
    private static void Debug(SerialPort serialPort)
    {
        try
        {
            while (true) Console.WriteLine("Debug: " + serialPort.ReadLine());
        }
        catch (TimeoutException) { }
        catch (OperationCanceledException) { }

    }

    /// <summary>
    /// Given a list of parameter names, asserts that they exist in the parameter Dictionary.
    /// </summary>
    /// <param name="parameters">The parameter Dictionary.</param>
    /// <param name="assertList">A list of parameters to be asserted.</param>
    /// <exception cref="Exception"></exception>
    private static void AssertParameters(Dictionary<string, string> parameters, string[] assertList)
    {
        foreach (string parameter in assertList)
        {
            if (!parameters.ContainsKey(parameter))
            {
                throw new Exception("Missing parameter: " + parameter + ".");
            }
        }
    }

    //// TODO: This solution is slow. Try different approaches:
    //// https://stackoverflow.com/questions/1483928/how-to-read-the-color-of-a-screen-pixel
    //[DllImport("user32.dll", SetLastError = true)]
    //public static extern IntPtr GetDesktopWindow();
    //[DllImport("user32.dll", SetLastError = true)]
    //public static extern IntPtr GetWindowDC(IntPtr window);
    //[DllImport("gdi32.dll", SetLastError = true)]
    //public static extern uint GetPixel(IntPtr dc, int x, int y);
    //[DllImport("user32.dll", SetLastError = true)]
    //public static extern int ReleaseDC(IntPtr window, IntPtr dc);

    //public static Color GetColorAt(int x, int y)
    //{
    //    IntPtr desk = GetDesktopWindow();
    //    IntPtr dc = GetWindowDC(desk);
    //    int a = (int)GetPixel(dc, x, y);
    //    ReleaseDC(desk, dc);
    //    return Color.FromArgb(255, (a >> 0) & 0xff, (a >> 8) & 0xff, (a >> 16) & 0xff);
    //}

    //public static List<Color> GetScreenColors()
    //{
    //    List<Color> colors = new();
    //    Rectangle screenBounds = Screen.PrimaryScreen.Bounds;

    //    for (int i = 0; i < Int32.Parse(hostConfig["LED_STRIP_LENGTH"]); i++)
    //    {
    //        colors.Add(GetColorAt(100, 100));

    //    }

    //    return colors;
    //}
}