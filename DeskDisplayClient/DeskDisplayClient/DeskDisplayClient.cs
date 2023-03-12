// See https://aka.ms/new-console-template for more information
using Microsoft.Win32;
using System.Diagnostics;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;

class DeskDisplay
{
    const string VID = "2E8A";
    const string PID = "000A";

    static void Main(string[] args)
    {
        SerialPort? serialPort = null;
        List<byte> leds;

        try
        {
            // Get the deskDisplay serialPort
            serialPort = GetSerialPort();

            // Make sure serialPort was successfully found
            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new Exception("DeskDisplay not found.");
            }

            //leds = new List<byte>
            //{
            //    (byte)Constants.DisplayMode.Pulse
            //};

            //for (int j = 0; j < Constants.LED_STRIP_LENGTH; j++)
            //{
            //    leds.Add(0x80);// (byte)((0xFF * i) / LED_STRIP_LENGTH);
            //    leds.Add(0x00);// (byte)((0xFF * i) / LED_STRIP_LENGTH);
            //    leds.Add(0xFF);// (byte)((0xFF * i) / LED_STRIP_LENGTH);
            //}

            //string result = WriteToSerialPort(serialPort, leds);

            //if (result == Constants.SUCCESS)
            //{
            //    Console.WriteLine("Success");
            //}
            //else
            //{
            //    Console.WriteLine("Failure");
            //}


            Stopwatch stopWatch = new Stopwatch();
            stopWatch.Start();

            for (byte i = 0x00; i < 0xFF; i++)
            {
                leds = new List<byte>
                {
                    (byte)Constants.DisplayMode.Stream
                };

                for (int j = 0; j < Constants.LED_STRIP_LENGTH; j++)
                {
                    leds.Add(i);// (byte)((0xFF * i) / LED_STRIP_LENGTH);
                    leds.Add(i);// (byte)((0xFF * i) / LED_STRIP_LENGTH);
                    leds.Add(i);// (byte)((0xFF * i) / LED_STRIP_LENGTH);
                }

                WriteToSerialPort(serialPort, leds);


            }

            stopWatch.Stop();

            Console.WriteLine(1f / (stopWatch.Elapsed.TotalSeconds / 0xFF) + " FPS");

        }
        finally
        {
            // Close serialPort
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
            }
        }
    }

    private static string WriteToSerialPort(SerialPort serialPort, IEnumerable<byte> payload)
    {

        // Get confirmation
        serialPort.DiscardInBuffer();
        serialPort.Write(new byte[] { (byte)Constants.START_CODE }, 0, 1);

        string deviceName = "DD" + serialPort.ReadTo("DD\r\n");
        //Console.WriteLine("WRITE: device name: " + deviceName);

        // Payload
        LinkedList<byte> leds = new LinkedList<byte>(payload);

        // Length
        short length = (short) payload.Count();

        leds.AddFirst((byte) (length >> 8));
        leds.AddFirst((byte) length);

        // Hash
        byte hash = 0;

        foreach (byte b in leds)
        {
            hash += b;
        }

        leds.AddLast(hash);

        // Write
        serialPort.Write(leds.ToArray(), 0, leds.Count);

        // Get response
        string response = serialPort.ReadTo("!\r\n");
        //Console.WriteLine("Response: " + response);

        return response[response.Length - 1] + "!";
    }

    // FIXME Windows only
    static List<string> ComPortNames(String VID, String PID)
    {
        String pattern = String.Format("^VID_{0}.PID_{1}", VID, PID);
        Regex _rx = new Regex(pattern, RegexOptions.IgnoreCase);
        List<string> comports = new List<string>();

        RegistryKey rk1 = Registry.LocalMachine;
        RegistryKey rk2 = rk1.OpenSubKey("SYSTEM\\CurrentControlSet\\Enum");

        foreach (String s3 in rk2.GetSubKeyNames())
        {
            RegistryKey rk3 = rk2.OpenSubKey(s3);
            foreach (String s in rk3.GetSubKeyNames())
            {
                //Console.WriteLine("RK3: " + s);
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

    private static SerialPort? GetSerialPort()
    {
        string deviceName;
        SerialPort? serialPort = null;

        foreach (String portName in ComPortNames(VID, PID))
        {
            Console.WriteLine(portName);

            serialPort = new SerialPort(portName);
            serialPort.ReadTimeout = 5000;
            serialPort.WriteTimeout = 1000;
            serialPort.BaudRate = 115200;
            serialPort.RtsEnable = true;
            serialPort.DtrEnable = true;

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

                serialPort.DiscardInBuffer();
                serialPort.Write(new byte[] { (byte)Constants.START_CODE }, 0, 1);
                deviceName = serialPort.ReadLine().Trim('\r', '\n');
                Console.WriteLine("Device Name: " + deviceName);

                if (deviceName == Constants.ROM_ID)  
                {
                    Thread.Sleep(Constants.TRANSMISSION_TIMEOUT_MS + 1);
                    break;
                }
                else
                {
                    serialPort = null;
                }
            }
            catch (TimeoutException)
            {
                serialPort.Close();
                continue;
            }
        }

        return serialPort;
    }
}