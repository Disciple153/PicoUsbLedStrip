// See https://aka.ms/new-console-template for more information
using System.Collections;
using System.IO.Ports;
using System.Management;
using System.Text;

class DeskDisplay
{

    const int LED_STRIP_PIN = 28;
    const int LED_STRIP_LENGTH = 96;
    const int STATUS_LED = 25;
    const int DATA_LENGTH = (LED_STRIP_LENGTH * 3);

    static void Main(string[] args)
    {
        SerialPort? serialPort = null;
        String deviceName;

        try
        {
            foreach (String portName in SerialPort.GetPortNames())
            {
                Console.WriteLine(portName);

                serialPort = new SerialPort(portName);
                serialPort.ReadTimeout = 10;
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
                    deviceName = serialPort.ReadLine();
                    Console.WriteLine("Device Name: " + deviceName);
                }
                catch (TimeoutException)
                {
                    serialPort.Close();
                    continue;
                }
            }

            Console.WriteLine("1");
            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new Exception("DeskDisplay not found.");
            }
            Console.WriteLine("2");



            byte[] ledValues = new byte[DATA_LENGTH];

            int j = 1;
            // Write data
            for (int i = 0; i < LED_STRIP_LENGTH; i++)
            {
                ledValues[(3 * i) + 0] = (byte) 255;
                ledValues[(3 * i) + 1] = (byte) 255;
                ledValues[(3 * i) + 2] = (byte) 255;
            }

            Console.WriteLine("LedData: " + Convert.ToHexString(ledValues));

            serialPort.Write(ledValues, 0, ledValues.Length);
            serialPort.Close();

            Console.WriteLine("3");

            // Recieve copy
            //String copy = serialPort.ReadLine();

            List<byte> copy = new List<byte>();
            bool timeout = false;

            serialPort.Open();
            while (!timeout)
            {
                try
                {
                    copy.Add((byte) serialPort.ReadByte());
                }
                catch (TimeoutException)
                {
                    timeout = true;
                }
            }

            Console.WriteLine(Encoding.Default.GetString(copy.ToArray()));
            Console.WriteLine(Convert.ToHexString(copy.ToArray()));

            //Console.WriteLine(copy);

            Console.WriteLine("4");

            serialPort.Close();
            Console.WriteLine("5");
        }
        finally
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
            }
        }
    }
}