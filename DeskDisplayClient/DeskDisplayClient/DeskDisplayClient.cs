// See https://aka.ms/new-console-template for more information
using System.IO.Ports;
using System.Text;

class DeskDisplay
{

    const int LED_STRIP_LENGTH = 96;
    const int DATA_LENGTH = (LED_STRIP_LENGTH * 3);

    static void Main(string[] args)
    {
        SerialPort? serialPort = null;

        try
        {
            serialPort = GetSerialPort();

            Console.WriteLine("1");
            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new Exception("DeskDisplay not found.");
            }
            Console.WriteLine("2");

            // Prepare data to be written
            byte[] ledValues = new byte[DATA_LENGTH];

            for (int i = 0; i < LED_STRIP_LENGTH; i++)
            {
                ledValues[(3 * i) + 0] = (byte)(i + 1);
                ledValues[(3 * i) + 1] = (byte)(i + 1);
                ledValues[(3 * i) + 2] = (byte)(i + 1);
            }

            WriteToSerialPort(serialPort, ledValues);

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

    private static void WriteToSerialPort(SerialPort serialPort, byte[] ledValues)
    {
        // Write data
        List<byte> copy;
        int offset = 0;
        int count = 16;
        while (offset + count < ledValues.Length)
        {
            serialPort.Write(ledValues, offset, count);
            //copy = Receive(serialPort);
            Confirm(serialPort, new List<byte>(ledValues).GetRange(offset, count));
            offset += count;
        }

        serialPort.Write(ledValues, offset, ledValues.Length - offset);
        Confirm(serialPort, new List<byte>(ledValues).GetRange(offset, ledValues.Length - offset));
    }

    static bool Confirm(SerialPort serialPort, List<byte> data)
    {
        bool result = true;
        byte received;

        foreach (byte b in data)
        {
            try
            {
                received = (byte)serialPort.ReadByte();
                result = b == received;
                if (result)
                {
                    Console.WriteLine("Recieved: " + b.ToString("X"));
                }
                else
                {
                    Console.WriteLine("Expected: " + b.ToString("X") +
                        " Recieved: " + received.ToString("X"));
                }
            }
            catch (TimeoutException)
            {
                Console.WriteLine("No input");
                break;
            }
        }
        Console.WriteLine();

        return result;
    }

    private static SerialPort? GetSerialPort()
    {
        string deviceName;
        SerialPort? serialPort = null;

        foreach (String portName in SerialPort.GetPortNames())
        {
            Console.WriteLine(portName);

            serialPort = new SerialPort(portName);
            serialPort.ReadTimeout = 100;
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
                deviceName = serialPort.ReadLine();
                Console.WriteLine("Device Name: " + deviceName);
            }
            catch (TimeoutException)
            {
                serialPort.Close();
                continue;
            }
        }

        return serialPort;
    }

    static List<byte> Receive(SerialPort serialPort)
    {
        List<byte> data = new List<byte>();
        bool timeout = false;

        while (!timeout)
        {
            try
            {
                data.Add((byte)serialPort.ReadByte());
            }
            catch (TimeoutException)
            {
                timeout = true;
            }
        }

        return data;
    }

}