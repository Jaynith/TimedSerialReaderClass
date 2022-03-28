using System;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace fastSerialReaderPrototype
{
    public class DataStreamEventArgs : EventArgs
    {
        private byte[] _byteArray;

        public DataStreamEventArgs(byte[] byteArray)
        {
            _byteArray = byteArray;
        }

        public byte[] Response
        {
            get { return _byteArray; }
        }
    }

    public class SerialClient : IDisposable
    {

        private String _port;
        private int _baudRate;
        private SerialPort _serialPort;
        private Thread serThread;
        private Double _packetRate;
        private DateTime _lastReceive;

        private const int freqCriticalLimit = 285;

        public SerialClient(String port)
        {
            _port = port;
            _baudRate = 921600;
            _lastReceive = DateTime.MinValue;

            serThread = new Thread(new ThreadStart(SerialReceiving));
            serThread.Priority = ThreadPriority.AboveNormal;
            serThread.Name = "SerialHandler" + serThread.ManagedThreadId;
        }

        public SerialClient (String port, int baudRate) 
            : this(port)
        {
            _baudRate = baudRate;
        }

        public event EventHandler<DataStreamEventArgs> OnReceiving;

        public String Port
        {
            get { return _port; }
        }
        public int BaudRate
        {
            get { return _baudRate; }
        }
        public String ConnectionString
        {
            get
            {
                return String.Format("Port : {0} | Baud Rate : {1}", _serialPort.PortName, _serialPort.BaudRate.ToString()); 
            }
        }

        public bool OpenConnection()
        {
            try
            {
                if (_serialPort == null)
                {
                    _serialPort = new SerialPort(_port, _baudRate, Parity.None);
                }
                else if (!_serialPort.IsOpen)
                {
                    _serialPort.ReadTimeout = -1;
                    _serialPort.WriteTimeout = -1;

                    _serialPort.Open();

                    if (_serialPort.IsOpen)
                        serThread.Start();
                }
            }
            catch (Exception ex)
            {
                return false;
            }
            return true;
        }

        public void CloseConnection()
        {
            if (_serialPort != null && _serialPort.IsOpen)
            {
                serThread.Abort();
                if (serThread.ThreadState == System.Threading.ThreadState.Aborted)
                {
                    _serialPort.Close();
                }
            }
        }

        public bool ResetConnection()
        {
            CloseConnection();
            return OpenConnection();
        }

        public void Transmit (byte[] packet)
        {
            _serialPort.Write(packet, 0, packet.Length);
        }

        public int Receive (byte[] byteArray, int offset, int length)
        {
            int bytesIn = 0;

            if (length > 0)
            {
                bytesIn = _serialPort.Read(byteArray, offset, length);
            }

            return bytesIn;
        }

        public void Dispose()
        {
            CloseConnection();
            if (_serialPort != null)
            {
                _serialPort.Dispose();
                _serialPort = null;
            }
        }

        private void SerialReceiving()
        {
            while (true)
            {
                int readCount = _serialPort.BytesToRead;

                TimeSpan threadInterval = DateTime.Now - _lastReceive;

                byte[] readBuffer = new byte[readCount];
                int readBytes = Receive(readBuffer, 0, readCount);

                if (readBytes > 0)
                {
                    OnReception(readBuffer);
                }

                _packetRate = (_packetRate + readBytes) / 2;
                _lastReceive = DateTime.Now;
            
                if ((double)(readBytes + _serialPort.BytesToRead)/2 <= _packetRate)
                {
                    if (threadInterval.Milliseconds > 0)
                    {
                        Thread.Sleep(threadInterval.Milliseconds > freqCriticalLimit ? freqCriticalLimit : threadInterval.Milliseconds);
                    }
                }
            
            }
        }

        private void OnReception(byte[] readData)
        {
            if(OnReceiving != null)
            {
                OnReceiving(this, new DataStreamEventArgs(readData));
            }
        }

    }
}
