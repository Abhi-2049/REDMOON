import time
import serial
from threading import Thread

def serial_command(serial):
    """separate thread so commands can be send to the Arduino whilst measurements are coming in 

    Args:
        serial (serial connection variabla): serial connection to the Arduino
    """
    while(True):
        command = input()
        serial.write(command.encode('ascii'))

def main():
    """open folder to store data into, read data, add timestamps, store it in csv file.
    Commands can be received via the parallel thread
    """
    # create datafile to store into
    date_time = time.strftime("%b_%d_%H-%M-%S", time.localtime())
    #datafile = "E:\SPACEFLIGHT TU DELFT\THESIS\LUNAR ZEBRO\Assembly, Integration and Testing\Payload Functional Testing\Data logging and processing\MSP_CSV\\FGDOS_03F_PAYLOAD_MAIN_FUNC_TEST_NOISE_BACKUP_LAB_POWER_" + date_time + ".csv"
    datafile = "\FGDOS_03F_PAYLOAD_MAIN_BACKUP_NOISE_EXT_OSC_LONG_WINDOW_" + date_time + ".csv"    
    # connect with MSP430 
    MSP430_port = "COM6"  # serial port of MSP430
    baud = 9600 # set at same rate as in MSP430 program
    ser = serial.Serial(MSP430_port, baud, timeout=0.1)
    print("Connected to COM port: " + MSP430_port)

    # thread to send commands to Arduino
    th_command = Thread(target=serial_command,args=(ser,))
    th_command.start()

    # display the data to the terminal and save it to csv file
    file = open(datafile, "a")  # append the data to the file
    t_stamp = time.time_ns()
    try:
        while True:
            # wait for data incoming over serial, as soon as something enters buffer, pass
            start = time.time()

            while ser.in_waiting < 1:
                if (time.time() - start) > 3:
                    print("timeout waiting for serial")
                    break
            while ser.in_waiting > 0:
                try:
                    data = ser.readline().decode("ASCII")[:-1]
                except Exception as e:
                    print("error reading serial")
                    print(e)
                    break
                t = time.time_ns()
                datastring = str((t-t_stamp)/1000) + " , " + data
                #datastring = str(t) + " , " + data
                print(datastring)
                file.write(datastring)
    except KeyboardInterrupt:
        pass

    file.close()
    ser.close()  #close the serial port 
    print("file closed, end of program")

if __name__ == "__main__":
    main()
