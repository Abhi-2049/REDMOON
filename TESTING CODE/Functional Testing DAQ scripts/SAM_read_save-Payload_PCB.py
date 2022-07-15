import time
import serial

def main():
    # create datafile to store into
    date_time = time.strftime("%b_%d_%H-%M-%S", time.localtime())
    datafile = "E:\SPACEFLIGHT TU DELFT\THESIS\LUNAR ZEBRO\Assembly, Integration and Testing\Payload Functional Testing\Data logging and processing\SAM_CSV\\PAYLOAD_MAIN_FUNC_TEST_POWER_TEMP_BACKUP_LAB_POWER_" + date_time + ".csv"

    # connect with SAM
    com_port = "COM5"  # serial port of SAM
    baud = 115200  # set at same rate as in Arduino and SAM program
    ser = serial.Serial(com_port, baud, timeout=0.1)
    print("Connected to COM port: " + com_port)

    # display the data to the terminal and save it to csv file in an infinite loop
    file = open(datafile, "a", encoding= "ASCII")
    time_stamp = time.time_ns()
    ser.flushInput()
    
    try:
        while True:
            # wait for data incoming over serial, as soon as something enters buffer, pass
            start = time.time()
            while (ser.in_waiting < 1):
                if (time.time() - start) > 3:
                    print("timeout waiting for serial")
                    break
            while (ser.in_waiting > 0):
                try:
                    #data = ser.readline() #.decode("UTF-8")[:-1]
                    #data = data[data.find(b'Vs1'):]                         $$
                    data = ser.readline().decode("ASCII")[:-1]
                    t = time.time_ns() - time_stamp
                except Exception as e:
                    print(e) # print error if failed
                    break
                
                #datastring = str(t) + "," + data.decode('ascii')            $$
                datastring = str(t) + " , " + data
                print(datastring[:-1])
                file.write(datastring)
                ser.flushInput()

    except KeyboardInterrupt:
        pass

    file.close()
    ser.close()  #close the serial port 
    print("file closed, end of program")

if __name__ == "__main__":
    main()