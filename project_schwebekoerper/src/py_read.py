import os, serial


def str_process(str):
    str_data = str.split('\n\r')
    for i in str_data:
        if 'state' in i:
            data_type = "state"
            data = i.split(' ')
            data.pop(0)
            data.pop(-1)
            data = [float(i) for i in data]
            return data,data_type

        elif 'input' in i:
            data_type = "input"
            data = i.split(' ')
            data.pop(0)
            data.pop(-1)
            data = [float(i) for i in data]
            return data,data_type
        else:
            print("wrong input\n")

if __name__ == "__main__":
    test_data = "state: 12.34 45.67 123 1029301 \n\r"
    print(str_process(test_data))

    sys_name = os.name
    print("system name is:" + sys_name)
    if sys_name is 'nt':
        ser_port = 'com3'
    elif  sys_name is 'posix':
        ser_port = '/dev/ttyACM0'
    else:
        print("system name Error")
    ser = serial.Serial()
    ser.port = ser_port
    ser.baudrate = 115200
    ser.bytesize = serial.EIGHTBITS
    ser.stopbits = serial.STOPBITS_ONE

    try:
        ser.open()
    except serial.SerialException:
        print("serial exception")

    if ser.isOpen():
        print("seirial opened")
        while(1):
            # x = ser.readline()
            x = ser.read_until(b'\r',size=None)
            x_str = x.decode("utf-8")


        ser.close()
    else:
        print("cannnot open serial port")