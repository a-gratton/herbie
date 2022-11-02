# !/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime as dt
import serial
import threading
import sys, getopt
import csv

DEFAULT_SERIAL_PORT = ''
DEFAULT_VARIABLES_TO_PLOT = []
DEFAULT_UNITS = ''
DEFAULT_TITLE = 'MRI'
DEFAULT_INTERVAL_MS = 10
DEFAULT_MAX_SAMPLES = 200

USAGE = '''Usage:
python3 mri.py [OPTIONS]

OPTIONS:
   -h, --help
      Outputs the help menu
   -u, --units [units]
      Sets the units of the plot
   -t, --title [title]
      Sets the title of the plot
   -p, --port [port]
      Sets the COM port to read serial data from
   -i, --interval [interval_ms]
      Sets the update frequency of the plot (default interval is {}ms)
   -s, --samples [samples]
      Sets the max number of samples to show on the plot at each instant (default is {} samples)
   -v, --variable [variable]
      Adds a variable to the plot
   -l, --log [path]
      Logs the values of the collected variables to the file specified at <path>

NOTE: For the plotter to work, make sure you are continuously outputting lines of data over 
the serial port, where each line has the following format:

[VARIABLE1_NAME] [VARIABLE1_VALUE] [VARIABLE2_NAME] [VARIABLE2_VALUE] ...

where VARIABLE#_NAME is a argument passed into the options list
'''.format(DEFAULT_INTERVAL_MS, DEFAULT_MAX_SAMPLES)


new_data_ready = False
new_data_values = []
run_read_data_thread = True
read_data_thread = None
log_file = None
csv_writer = None


def read_data(ser, variables_to_plot):
    global run_read_data_thread
    global new_data_ready
    global new_data_values
    global csv_writer

    for i in variables_to_plot:
        new_data_values.append(0)

    while run_read_data_thread:
        line = ser.readline().strip().split()
        try:
            for i in range(len(line)-1):
                for j in range(len(variables_to_plot)):
                    if variables_to_plot[j] in line[i].decode("utf-8"):
                        new_data_values[j] = float(line[i+1].decode("utf-8"))
                        new_data_ready = True
                        if csv_writer is not None:
                            csv_writer.writerow([dt.datetime.now().timestamp()] + new_data_values)
        except:
            pass


def animate(i, ax, start_time, variables_to_plot, title, units, max_samples, xs, ys):
    global new_data_ready
    global new_data_values

    if new_data_ready:
        xs.append(dt.datetime.now().timestamp() - start_time)
        for i in range(len(variables_to_plot)):
            ys[i].append(new_data_values[i])
        new_data_ready = False

    xs = xs[-max_samples:]
    for i in range(len(ys)):
        ys[i] = ys[i][-max_samples:]

    ax.clear()

    for i in range(len(variables_to_plot)):
        ax.plot(xs, ys[i], label=variables_to_plot[i])
        if len(ys[i]) > 0:
            ax.annotate(str(ys[i][-1]), xy=(xs[-1], ys[i][-1]))

    ax.legend()

    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel(units)


def on_close(sig):
    global run_read_data_thread
    global log_file
    run_read_data_thread = False
    if log_file is not None:
        log_file.close()
    read_data_thread.join()
    plt.close()
    sys.exit(0)


def main(argv):
    units = DEFAULT_UNITS
    title = DEFAULT_TITLE
    variables_to_plot = DEFAULT_VARIABLES_TO_PLOT
    serial_port = DEFAULT_SERIAL_PORT
    interval_ms = DEFAULT_INTERVAL_MS
    max_samples = DEFAULT_MAX_SAMPLES
    log_file_path = None

    # User args
    try:
        opts, _ = getopt.getopt(argv,"hu:t:p:i:s:v:l", ['help', 'units=', 'title=', 'port=', 'interval=', 'samples=', 'variable=', 'log='])
    except getopt.GetoptError:
        print(USAGE)
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            print(USAGE)
            sys.exit(0)
        elif opt in ('-u', '--units'):
            units = arg
        elif opt in ('-t', '--title'):
            title = arg
        elif opt in ('-p', '--port'):
            serial_port = arg
        elif opt in ('-i', '--interval'):
            interval_ms = int(arg)
        elif opt in ('-s', '--samples'):
            max_samples = int(arg)
        elif opt in ('-v', '--variable'):
            variables_to_plot.append(arg)
        elif opt in ('-l', '--log'):
            print(arg);
            log_file_path = arg;

    global run_read_data_thread
    global read_data_thread
    global log_file
    global csv_writer

    if log_file_path is not None:
        log_file = open(f"{log_file_path}", 'w', newline='')
        csv_writer = csv.writer(log_file)
        csv_writer.writerow(["Timestamp"] + variables_to_plot)

    start_time = dt.datetime.now().timestamp()

    ser = serial.Serial(
        port=serial_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )

    read_data_thread = threading.Thread(target=read_data, args=(ser, variables_to_plot))
    read_data_thread.start()

    fig = plt.figure('MRI')
    ax = fig.add_subplot(1, 1, 1)
    xs = []
    ys = []
    for _ in variables_to_plot:
        ys.append([])

    ani = animation.FuncAnimation(fig, animate, fargs=(ax, start_time, variables_to_plot, title, units, max_samples, xs, ys), interval=interval_ms)
    fig.canvas.mpl_connect('close_event', on_close)
    plt.show()

    run_read_data_thread = False
    log_file.close()
    read_data_thread.join()
    plt.close()

if __name__ == '__main__':
    main(sys.argv[1:])
