from MainGUI import Ui_MainWindow
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc
from XInput import *
import gclib
import sys
from simple_pid import PID

from tkinter import *

g = gclib.py()


class UserWindow(qtw.QMainWindow, Ui_MainWindow):
    count = 0
    direction = 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)
        self.centerSlider.pressed.connect(self.setSliderZero)
        self.estop.pressed.connect(self.process_estop)
        self.enable.pressed.connect(self.process_enable)
        self.jog_fwd.pressed.connect(self.process_jog_fwd)
        self.jog_rev.pressed.connect(self.process_jog_rev)
        self.stop_movement.pressed.connect(self.process_stop)
        self.jog_cw.pressed.connect(self.process_cw)
        self.jog_ccw.pressed.connect(self.process_ccw)

        self.jog_fwd.released.connect(self.process_stop)
        self.jog_rev.released.connect(self.process_stop)
        self.jog_cw.released.connect(self.process_stop)
        self.jog_ccw.released.connect(self.process_stop)

        self.velocity.valueChanged.connect(self.update_jog)
        self.activateAngle.pressed.connect(self.start_worker_1)
        self.deactivateAngle.pressed.connect(self.stop_worker_1)
        self.deactivateAngle.pressed.connect(lambda: self.thread[1].stop)
        self.start_goto.pressed.connect(self.process_goto)
        self.pid = PID(10, 1., 0.5, setpoint=2.5)
        self.pid.output_limits = (0, 5)
        self.file = open("output.txt", "w")
        g.GOpen('192.168.42.100 -s ALL')
        self.thread = {}
        self.encoder_target = 0
        self.last_delta_counts = 0
        #
        self.mainUpdateThread = ThreadUpdate(parent=None)
        self.mainUpdateThread.start()
        self.mainUpdateThread.data_ready.connect(self.update_data)
        self.left_mtr_cur_avg = 0
        self.right_mtr_cur_avg = 0
        self.disable_bus.pressed.connect(self.process_bus_disable)
        self.enable_bus.pressed.connect(self.process_bus_enable)

        self.serialThread = ThreadSerial(parent=None)
        self.serialThread.data_ready2.connect(self.process_serial_data)

        self.start_program.pressed.connect(self.process_start_program)
        self.stop_program.pressed.connect(self.process_stop_program)
        self.help_cmds.pressed.connect(lambda: g.GCommand("MG {P2} \"H\"{^10}"))
        self.reset_drive.pressed.connect(lambda: g.GCommand("MG {P2} \"R\"{^10}"))
        self.clear_screen.pressed.connect(lambda: self.text_box.clear())

    def process_start_program(self):
        g.GCommand('XQ #I_SCOM,0')
        self.serialThread.start()
        self.start_program.setEnabled(False)

    def process_stop_program(self):
        g.GCommand('HX #I_SCOM, 0')
        self.start_program.setEnabled(True)

    def process_serial_data(self, key):
        self.text_box.appendPlainText(key)
        g.GCommand('is_empty = 1')

    def process_bus_disable(self):
        g.GCommand('SB 24')

    def process_bus_enable(self):
        g.GCommand('CB 24')

    def setSliderZero(self):
        self.horizontalSlider.setValue(0)

    # Any key pressed called ESTOP
    # def keyPressEvent(self, event):
    #     self.process_estop()

    def process_enable(self):
        # Check if the motors are off (_MOx = 1) and then turn on servo control
        if int(float(g.GCommand('MG _MOA'))) and int(float(g.GCommand('MG _MOB'))):
            g.GCommand('CB 24')  # Enables motor bus voltage
            g.GCommand('GA ,A,,')
            g.GCommand('GR ,-1,,')
            g.GCommand('SH A,B')  # Axis A and B enable goes to an AND gate, both must enable to enable LA-310
            self.enable.setStyleSheet("background-color: green")

        elif int(float(g.GCommand('MG _MOA'))) == 0 and int(float(g.GCommand('MG _MOB'))) == 0:
            self.enable.setStyleSheet("background-color: light grey")
            g.GCommand('MOA')
            g.GCommand('MOB')
            g.GCommand('SB 24')
        else:
            g.GCommand('MOA')
            g.GCommand('MOB')

    # When the velocity is changed change the SP parameter
    def update_jog(self):
        g.GCommand('SP ' + str(int(self.velocity.value()) * 1000))

    def process_jog_rev(self):
        g.GCommand('GA ,A,,')
        self.processSliderInput()
        g.GCommand('JG ' + str(int(self.velocity.value()) * 1000))
        g.GCommand('BG A')

    def process_jog_fwd(self):
        g.GCommand('GA ,A,,')
        self.processSliderInput()
        g.GCommand('JG ' + str(-int(self.velocity.value()) * 1000))
        g.GCommand('BG A')

    def process_stop(self):
        g.GCommand('ST')
        # The ST command clears the gearing ratio
        g.GCommand('GA ,A,,')
        self.processSliderInput()
        g.GCommand('SH A,B')

    def process_cw(self):
        g.GCommand('GR 0,0,0,0')
        velocity = str(-int(self.velocity.value()) * 1000)
        g.GCommand('JG ' + velocity + ',' + velocity)
        g.GCommand('BG A,B')

    def process_ccw(self):
        g.GCommand('GR 0,0,0,0')
        velocity = str(int(self.velocity.value()) * 1000)
        g.GCommand('JG ' + velocity + ',' + velocity)
        g.GCommand('BG A,B')

    # Differential speed to counter drift
    def processSliderInput(self):
        value = self.horizontalSlider.value()
        offset = value / 100
        # When turning left, left axis goes slower, offset value is negative
        if -99 <= value < 0:
            g.GCommand('GR ,' + str(-1 + offset) + ',,')
        # When turning right, right goes slower
        elif 0 < value <= 99:
            g.GCommand('GR ,' + str(-1 + offset) + ',,')
        elif value == 0:
            g.GCommand('GR ,-1,,')

    # Thread to control tool angle
    def start_worker_1(self):
        self.thread[1] = ThreadClass(parent=None, index=1)
        self.thread[1].start()
        self.thread[1].any_signal.connect(self.my_function)

    # Thread to constantly update PT command for incremental move
    def start_worker_2(self):
        self.thread[2] = ThreadClass(parent=None, index=2)
        self.thread[2].start()
        self.encoder_target = float(self.goto_position.text()) * 19771
        # print("Target ", float(self.goto_position.text()) * 19771)
        self.thread[2].any_signal2.connect(self.update_error)

    def stop_worker_1(self):
        self.thread[1].stop()
        g.GCommand('ST')

    def stop_worker_2(self):
        self.thread[2].stop()
        g.GCommand('ST')

    def process_estop(self):
        self.direction = 0
        g.GCommand('ST')
        g.GCommand('SB 24')
        self.process_enable()

    def process_goto(self):
        g.GCommand('GA ,A,,')
        g.GCommand('GR ,-1,,')
        g.GCommand('DE 0,0,0,0')
        g.GCommand('DP 0,0,0,0')
        g.GCommand('SH A,B')
        g.GCommand('PT 1')
        self.start_worker_2()

    # This code is called from another thread and cannot update GUI variables! FIX TODO
    # Associated with worker 2 thread. Calculates new PT position
    def update_error(self, encoder, follower):
        self.error_2.setText(encoder + " " + follower)
        self.error_2.adjustSize()
        # self.label_follower_position.setText(str(round(float(follower) / 1382, 2)))
        encoder = float(encoder)
        follower = float(follower)
        ratio = 13.766  # Motor encoder counts divided by follower encoder counts
        # Follower set point converted from inches to counts, should not change
        f_sp = float(self.goto_position.text()) * 1382
        # Calculate motor encoder counts to get to follower set point
        delta_counts = ((f_sp - follower) * ratio)
        # Check if the difference is larger than slider and issue an update
        if abs(delta_counts) > 15:
            g.GCommand('PA ' + str(encoder + delta_counts))
        else:
            self.stop_worker_2()
        self.last_delta_counts = delta_counts
        error_amt = 0.04
        if self.encoder_target > 0:
            # Target 1000, if adjusted is greater than 1010
            if (encoder + delta_counts) > self.encoder_target * (1 + error_amt):
                # print("Over: ", round(encoder + delta_counts, 2))
                self.error.setText("SLIP")
                self.error.adjustSize()
            # Target 1000,
            elif (encoder + delta_counts) < self.encoder_target * (1 - error_amt):
                # print("Under: ", round(encoder + delta_counts, 2))
                self.error.setText("SLIP")
                self.error.adjustSize()
        elif self.encoder_target < 0:
            if (encoder + delta_counts) > self.encoder_target * (1 - error_amt):
                # print("Under: ", round(encoder + delta_counts, 2))
                self.error.setText("SLIP")
                self.error.adjustSize()
            elif (encoder + delta_counts) < self.encoder_target * (1 + error_amt):
                # print("Over: ", round(encoder + delta_counts, 2))
                self.error.setText("SLIP")
                self.error.adjustSize()
        # Once in position stop the thread

# Tool angle
    def my_function(self, voltage):
        voltage = float(voltage)
        # sliderValue = self.horizontalSlider.value()
        newSliderValue = self.pid(voltage)
        # print("NV:", newSliderValue)
        # Right downward turns increase voltage
        # Left downward turns decrease voltage
        # if voltage > 2.5:
        if newSliderValue > 2.5:
            # self.horizontalSlider.setValue(sliderValue - 5)
            self.horizontalSlider.setValue(int((newSliderValue - 2.5) * 20))
        # elif voltage < 2.5:
        elif newSliderValue < 2.5:
            # print("Low Voltage, turn right")
            # self.horizontalSlider.setValue(sliderValue + 5)
            self.horizontalSlider.setValue(int((newSliderValue - 2.5) * 20))
        self.voltageFeedback.setText(str(voltage))
        self.voltageSetpoint.setText(str(newSliderValue))
        # self.file.writelines(voltage,newSliderValue)

    def closeEvent(self, event):
        self.process_estop()
        g.GClose()

    # Update the labels on the GUI for the position, velocity, error, current
    def update_data(self, data):
        self.label_left_position.setText(str('{:.2f}'.format(round(float(data["left pos"][0]) / 19771, 2))))
        self.label_right_position.setText(str('{:.2f}'.format(round(float(data["right pos"][0]) / 19771, 2))))
        self.label_follower_position.setText(str('{:.2f}'.format(round(float(data["follower pos"][0]) / 1382, 2))))
        self.label_velocity.setText(str('{:.2f}'.format(round(float(data["velocity"][0]) / 19771, 2))))
        self.label_left_position_error.setText(str('{:.2f}'.format(round(float(data["left pos err"][0]) / 19771, 2))))
        self.label_right_position_error.setText(str('{:.2f}'.format(round(float(data["right pos err"][0]) / 19771, 2))))

        N = 25
        # Calculate moving average
        self.left_mtr_cur_avg -= self.left_mtr_cur_avg / N
        self.left_mtr_cur_avg += round(float(data["left mtr cur"][0]) / N, 2)

        self.right_mtr_cur_avg -= self.right_mtr_cur_avg / N
        self.right_mtr_cur_avg += round(float(data["right mtr cur"][0]) / N, 2)

        # Update current on GUI
        self.label_left_motor_current.setText(str('{:.2f}'.format(self.left_mtr_cur_avg)))
        self.label_right_motor_current.setText(str('{:.2f}'.format(self.right_mtr_cur_avg)))


# Thread to gather information to update the GUI with position, velocity, error, and motor current
class ThreadUpdate(qtc.QThread):
    data_ready = qtc.pyqtSignal(dict)

    def __init__(self, parent=None):
        super(ThreadUpdate, self).__init__(parent)
        self.is_running = True
        self.data_packet = {
            "left pos": ['N/A', 'MG _TPA'],
            "right pos": ['N/A', 'MG _TPB'],
            "follower pos": ['N/A', 'MG _TDA'],
            "velocity": ['N/A', 'MG _TVA'],
            "left pos err": ['N/A', 'MG _TEA'],
            "right pos err": ['N/A', 'MG _TEB'],
            "left mtr cur": ['N/A', 'MG _TTA'],
            "right mtr cur": ['N/A', 'MG _TTB'],
        }

    def run(self):
        while True:
            time.sleep(0.01)
            for key in self.data_packet:
                self.data_packet[key][0] = g.GCommand(self.data_packet[key][1])
            self.data_ready.emit(self.data_packet)

    def stop(self):
        self.is_running = False
        print("stopping update thread")
        self.terminate()


# Thread for serial connection with LA 415 via AUX port on DMC 4040
def buffer_has_data():
    # List the variables created in the DMC-4040
    variables = g.GCommand('LV')
    # Check data set ready variable
    data_ready_flag = int(float(g.GCommand('MG dsr')))
    if 'is_empty' in variables:
        # Buffer is not empty
        if int(float(g.GCommand('MG is_empty'))) == 0 and data_ready_flag:
            return True
        # Buffer is empty
        elif int(float(g.GCommand('MG is_empty'))) == 1:
            return False
    # Variable 'is_empty' is not present, program not loaded on DMC 4040
    else:
        return False


def convert_buffer_data(data: list, buff_size: int):

    # Convert to characters
    for i in range(len(data)):
        data[i] = chr(round(float(data[i])) >> 4 * 6)

    # Remove NULL characters from list if they are present
    if data.count('\x00'):
        data.remove('\x00')

    # Remove the first CR and NL characters
    if data.count('\r'):
        data.remove('\r')
    if data.count('\n'):
        data.remove('\n')
    # print(data)
    # print(''.join(data))
    return ''.join(data)


class ThreadSerial(qtc.QThread):
    data_ready2 = qtc.pyqtSignal(str)

    def __init__(self, parent=None):
        super(ThreadSerial, self).__init__(parent)
        self.is_running = True
        self.buffer_size = int(float(g.GCommand('MG bfsize')))

    def run(self):
        while True:
            time.sleep(0.2)
            if buffer_has_data():
                # Get head and tail pointers
                head = int(float(g.GCommand('MG head')))
                tail = int(float(g.GCommand('MG tail')))
                # Get the python list equivalent of the array
                data_list = g.GArrayUpload('buffer', 0, 399)
                # print(g.GCommand('QU buffer[], 0,399,1'))
                # variable = g.GCommand('QU buffer[],' + str(tail) + ',' + str(head) + ',1')
                self.data_ready2.emit(convert_buffer_data(data_list, self.buffer_size))
                g.GCommand('tail = ' + str(head))
                g.GCommand('is_empty = True')

    def stop(self):
        self.is_running = False
        print("stopping serial thread")
        self.terminate()


# Testing out using threads
class ThreadClass(qtc.QThread):
    any_signal = qtc.pyqtSignal(str)
    any_signal2 = qtc.pyqtSignal(str, str)

    def __init__(self, parent=None, index=0):
        super(ThreadClass, self).__init__(parent)
        self.index = index

    def run(self):
        while True:
            time.sleep(0.01)
            self.any_signal.emit(g.GCommand('MG @AN[3]'))
            self.any_signal2.emit(g.GCommand('MG _TPA'), g.GCommand('MG _TDA'))

    def stop(self):
        self.terminate()


if __name__ == '__main__':
    app = qtw.QApplication([])
    mw = UserWindow()
    mw.show()
    app.exec_()
