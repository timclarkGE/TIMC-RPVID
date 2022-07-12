# I stopped working on this code when I couldn't find a good solution to braking with electronic gearing

from MainGUIrA import Ui_MainWindow
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc
from XInput import *
import gclib
import sys
from simple_pid import PID

# from pyqtgraph import PlotWidget, plot
# import pyqtgraph as pg

from tkinter import *

g = gclib.py()
b = gclib.py()


class Galil_Widget:
    def __init__(self):
        self.g = gclib.py()
        self.g.GOpen('192.168.42.100 -s ALL')

    def gcmd(self, cmd):
        try:
            result = g.GCommand(cmd)
            return result
        except gclib.GclibError as e:
            if str(e) == 'question mark returned by controller':
                print("DMC Error: " + g.GCommand('TC1'))
            else:
                print('Unexpected GclibError:', e)

    def close(self):
        self.g.GClose()


class UserWindow(qtw.QMainWindow, Ui_MainWindow):
    count = 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)
        self.centerSlider.pressed.connect(lambda: self.horizontalSlider.setValue(0))
        self.centerSlider_2.pressed.connect(lambda: self.angle_setpoint_slider.setValue(2500))
        self.estop.pressed.connect(self.process_estop)
        self.enable.pressed.connect(self.process_enable)
        self.jog_fwd.pressed.connect(self.process_jog_fwd)
        self.jog_rev.pressed.connect(self.process_jog_rev)
        self.stop_movement.pressed.connect(self.stop_jog)
        self.jog_cw.pressed.connect(self.process_cw)
        self.jog_ccw.pressed.connect(self.process_ccw)

        self.brake_left.pressed.connect(self.process_left_brake)
        self.brake_right.pressed.connect(self.process_right_brake)
        self.brake_right.released.connect(self.restore_slider_after_braking)
        self.brake_left.released.connect(self.restore_slider_after_braking)
        self.slider_value_before_brake = 0

        self.jog_fwd.released.connect(self.stop_jog)
        self.jog_rev.released.connect(self.stop_jog)
        self.jog_cw.released.connect(self.stop_jog)
        self.jog_ccw.released.connect(self.stop_jog)

        self.incremental_move_active = False
        self.incremental_moves = ThreadDataFetch(['MG _TPA', 'MG _TDA'], index=1)
        self.incremental_moves.received_data.connect(self.update_error)

        self.horizontalSlider.valueChanged.connect(self.processSliderInput)
        self.velocity.valueChanged.connect(self.processSliderInput)
        self.last_slider_value = 0

        self.goto_button.pressed.connect(self.process_goto)
        self.pid = PID(20, 15, 0.01, setpoint=2.5)
        self.pid.output_limits = (0, 5)
        # self.file_x = open("output_x.txt", "w")
        # self.file_y = open("output_y.txt", "w")
        self.process_pid()
        self.set_pid.pressed.connect(self.process_pid)
        g.GOpen('192.168.42.100 -s ALL')
        b.GOpen('192.168.42.100 -s ALL')
        self.follower_target = 0

        # g.GProgramUploadFile('serial_communication.dmc')
        # g.GProgramUpload()
        g.GProgramDownloadFile('serial_communication.dmc', '')
        self.stop_program.setEnabled(False)

        self.mainUpdateThread = ThreadUpdate(parent=None)
        self.mainUpdateThread.start()
        self.mainUpdateThread.data_ready.connect(self.update_data)
        self.left_mtr_cur_avg = 0
        self.right_mtr_cur_avg = 0

        self.serialThread = ThreadSerial(parent=None)
        self.serialThread.reported_serial_data.connect(self.process_serial_data)
        self.process_start_program()

        self.inclinometer = ThreadDataFetch(['MG @AN[1]'], index=0)
        self.activateAngle.pressed.connect(self.process_activateAngle)
        self.deactivateAngle.pressed.connect(self.process_deactivateAngle)
        self.inclinometer.received_data.connect(self.average_vfbk)
        self.inclinometer.received_data.connect(lambda data: self.auto_angle_adjustment(data[0]))
        self.angle_setpoint_slider.valueChanged.connect(self.update_setpoint)
        self.running_average_vfbk = 2.5
        self.running_average_vcmd = 2.5

        self.start_program.pressed.connect(self.process_start_program)
        self.stop_program.pressed.connect(self.process_stop_program)
        self.help_cmds.pressed.connect(lambda: self.gcmd("MG {P2} \"H\"{^13}{N}"))
        self.reset_drive.pressed.connect(lambda: self.gcmd("MG {P2} \"R\"{^13}{N}"))
        self.clear_screen.pressed.connect(lambda: self.text_box.clear())
        self.send_cmd.pressed.connect(self.process_send_cmd)
        self.varedan_select.currentIndexChanged.connect(self.process_varedan_select)
        self.process_varedan_select()
        self.gamepad = MyGamepadThread()
        self.gamepad.start()
        self.gamepad.b_pressed.connect(self.brake_right.pressed)
        self.gamepad.x_pressed.connect(self.brake_left.pressed)
        self.gamepad.b_released.connect(self.brake_right.released)
        self.gamepad.x_released.connect(self.brake_left.released)
        self.gamepad.velocity_set.connect(lambda data: self.velocity.setValue(abs(int(150 * data))))
        self.gamepad.right_thumb_deflected_left.connect(self.jog_ccw.pressed)
        self.gamepad.right_thumb_deflected_right.connect(self.jog_cw.pressed)
        self.gamepad.right_thumb_zero_deflection.connect(self.stop_jog)
        self.gamepad.gamepad_disconnected.connect(self.stop_jog)


        self.activate_gamepad.pressed.connect(self.process_activate_gamepad)

    # def update_error_msg(self, text):
    #     if self.sender().index == 0:
    #         self.error_mg.setText(str(int(float(text))))
    #     elif self.sender().index == 1:
    #         self.error_mg.setText(str(text))
    #     elif self.sender().index == 2:
    #         self.error_mg.setText(str(int(float(text))))

    def process_left_brake(self):
        self.slider_value_before_brake = self.horizontalSlider.value()
        for i in range(50):
            self.horizontalSlider.setValue(-1 * i)
        # self.gcmd('GR 0,0,0,0')
        # self.gcmd('SP 0')
        # self.horizontalSlider.setValue(-50)

    def process_right_brake(self):
        self.slider_value_before_brake = self.horizontalSlider.value()
        self.horizontalSlider.setValue(50)

    def restore_slider_after_braking(self):
        # if self.horizontalSlider.value() < 0:
        #     self.gcmd('GR 0,-1,0,0')

        #     start = self.horizontalSlider.value()
        #     end = self.slider_value_before_brake
        #     for i in range(start,end):
        #         self.horizontalSlider.setValue(i)
        # else:
        self.horizontalSlider.setValue(self.slider_value_before_brake)

    def process_activate_gamepad(self):
        if self.activate_gamepad.isChecked():
            self.enable_jog_buttons()
            self.goto_button.setEnabled(True)
        else:
            self.disable_jog_buttons()
            self.goto_button.setEnabled(False)

    def process_activateAngle(self):
        self.inclinometer.start()
        self.activateAngle.setEnabled(False)
        self.deactivateAngle.setEnabled(True)
        self.update_setpoint(self.angle_setpoint_slider.value())

    def process_deactivateAngle(self):
        self.inclinometer.stop()
        self.activateAngle.setEnabled(True)
        self.deactivateAngle.setEnabled(False)
        self.horizontalSlider.setValue(0)
        self.voltageSetpoint.setText("(V)")
        self.voltageCommand.setText("(V)")
        self.voltageActual.setText("(V)")

    def update_setpoint(self, sp):
        self.pid.setpoint = sp / 1000
        self.voltageSetpoint.setText(str('{:.2f}'.format(self.pid.setpoint)))

    def average_vfbk(self, voltage):
        N = 15
        # Calculate moving average of raw voltage output of tilt sensor
        self.running_average_vfbk -= self.running_average_vfbk / N
        self.running_average_vfbk += round(float(voltage[0]) / N, 4)
        self.voltageActual.setText(str('{:.2f}'.format(self.running_average_vfbk)))

    def process_pid(self):
        if self.kp_edit.text() == '':
            self.kp_edit.setText(str(self.pid.Kp))
            self.ki_edit.setText(str(self.pid.Ki))
            self.kd_edit.setText(str(self.pid.Kd))
        else:
            self.pid.Kp = float(self.kp_edit.text())
            self.pid.Ki = float(self.ki_edit.text())
            self.pid.Kd = float(self.kd_edit.text())

    def gcmd(self, cmd):
        try:
            result = g.GCommand(cmd)
            self.error_mg.setText("")
            return result
        except gclib.GclibError as e:
            if str(e) == 'question mark returned by controller':
                self.error_mg.setText("DMC Error: " + g.GCommand('TC1'))
            else:
                print('Unexpected GclibError:', e)

    def process_start_program(self):
        self.gcmd('XQ #I_SCOM,0')
        self.serialThread.start()
        self.start_program.setEnabled(False)
        self.stop_program.setEnabled(True)

    def process_stop_program(self):
        self.gcmd('HX #I_SCOM, 0')
        self.serialThread.stop()
        self.start_program.setEnabled(True)

    def process_serial_data(self, key):
        self.text_box.appendPlainText(key)

    def process_send_cmd(self):
        cmd_str = "MG {P2} \"" + self.cmd_text.text() + "\"{^13}{N}"
        self.gcmd(cmd_str)

    def process_varedan_select(self):
        if self.varedan_select.currentIndex() == 0:
            self.gcmd('CB 21')
            self.gcmd('SB 18')
        elif self.varedan_select.currentIndex() == 1:
            self.gcmd('CB 18')
            self.gcmd('SB 21')
        else:
            self.gcmd('CB 18')
            self.gcmd('CB 21')

    def process_enable(self):
        # Check if the motors are off (_MOx = 1) and then turn on servo control
        if int(float(self.gcmd('MG _MOA'))) and int(float(self.gcmd('MG _MOB'))):
            self.gcmd('CB 24')  # Enables motor bus voltage
            self.gcmd('GA ,CA,,')
            self.processSliderInput()
            self.gcmd('GM 1,1,0,0')  # turn on gantry mode
            self.gcmd('SH A,B')  # Axis A and B enable goes to an AND gate, both must enable to enable LA-310
            self.enable.setStyleSheet("background-color: green")

        elif int(float(self.gcmd('MG _MOA'))) == 0 and int(float(self.gcmd('MG _MOB'))) == 0:
            self.enable.setStyleSheet("background-color: light grey")
            self.gcmd('MOA')
            self.gcmd('MOB')
            self.gcmd('SB 24')
        else:
            self.gcmd('MOA')
            self.gcmd('MOB')

    def process_jog_rev(self):
        self.processSliderInput()
        self.gcmd('BG A')

    def process_jog_fwd(self):
        self.processSliderInput()
        result = -1 * float(self.gcmd('JGA = ?'))
        self.gcmd('JG ' + str(result))
        self.gcmd('BG A')

    def stop_jog(self):
        self.gcmd('ST A,B,C,D')
        if self.incremental_moves.is_running:
            self.stop_inc_move()

    def process_cw(self):
        self.gcmd('GR 0,1,0,0')
        velocity = str(-int(self.velocity.value()) * 1000)
        self.gcmd('JG ' + velocity)
        result = self.gcmd('BG A')

    def process_ccw(self):
        self.gcmd('GR 0,1,0,0')
        velocity = str(int(self.velocity.value()) * 1000)
        self.gcmd('JG ' + velocity)
        self.gcmd('BG A')

    # Differential speed to counter drift
    def processSliderInput(self):
        value = self.horizontalSlider.value()
        offset = value / 51
        # When turning left, left axis goes slower and the gear ratio must be increased
        # There is a problem when the slider is change quickly from zero to -50, the motor jumps fast
        # print(value, self.last_slider_value)
        if -99 <= value < 0:
            if abs(value - self.last_slider_value) > 10:
                value = self.last_slider_value - 1
                offset = value / 51
                self.gcmd('SP ' + str(int(self.velocity.value()) * 1000 * (1 - abs(offset))))
                self.gcmd('GR ,' + str(-1 / (1 - abs(offset))) + ',,')
                print("Output0:", value, 'GR ,' + str(-1 / (1 - abs(offset))) + ',,')
            else:
                # print("Output1:", value, 'GR ,' + str(-1 / (1 - abs(offset))) + ',,')
                self.gcmd('SP ' + str(int(self.velocity.value()) * 1000 * (1 - abs(offset))))
                # print('GR ,' + str(-1 / (1 - abs(offset))) + ',,')
                self.gcmd('GR ,' + str(-1 / (1 - abs(offset))) + ',,')
            if int(float(self.gcmd('PTA = ?'))) == 0:
                self.gcmd('JG ' + str(int(self.velocity.value()) * 1000 * (1 - abs(offset))))
                # Update the SP if incremental move is happening

        # When turning right, right goes slower because of decreased gear ratio
        elif 0 <= value <= 99:
            # print('GR ,' + str(-1 + offset) + ',,')
            self.gcmd('SP ' + str(int(self.velocity.value()) * 1000))
            self.gcmd('GR ,' + str(-1 + offset) + ',,')  # ratio needs to always be negative
            if int(float(self.gcmd('PTA = ?'))) == 0:
                self.gcmd('JG ' + str(int(self.velocity.value()) * 1000))
            # Update the SP if incremental move is happening
        self.last_slider_value = value

    def process_estop(self):
        self.gcmd('ST A,B,C,D')
        self.gcmd('SB 24')
        self.process_enable()

    def process_goto(self):
        self.gcmd('GA ,CA,,')
        self.gcmd('GR ,-1,,')
        self.gcmd('DE 0,0,0,0')
        self.gcmd('DP 0,0,0,0')
        self.gcmd('SH A,B')
        self.gcmd('PT 1')
        self.disable_jog_buttons()
        self.start_inc_move()

    def disable_jog_buttons(self):
        self.jog_cw.setEnabled(False)
        self.jog_ccw.setEnabled(False)
        self.jog_fwd.setEnabled(False)
        self.jog_rev.setEnabled(False)
        self.brake_left.setEnabled(False)
        self.brake_right.setEnabled(False)

    def enable_jog_buttons(self):
        self.jog_cw.setEnabled(True)
        self.jog_ccw.setEnabled(True)
        self.jog_fwd.setEnabled(True)
        self.jog_rev.setEnabled(True)
        self.brake_left.setEnabled(True)
        self.brake_right.setEnabled(True)

    # Thread to constantly update PT command for incremental move
    def start_inc_move(self):
        self.incremental_moves.start()
        self.follower_target = float(self.goto_position.text()) * 1382

    def stop_inc_move(self):
        self.incremental_moves.stop()
        self.enable_jog_buttons()

    # Calculates new PT position needed for incremental move
    def update_error(self, data):
        encoder = float(data[0])
        follower = float(data[1])
        ratio = 13.766  # Motor encoder counts divided by follower encoder counts
        # Calculate motor encoder counts to get to follower set point
        delta_counts = ((self.follower_target - follower) * ratio)
        # Check if the difference is larger than slider and issue an update
        if abs(delta_counts) > 15:
            self.gcmd('PA ' + str(encoder + delta_counts))
        else:
            self.stop_inc_move()

    # Tool angle adjustment calculation
    def auto_angle_adjustment(self, voltage):
        voltage = float(voltage)
        newSliderValue = self.pid(voltage)
        self.horizontalSlider.setValue(int((newSliderValue - 2.5) * 20))
        self.processSliderInput()

        N = 30
        # Calculate moving average of command voltage for display purposes
        self.running_average_vcmd -= self.running_average_vcmd / N
        self.running_average_vcmd += round(float(newSliderValue) / N, 4)
        self.voltageCommand.setText(str('{:.2f}'.format(self.running_average_vcmd)))

    def closeEvent(self, event):
        self.process_estop()
        self.gcmd('RS')
        g.GClose()
        self.file_x.close()
        self.file_y.close()

    # Update the labels on the GUI for the position, velocity, error, current
    def update_data(self, data):
        self.label_left_position.setText(str('{:.2f}'.format(round(float(data["left pos"][0]) / 19771, 2))))
        self.label_right_position.setText(str('{:.2f}'.format(round(float(data["right pos"][0]) / 19771, 2))))
        self.label_follower_position.setText(str('{:.2f}'.format(round(float(data["follower pos"][0]) / 1382, 2))))
        self.label_velocity.setText(str('{:.2f}'.format(round(float(data["velocity"][0]) / 19771, 2))))
        self.label_left_position_error.setText(str('{:.2f}'.format(round(float(data["left pos err"][0]) / 19771, 2))))
        self.label_right_position_error.setText(str('{:.2f}'.format(round(float(data["right pos err"][0]) / 19771, 2))))
        self.fault_index.setChecked(not int(round(float(data["index fault"][0]))))
        self.fault_scan.setChecked(not int(round(float(data["scan fault"][0]))))
        # self.fault_index.setChecked(bool(round(float(data["index fault"][0]))))
        # self.fault_scan.setChecked(bool(round(float(data["scan fault"][0]))))

        N = 25
        # Calculate moving average
        self.left_mtr_cur_avg -= self.left_mtr_cur_avg / N
        self.left_mtr_cur_avg += round(float(data["left mtr cur"][0]) / N, 5)

        self.right_mtr_cur_avg -= self.right_mtr_cur_avg / N
        self.right_mtr_cur_avg += round(float(data["right mtr cur"][0]) / N, 5)

        # Update current on GUI
        self.label_left_motor_current.setText(str('{:.2f}'.format(self.left_mtr_cur_avg)))
        self.label_right_motor_current.setText(str('{:.2f}'.format(self.right_mtr_cur_avg)))


# Thread to gather information to update the GUI with position, velocity, error, and motor current
class ThreadUpdate(qtc.QThread):
    data_ready = qtc.pyqtSignal(dict)

    def __init__(self, parent=None):
        super(ThreadUpdate, self).__init__(parent)
        self.is_running = False
        self.data_packet = {
            "left pos": ['N/A', 'MG _TPA'],
            "right pos": ['N/A', 'MG _TPB'],
            "follower pos": ['N/A', 'MG _TDA'],
            "velocity": ['N/A', 'MG _TVA'],
            "left pos err": ['N/A', 'MG _TEA'],
            "right pos err": ['N/A', 'MG _TEB'],
            "left mtr cur": ['N/A', 'MG _TTA'],
            "right mtr cur": ['N/A', 'MG _TTB'],
            "index fault": ['N/A', 'MG @IN[1]'],
            "scan fault": ['N/A', 'MG @IN[2]']
        }
        self.g = gclib.py()
        self.g.GOpen('192.168.42.100 -s ALL')

    def run(self):
        self.is_running = True
        while True:
            time.sleep(0.01)
            for key in self.data_packet:
                self.data_packet[key][0] = self.g.GCommand(self.data_packet[key][1])
            self.data_ready.emit(self.data_packet)

    def stop(self):
        self.is_running = False
        print("stopping update thread")
        self.terminate()


# Thread for serial connection with LA 415 via AUX port on DMC 4040
def buffer_has_data():
    # List the variables created in the DMC-4040
    variables = b.GCommand('LV')
    # Check data set ready variable
    data_ready_flag = int(float(b.GCommand('MG dsr')))
    if 'is_empty' in variables:
        # Buffer is not empty
        if int(float(b.GCommand('MG is_empty'))) == 0 and data_ready_flag:
            return True
        # Buffer is empty
        elif int(float(b.GCommand('MG is_empty'))) == 1:
            return False
    # Variable 'is_empty' is not present, program not loaded on DMC 4040
    else:
        return False


def convert_buffer_data(data: list):
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

    # Return a string of the result
    return ''.join(data)


class ThreadSerial(qtc.QThread):
    reported_serial_data = qtc.pyqtSignal(str)

    def __init__(self, parent=None):
        super(ThreadSerial, self).__init__(parent)
        self.is_running = False

    def run(self):
        self.is_running = True
        while True:
            time.sleep(0.2)
            if buffer_has_data():
                # Get head and tail pointers
                head = int(float(b.GCommand('MG head')))
                tail = int(float(b.GCommand('MG tail')))
                buffer_size = int(float(b.GCommand('MG bfsize')))
                # Get the python list equivalent of the array
                # If tail is > head, then the data overlaps index 0
                if tail > head:
                    data_list = b.GArrayUpload('buffer', tail, (buffer_size - 1))
                    data_list.extend(b.GArrayUpload('buffer', 0, head - 1))
                    self.reported_serial_data.emit(convert_buffer_data(data_list))
                else:
                    data_list = b.GArrayUpload('buffer', tail, head - 1)
                    self.reported_serial_data.emit(convert_buffer_data(data_list))
                b.GCommand('tail = ' + str(head))
                b.GCommand('is_empty = True')

    def stop(self):
        self.is_running = False
        print("stopping serial thread")
        self.terminate()


# Thread to
class ThreadClass(qtc.QThread):
    reported_encoder_positions = qtc.pyqtSignal(str, str)

    def __init__(self, parent=None):
        super(ThreadClass, self).__init__(parent)
        self.is_running = False
        self.g = gclib.py()
        self.g.GOpen('192.168.42.100 -s ALL')

    def run(self):
        self.is_running = True
        while True:
            time.sleep(0.01)
            self.reported_encoder_positions.emit(self.g.GCommand('MG _TPA'), self.g.GCommand('MG _TDA'))

    def stop(self):
        self.terminate()
        self.is_running = False


class ThreadDataFetch(qtc.QThread):
    received_data = qtc.pyqtSignal(list)
    refresh_wait = 0.01

    def __init__(self, commands, index=0):
        super(ThreadDataFetch, self).__init__()
        self.is_running = False
        self.index = index
        self.connection = Galil_Widget()
        self.cmd_list = []
        for i in range(len(commands)):
            self.cmd_list.append(commands[i])

    def run(self):
        self.is_running = True
        data = []
        while True:
            data.clear()
            time.sleep(self.refresh_wait)
            for i in range(len(self.cmd_list)):
                data.append(self.connection.gcmd(self.cmd_list[i]))
            self.received_data.emit(data)

    def stop(self):
        self.terminate()
        self.is_running = False
        print("ThreadDataFetch Terminate: ", self.senderSignalIndex())
        self.connection.g.GClose()


class MyGamepadThread(qtc.QThread):
    x_pressed = qtc.pyqtSignal()
    x_released = qtc.pyqtSignal()
    b_pressed = qtc.pyqtSignal()
    b_released = qtc.pyqtSignal()
    velocity_set = qtc.pyqtSignal(float)
    right_thumb_deflected_right = qtc.pyqtSignal()
    right_thumb_deflected_left = qtc.pyqtSignal()
    right_thumb_zero_deflection = qtc.pyqtSignal()

    gamepad_disconnected = qtc.pyqtSignal()
    refresh_wait = 0.01

    def __init__(self):
        super(MyGamepadThread, self).__init__()
        self.connected_last = tuple((False, False, False, False))
        self.btn_state_last = None
        self.thumb_values_last = None
        self.rotation_state = 0

    def run(self):
        while True:
            time.sleep(self.refresh_wait)
            try:
                query_connections = get_connected()
                if query_connections.count(True) == 1:
                    index = query_connections.index(True)
                    state = get_state(index)
                    btn_state = get_button_values(state)
                    thumb_values_state = get_thumb_values(state)

                    if self.btn_state_last is not None and self.thumb_values_last is not None:
                        if btn_state['X'] != self.btn_state_last['X']:
                            if btn_state['X']:
                                self.x_pressed.emit()
                            else:
                                self.x_released.emit()
                        if btn_state['B'] != self.btn_state_last['B']:
                            if btn_state['B']:
                                self.b_pressed.emit()
                            else:
                                self.b_released.emit()
                        if thumb_values_state[1][0] != self.thumb_values_last[1][0]:
                            rotation_variable = thumb_values_state[1][0]
                            print(rotation_variable)
                            self.velocity_set.emit(rotation_variable)
                            # CW rotation commanded
                            if rotation_variable > 0:
                                self.rotation_state = 1
                                self.right_thumb_deflected_right.emit()
                                print("CW")
                            elif rotation_variable == 0 and self.rotation_state != 0:
                                self.rotation_state = 0
                                self.right_thumb_zero_deflection.emit()
                                print("Stopped")
                            elif rotation_variable < 0:
                                self.rotation_state = -1
                                self.right_thumb_deflected_left.emit()
                                print("CCW")

                    self.thumb_values_last = thumb_values_state
                    self.btn_state_last = btn_state

            except:
                print("Problem with Gamepad")

            if query_connections != self.connected_last:
                new_count = query_connections.count(True)
                old_count = self.connected_last.count(True)
                if new_count < old_count:
                    self.gamepad_disconnected.emit()
                elif new_count > 1:
                    self.gamepad_disconnected.emit()
                self.connected_last = query_connections

    def stop(self):
        self.terminate()
        self.gamepad_disconnected.emit()
        self.connected_last = tuple((False, False, False, False))
        self.btn_state_last = None
        self.thumb_values_last = None


if __name__ == '__main__':
    app = qtw.QApplication([])
    mw = UserWindow()
    mw.show()
    app.exec_()
