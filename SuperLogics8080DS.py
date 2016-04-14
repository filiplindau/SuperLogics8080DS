"""Created on 08 apr 2016

@author: Filip Lindau
"""
import sys
import PyTango
import Superlogics8080_control as sc
import threading
import logging
import time
import numpy as np
import Queue

logging.basicConfig(format='%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s', level=logging.INFO)


class Command:
    def __init__(self, command, data=None):
        self.command = command
        self.data = data

#==================================================================
#   Superlogics8080DS Class Description:
#
#         Control of a Superlogics8080 frequency counter
#
#==================================================================
#     Device States Description:
#
#   DevState.ON :       Connected to Halcyon driver
#   DevState.OFF :      Disconnected from Halcyon
#   DevState.FAULT :    Error detected
#   DevState.UNKNOWN :  Communication problem
#   DevState.MOVING :  Motor moving
#   DevState.INIT :     Initializing Halcyon driver.
#==================================================================


class Superlogics8080DS(PyTango.Device_4Impl):

#--------- Add your global variables here --------------------------

#------------------------------------------------------------------
#     Device constructor
#------------------------------------------------------------------
    def __init__(self, cl, name):
        PyTango.Device_4Impl.__init__(self, cl, name)
        Superlogics8080DS.init_device(self)

#------------------------------------------------------------------
#     Device destructor
#------------------------------------------------------------------
    def delete_device(self):
        with self.streamLock:
            self.info_stream(''.join(("[Device delete_device method] for device", self.get_name())))
        self.stopThread()


#------------------------------------------------------------------
#     Device initialization
#------------------------------------------------------------------
    def init_device(self):
        self.streamLock = threading.Lock()
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::init_device()")))
        self.set_state(PyTango.DevState.UNKNOWN)
        self.get_device_properties(self.get_device_class())

        # Try stopping the stateThread if it was started before. Will fail if this
        # is the initial start.
        try:
            self.stopThread()

        except Exception, e:
            pass


        self.attrLock = threading.Lock()
        self.eventIdList = []
        self.stateThread = threading.Thread()
        threading.Thread.__init__(self.stateThread, target=self.stateHandlerDispatcher)

        self.commandQueue = Queue.Queue(100)

        self.stateHandlerDict = {PyTango.DevState.ON: self.onHandler,
                                 PyTango.DevState.MOVING: self.onHandler,
                                 PyTango.DevState.ALARM: self.onHandler,
                                 PyTango.DevState.FAULT: self.onHandler,
                                 PyTango.DevState.INIT: self.initHandler,
                                 PyTango.DevState.UNKNOWN: self.unknownHandler,
                                 PyTango.DevState.OFF: self.offHandler}

        self.stopStateThreadFlag = False

        self.stateThread.start()

    def stateHandlerDispatcher(self):
        """Handles switch of states in the state machine thread.
        Each state handled method should exit by setting the next state,
        going back to this method. The previous state is also included when
        calling the next state handler method.
        The thread is stopped by setting the stopStateThreadFlag.
        """
        prevState = self.get_state()
        while self.stopStateThreadFlag == False:
            try:
                self.stateHandlerDict[self.get_state()](prevState)
                prevState = self.get_state()
            except KeyError:
                self.stateHandlerDict[PyTango.DevState.UNKNOWN](prevState)
                prevState = self.get_state()

    def stopThread(self):
        """Stops the state handler thread by setting the stopStateThreadFlag
        """
        self.stopStateThreadFlag = True
        self.stateThread.join(3)
        self.frequencyDevice.close()


    def unknownHandler(self, prevState):
        """Handles the UNKNOWN state, before communication with the hardware devices
        has been established. Here all devices are initialized.
        """
        with self.streamLock:
            self.info_stream('Entering unknownHandler')
        connectionTimeout = 1.0
        self.set_status('Connecting to frequency counter')

        # Need to connect to frequency counter

        while self.stopStateThreadFlag == False:
            # Frequency counter:
            try:
                with self.streamLock:
                    self.info_stream('Closing old frequency device')
                self.frequencyDevice.close()
            except:
                pass
            try:
                with self.streamLock:
                    self.info_stream(''.join(('Opening frequency device on port ', self.port)))
                self.frequencyDevice = sc.Superlogics8080_control(self.port)
            except Exception, e:
                with self.streamLock:
                    self.error_stream(''.join(('Could not connect to frequency counter ', self.port)))
                with self.streamLock:
                    self.error_stream(str(e))
                self.set_status('Could not connect to frequency counter')
                self.checkCommands(blockTime=connectionTimeout)
                continue
            self.set_state(PyTango.DevState.INIT)
            break

    def initHandler(self, prevState):
        """Handles the INIT state. Query Halcyon device to see if it is alive.
        """
        with self.streamLock:
            self.info_stream('Entering initHandler')
        waitTime = 1.0
        self.set_status('Initializing device')
        retries = 0
        maxTries = 5

        while self.stopStateThreadFlag == False:
            retries += 1
            if retries > maxTries:
                self.set_state(PyTango.DevState.UNKNOWN)
                break
            try:
                with self.streamLock:
                    self.info_stream('Trying to connect...')
                self.frequency = 0
                # Check configuration and set it to frequency if not:
                self.configuration = self.frequencyDevice.getConfiguration(01)
                if self.configuration[1] != 'frequency':
                    self.frequencyDevice.setConfiguration(01, 'frequency')
                    self.configuration = self.frequencyDevice.getConfiguration(01)

            except Exception, e:
                with self.streamLock:
                    self.error_stream(''.join(('Error when initializing device')))
                    self.error_stream(str(e))
                self.checkCommands(blockTime=waitTime)
                continue

            self.set_state(PyTango.DevState.ON)
            break

    def onHandler(self, prevState):
        """Handles the ON state. Connected to the Halcyon driver.
        Waits in a loop checking commands.
        """
        with self.streamLock:
            self.info_stream('Entering onHandler')
        handledStates = [PyTango.DevState.ON, PyTango.DevState.ALARM, PyTango.DevState.MOVING]
        waitTime = 0.1
        self.set_status('On')
        while self.stopStateThreadFlag == False:
            self.info_stream('onhandler loop')
            with self.attrLock:
                state = self.get_state()
            if state not in handledStates:
                break

            # Read frequency from frequency counter
            with self.attrLock:
                try:
                    self.info_stream('frequency check')
                    self.frequency = self.frequencyDevice.getFrequency(1)
                    self.info_stream('frequency ok')
                except Exception, e:
                    with self.streamLock:
                        self.error_stream(''.join(('Error reading frequency counter: ', str(e))))
                    self.set_state(PyTango.DevState.FAULT)
                    self.frequency = None

            self.checkCommands(blockTime=waitTime)


    def faultHandler(self, prevState):
        """Handles the FAULT state. A problem has been detected.
        """
        with self.streamLock:
            self.info_stream('Entering faultHandler')
        handledStates = [PyTango.DevState.FAULT]
        waitTime = 0.1
        retries = 0
        maxTries = 5

        while self.stopStateThreadFlag == False:
            if self.get_state() not in handledStates:
                break
            # Test frequency counter:
            try:
                status = self.frequencyDevice.getStatus(01)
                self.set_status(status)
                if status == 'host down':
                    self.frequencyDevice.resetStatus(01)
                self.configuration = self.frequencyDevice.getConfiguration(01)
                if self.configuration[1] != 'frequency':
                    self.frequencyDevice.setConfiguration(01, 'frequency')
                    self.configuration = self.frequencyDevice.getConfiguration(01)
                else:
                    self.set_state(PyTango.DevState.ON)

                with self.streamLock:
                    self.info_stream(''.join(('Frequency counter configuration: ', str(self.configuration))))

            except Exception, e:
                self.set_state(PyTango.DevState.UNKNOWN)

            if self.get_state() == PyTango.DevState.FAULT:
                retries += 1
                if retries > maxTries:
                    self.set_state(PyTango.DevState.UNKNOWN)
            self.checkCommands(blockTime=waitTime)

    def offHandler(self, prevState):
        """Handles the OFF state. Does nothing, just goes back to ON.
        """
        with self.streamLock:
            self.info_stream('Entering offHandler')
        self.set_state(PyTango.DevState.ON)


    def checkCommands(self, blockTime=0):
        """Checks the commandQueue for new commands. Must be called regularly.
        If the queue is empty the method exits immediately.
        """
#         with self.streamLock:
#             self.debug_stream('Entering checkCommands')
        try:
            if blockTime == 0:
                cmd = self.commandQueue.get(block=False)
            else:
                cmd = self.commandQueue.get(block=True, timeout=blockTime)

            if cmd.command == 'off':
                if self.get_state() not in [PyTango.DevState.INIT, PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.OFF)

            elif cmd.command == 'init':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.UNKNOWN)

            elif cmd.command == 'alarm':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ALARM)

            elif cmd.command == 'on':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ON)


        except Queue.Empty:
#             with self.streamLock:
#                 self.debug_stream('checkCommands: queue empty')
            pass

#------------------------------------------------------------------
#     Always excuted hook method
#------------------------------------------------------------------
    def always_executed_hook(self):
        pass


#------------------------------------------------------------------
#     Frequency attribute
#------------------------------------------------------------------
    def read_Frequency(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading frequency')))
        with self.attrLock:
            attr_read = self.frequency
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def is_Frequency_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True






#==================================================================
#
#     Superlogics8080DS command methods
#
#==================================================================

#------------------------------------------------------------------
#     On command:
#
#     Description: Start Halcyon driver
#
#------------------------------------------------------------------
    def On(self):
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::On")))
        cmdMsg = Command('on')
        self.commandQueue.put(cmdMsg)

#---- On command State Machine -----------------
    def is_On_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True



#==================================================================
#
#     Superlogics8080DSClass class definition
#
#==================================================================
class Superlogics8080DSClass(PyTango.DeviceClass):

    #     Class Properties
    class_property_list = {
        }


    #     Device Properties
    device_property_list = {
        'port':
            [PyTango.DevString,
            "Com port of the 8080 frequency counter",
            [  ] ],

        }


    #     Command definitions
    cmd_list = {
        'On':
            [[PyTango.DevVoid, ""],
            [PyTango.DevVoid, ""]],
        }


    #     Attribute definitions
    attr_list = {
        'Frequency':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description':"Detected frequency",
                'unit':'Hz',
            } ],
        }


#------------------------------------------------------------------
#     Superlogics8080DSClass Constructor
#------------------------------------------------------------------
    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name);
        print "In Superlogics8080DSClass  constructor"

#==================================================================
#
#     Superlogics8080DS class main method
#
#==================================================================
if __name__ == '__main__':
    try:
        py = PyTango.Util(sys.argv)
        py.add_class(Superlogics8080DSClass, Superlogics8080DS, 'Superlogics8080DS')

        U = PyTango.Util.instance()
        U.server_init()
        U.server_run()

    except PyTango.DevFailed, e:
        print '-------> Received a DevFailed exception:', e
    except Exception, e:
        print '-------> An unforeseen exception occured....', e
