#!/usr/bin/env python3
import socket
import re


class RobotFeedback:
    """Class for the Mecademic Robot allowing for live positional
    feedback of the Mecademic Robot.

    Attributes
    ----------
    address : string
        The IP address associated to the Mecademic robot.
    socket : socket
        Socket connecting to physical Mecademic Robot.
    robot_status : tuple of boolean
        States status bit of the robot.
    gripper_status : tuple of boolean
        States status bit of the gripper.
    joints : tuple of floats
        Joint angle in degrees of each joint starting from
        joint 1 going all way to joint 6.
    cartesian : tuple of floats
        The cartesian values in mm and degrees of the TRF.
    joints_vel : floats
        Velocity of joints.
    torque : tuple of floats
        Torque of joints.
    accelerometer : tuple of floats
        Acceleration of joints.
    last_msg_chunk : string
        Buffer of received messages.
    version : string
        Firmware version of the Mecademic Robot.
    version_regex : list of int
        Version_regex.

    """

    def __init__(self, address, firmware_version):
        """Constructor for an instance of the class Mecademic robot.

        Parameters
        ----------
        address : string
            The IP address associated to the Mecademic robot.
        firmware_version : string
            Firmware version of the Mecademic Robot.

        """
        self.address = address
        self.socket = None
        self.robot_status = ()
        self.gripper_status = ()
        self.joints = () #Joint Angles, angles in degrees | [theta_1, theta_2, ... theta_n]
        self.cartesian = () #Cartesian coordinates, distances in mm, angles in degrees | [x,y,z,alpha,beta,gamma]
        self.joints_vel =()
        self.torque =()
        self.accelerometer =()
        self.last_msg_chunk = ''
        a = re.search(r'(\d+)\.(\d+)\.(\d+)', firmware_version)
        self.version = a.group(0)
        self.version_regex = [int(a.group(1)), int(a.group(2)), int(a.group(3))]
        self.received_data = {} # Contains the most recent received data from the robot with the error code as keys

    def connect(self):
        """Connects Mecademic Robot object communication to the physical Mecademic Robot.

        Returns
        -------
        status : boolean
            Return whether the connection is established.

        """
        try:
            self.socket = socket.socket()
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY,1)
            self.socket.settimeout(1) #1s
            try:
                self.socket.connect((self.address, 10001)) #connect to the robot's address
            except socket.timeout: #catch if the robot is not connected to in time
                raise TimeoutError
            # Receive confirmation of connection
            if self.socket is None: #check that socket is not connected to nothing
                raise RuntimeError
            self.socket.settimeout(1) #1s
            try:
                if(self.version_regex[0] <= 7):
                    self.get_data()
                elif(self.version_regex[0] > 7): #RobotStatus and GripperStatus are sent on 10001 upon connecting from 8.x firmware
                    msg = self.socket.recv(256).decode('ascii') #read message from robot
                    self.received_data = self._convert_response(msg)  # Convert the response message to a dict
                    self._get_robot_status(msg)
                    self._get_gripper_status(msg)
                return True
            except socket.timeout:
                raise RuntimeError
        except TimeoutError:
            return False
        # OTHER USER !!!
        except RuntimeError:
            return False

    def disconnect(self):
        """Disconnects Mecademic Robot object from physical Mecademic Robot.

        """
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def get_data(self, delay=0.1):
        """Receives message from the Mecademic Robot and 
        saves the values in appropriate variables.

        Parameters
        ----------
        delay: int or float 
            Time to set for timeout of the socket.

        """
        if self.socket is None:                         #check that the connection is established
            return                                      #if no connection, nothing to receive
        self.socket.settimeout(delay)                   #set read timeout to desired delay
        try:
            raw_msg = self.socket.recv(256).decode('ascii')         #read message from robot
            self.received_data = self._convert_response(raw_msg)  # Convert the response message to a dict
            raw_response = raw_msg.split('\x00')                    # Split the data at \x00 to manage fragmented data
            raw_response[0] = self.last_msg_chunk + raw_response[0] # Merge the first data with last fragment from previous data stream
            self.last_msg_chunk = raw_response[-1]
            for response in raw_response[:-1]:
                if(self.version_regex[0] <= 7):
                    self._get_joints(response)
                    self._get_cartesian(response)
                elif(self.version_regex[0] > 7):
                    self._get_joints(response)
                    self._get_cartesian(response)
                    self._get_joints_vel(response)
                    self._get_torque_ratio(response)
                    self._get_accelerometer(response)
        except TimeoutError:
            pass

    def _get_robot_status(self, response):
        """Gets the values of RobotStatus bits from the message sent by
        the Robot upon connecting.
        Values saved to attribute robotstatus of the object.

        Parameters
        ----------
        response : string
            Message received from the Robot.

        """
        code = self._get_response_code('RobotStatus')
        try:
            for code_number in code:
                # Don't include the brackets when looking up the code number
                self.robot_status = self.received_data[code_number[1:-1]]
        except KeyError:
            pass  # Don't raise an error for compatibility with previous implementations.

    def _get_gripper_status(self, response):
        """Gets the values of GripperStatus bits from the message sent by
        the Robot upon connecting.
        Values saved to attribute robotstatus of the object.

        Parameters
        ----------
        response : string
            Message received from the robot.

        """
        code = self._get_response_code('GripperStatus')
        try:
            for code_number in code:
                # Don't include the brackets when looking up the code number
                self.gripper_status = self.received_data[code_number[1:-1]]
        except KeyError:
            pass  # Don't raise an error for compatibility with previous implementations.

    def _get_joints(self, response):
        """Gets the joint values of the variables from the message sent by the Robot.
        Values saved to attribute joints of the object.

        Parameters
        ----------
        response: string
            Message received from the Robot.

        """
        code = self._get_response_code('JointsPose')

        try:
            for code_number in code:
                # Don't include the brackets when looking up the code number
                self.joints = self.received_data[code_number[1:-1]]
        except KeyError:
            pass  # Don't raise an error for compatibility with previous implementations.

    def _get_cartesian(self, response):
        """Gets the cartesian values of the variables from the message sent by the Robot.
        Values saved to attribute cartesian of the object.

        Parameters
        ----------
        response : string
            Message received from the Robot.

        """
        code = self._get_response_code('CartesianPose')

        try:
            for code_number in code:
                # Don't include the brackets when looking up the code number
                self.cartesian = self.received_data[code_number[1:-1]]
        except KeyError:
            pass  # Don't raise an error for compatibility with previous implementations.

    def _get_joints_vel(self, response):
        """Gets the velocity values of the Joints from the message sent by the Robot.
        Values saved to attribute jointsvel of the object.

        Parameters
        ----------
        response : string
            Message received from the Robot.

        """
        code = self._get_response_code('JointsVel')

        try:
            for code_number in code:
                # Don't include the brackets when looking up the code number
                self.joints_vel = self.received_data[code_number[1:-1]]
        except KeyError:
            pass  # Don't raise an error for compatibility with previous implementations.

    def _get_torque_ratio(self, response):
        """Gets the torque ratio values of the Joints from the message sent by the Robot.
        Values saved to attribute torque of the object.

        Parameters
        ----------
        response : string
            Message received from the Robot.

        """
        code = self._get_response_code('TorqueRatio')

        try:
            for code_number in code:
                # Don't include the brackets when looking up the code number
                self.torque = self.received_data[code_number[1:-1]]
        except KeyError:
            pass  # Don't raise an error for compatibility with previous implementations.
    
    def _get_accelerometer(self,response):
        """Gets the accelerometers values from the message sent by the Robot.
        Values saved to attribute accelerometer of the object.

        Parameters
        ----------
        response : string
            Message received from the Robot.

        """
        code = self._get_response_code('AccelerometerData')

        try:
            for code_number in code:
                # Don't include the brackets when looking up the code number
                self.accelerometer = self.received_data[code_number[1:-1]]
        except KeyError:
            pass  # Don't raise an error for compatibility with previous implementations.

    def _get_response_code(self, param):
        """Retreives the response code for the parameters being streamed on port 100001.

        Parameters
        ----------
        param : string
            Parameter that needs to be extracted from raw data strem from Mecademic Robot.
            1. Robot Status {sent only once upon connecting on 10001}.
            2. Gripper Status {sent only once upon connecting on 10001}.
            3. Joints Pose feedback.
            4. Cartesian Pose feedback.
            5. Joints Velocity feedback.
            6. Torque Ratio.
            7. Accelerometer data.

        Returns
        --------
        answer_list : list of strings
            List of response codes to search for in the raw data stream.

        """
        if param.find('RobotStatus') != -1:
            return ['[2007]']
        elif param.find('GripperStatus')!= -1:
            return ['[2079]']
        elif param.find('JointsPose') != -1:
            if(self.version_regex[0] <= 7):
                return ['[2102]']
            elif(self.version_regex[0] > 7):
                return ['[2026]','[2210]']
        elif  param.find('CartesianPose') != -1:
            if(self.version_regex[0] <= 7):
                return ['[2103]']
            elif(self.version_regex[0] > 7):
                return ['[2027]','[2211]']
        elif param.find('JointsVel') != -1:
            return ['[2212]']
        elif param.find('TorqueRatio') != -1:
            return ['[2213]']
        elif param.find('AccelerometerData') != -1:
            return ['[2220]']
        else:
            return ['Invalid']

    def _convert_response(self, msg):

        """
        Method to convert and order the response message received from the robot. It uses regex to find valid
        response code/messages which are converted and returned into an ordered dict.

        :param msg (str): Entire message received from the robot (with or without the \x00 null character)
        :return (dict): Contains the response code as keys and the corresponding decoded message as a value.
        """
        # Find the individual responses with a valid format and convert those into a list
        msg_parts = re.findall("\[....\]\[.+?\]", msg)

        response_dict = {}
        # Put each individual response message/code in a dictonary
        for individual_response in msg_parts:
            response_code = individual_response[1:5]
            response_msg = individual_response[7:-1].split(',')  # Convert the response message to a list

            if len(response_msg) == 6:
                response_content = tuple((float(x) for x in response_msg))
            elif len(response_msg) == 7:
                response_content = tuple((float(x) for x in response_msg[1:]))   # Remove the timestamp
            else:
                response_content = ()  # If the length of the message is different set the content of to empty

            response_dict.update({response_code: response_content})

        return response_dict
