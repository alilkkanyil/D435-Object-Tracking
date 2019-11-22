%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2017 ROBOTIS CO., LTD.
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author: Ryu Woon Jung (Leon)

%
% *********     Bulk Read and Bulk Write Example      *********
%
%
% Available Dynamixel model on this example : All models using Protocol 2.0
% This example is designed for using two Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
% To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
% Be sure that Dynamixel PRO properties are already set as %% ID : 1 and 2 / Baudnum : 1 (Baudrate : 57600)
%

clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    addpath(fullfile("C:/","Users","smith","Documents","GitHub","DynamixelSDK","matlab"))
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
end

global DXL_MINIMUM_POSITION_VALUE;
global DXL_MAXIMUM_POSITION_VALUE;

global end_effector;
global base_link;

% Control table address
ADDR_TORQUE_ENABLE          = 64;          % Control table address is different in Dynamixel model
ADDR_LED_RED                = 65;
ADDR_GOAL_POSITION          = 116;
ADDR_PRESENT_POSITION       = 132;
ADDR_RETURN_DELAY_TIME      = 8;

% Data Byte Length
LEN_PRO_LED_RED                 = 1;
LEN_PRO_GOAL_POSITION           = 4;
LEN_PRO_PRESENT_POSITION        = 4;

% Protocol version
PROTOCOL_VERSION                = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_IDS = [0 1 2 3 4 5];        % Goal position
DXL_POS = [0 0 0 0 0 0];        % Present position
DXL_STATUS = [0 0 0 0 0 0];     % Dynamixel moving status

BAUDRATE                        = 1000000;
DEVICENAME                      = 'COM3';       % Check which port is being used on your controller
                                                % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE                   = 1;            % Value for enabling the torque
TORQUE_DISABLE                  = 0;            % Value for disabling the torque
RETURN_DELAY_TIME               = 10;            % Amount of time(2us) Dynamixel waits before replying
DXL_MINIMUM_POSITION_VALUE      = 0;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE      = 4095;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD     = 20;           % Dynamixel moving status threshold

ESC_CHARACTER                   = 'e';          % Key for escaping loop

COMM_SUCCESS                    = 0;            % Communication Success result value
COMM_TX_FAIL                    = -1001;        % Communication Tx 
COMM_PORT_BUSY                  = -1000;        % Communication port was busy

end_effector = 'wrist_link';
base_link = 'base_link';

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;                 % Communication result
dxl_getdata_result = false;                     % GetParam result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                                  % Dynamixel error
dxl_led_value = [0 255];                        % Dynamixel LED value for write

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open port %d!\n',port_num);
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Enable all servos
for i = [1 2 3]
    EnableTorque(DXL_IDS(i), port_num, PROTOCOL_VERSION);
end

while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end
    
    id = input('Enter servo #:');
    
    if (id >= 0 && id<= 2)
        pos = input('Goal position:');
        write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_GOAL_POSITION, uint16(pos));
    end
end

for i = [1 2 3]
    % Disable Dynamixel Torque
    DisableTorque(DXL_IDS(i), port_num, PROTOCOL_VERSION);
end

% Close port
closePort(port_num);
disp("Closed port")

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;

function b = EnableTorque(id, port, protocol)
    write1ByteTxRx(port, protocol, id, 64, 1);
    
    dxl_comm_result = getLastTxRxResult(port, protocol);
    dxl_error = getLastRxPacketError(port, protocol);
    if dxl_comm_result ~= 0
        fprintf('%s %d\n', getTxRxResult(protocol, dxl_comm_result),dxl_comm_result);
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(protocol, dxl_error));
    else
        fprintf('Dynamixel %d has been successfully connected \n', id);
    end
        
    b = dxl_comm_result & dxl_error;
end

function b = DisableTorque(id, port, protocol)
    write1ByteTxRx(port, protocol, id, 64, 0);
    
    dxl_comm_result = getLastTxRxResult(port, protocol);
    dxl_error = getLastRxPacketError(port, protocol);
    if dxl_comm_result ~= 0
        fprintf('%s %d\n', getTxRxResult(protocol, dxl_comm_result),dxl_comm_result);
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(protocol, dxl_error));
    else
        fprintf('Dynamixel %d has been disconnected \n', id);
    end
        
    b = dxl_comm_result & dxl_error;
end

