%%关于三维转化二维的处理，
% 计算得到pitch 和roll，三轴加速度，然后去除重力。
%角速度一直用IMU的，然后速度没打滑，就用轮速传感器的。阿克曼模型，其水平角度，等于编码器算得的角度，乘上cos(pitch)
%如果打滑，不能用阿克曼，直接v=(v1+v1+a dt)/2来更新，角度取(θ+θ+w dt)/2,其中及速度。
%关于加速度的考虑，我们得到了小车坐标系XYZ方向的，但是我们认为X方向的才存在，因为小车。然后x_acc*cos(pitch)作为水平面加速度。

clc;
clear;
%% load the resources
load('..\main_120118_5\main_120118_5\DMC__A01.mat');
load('..\main_120118_5\main_120118_5\DMCVel01b.mat');
load('..\main_120118_5\main_120118_5\IMU_MS01.mat');
load('..\main_120118_5\main_120118_5\Table3DFrames.mat');
load('..\main_120118_5\main_120118_5\positionXYZ.mat');
plot3(position.x,position.y,position.z);
daspect([1 1 1]);

%% Parameter Setting:
%unit :standard unit including meter(m),second(s)
slip_threshold =0.2 ;
wheelDiameter = 0.315;
wheelRadius =wheelDiameter/2;
Axledist = 0.485;  
IMU_freq =200;
gravity = 9.98;
%steering param of UGV
Maingearratio = 91;
Bevelgearratio =1.25;
counterofWheel =227500; %a circle of wheel will produce 227500 pulses of encoder
Angleperpulse = 2*pi/counterofWheel;
dt = 5e-3;

%initialization
x0=0;y0=0;
yaw=0;
yawrate=0;
pos=[x0;y0;yaw];
pos = zeros(3,IMU.N);
vwstate =zeros(2,IMU.N);
pitch =zeros(1,IMU.N);
roll = zeros(1,IMU.N);
for i=2:IMU.N
    %get info from IMU
    x_acc = IMU.DATAf(1,i)*gravity;
    y_acc = IMU.DATAf(2,i) * gravity;
    z_acc = IMU.DATAf(3,i) *gravity;
    IMU_yawrate= IMU.DATAf(6,i);%t-run left ,positive,but imu is negative ,so 
    IMU_pitch = IMU.DATAf(5,i);
    IMU_roll = IMU.DATAf(4,i);
    
 

    pitch(i) = pitch(i-1) + IMU_pitch *1/IMU_freq;
    roll(i) = roll(i-1) + IMU_roll*1/IMU_freq;
    %remove gravity from acc of IMU
    a_removeG = removeG(x_acc,y_acc,z_acc,roll(i),pitch(i));
    x_acc =a_removeG(1);
    IMUtime = IMU.timesE(i);
    %get encoder data to calculate the yawrate and velocity
    dmcTimesList = dmc.timesE;
    IndexofEncoder = findIndex(IMUtime,dmcTimesList);
    LeftWheelSpeed = dmc.DATAf(5,IndexofEncoder)*Angleperpulse*wheelRadius;
    RightWheelSpeed = dmc.DATAf(6,IndexofEncoder)*Angleperpulse*wheelRadius;
    
    steerAngle = dmc.DATAf(1,IndexofEncoder)/counterofWheel*2*pi;
    VelfromEncoder1 = ((LeftWheelSpeed))/2;
    VelfromEncoder = VelDMC.speeds(IndexofEncoder) * cos(pitch(i));  
    YawratefromEncoder = VelfromEncoder/(Axledist/tan(steerAngle)) ;
    
    slipratio = cal_slip(YawratefromEncoder, IMU_yawrate+1e-10);
    %if slip srerious, Arkemann is not working
    slipping =(slipratio>slip_threshold);
    if 0
        disp('slip is serious,the encoder data is unuseful')
        w =  IMU_yawrate;
        v =  (2*vwstate(2,i-1)+ x_acc*dt*cos(pitch(i)))/2;
        %sliping ,Arckermann don't work
         vwstate(:,i) =[v,w];   
        pos(:,i) = simpleUpdatePos(pos(:,i-1),[v,w]);
    else
        disp('slip dont happen,fusion work!')
        %w = KalmanF(YawratefromEncoder,IMU_yawrate);
        %w =(YawratefromEncoder + IMU_yawrate)/2;  
        w = IMU_yawrate * cos(pitch(i));
        v = VelfromEncoder*cos(pitch(i));
         vwstate(:,i) =[v,w];   
        pos(:,i) =simpleUpdatePos(pos(:,i-1),[v,w]);  
    end
    
end
%% Visulization the result
figure(3)
plot(pos(1,:),pos(2,:),'r')
hold on;
grid on;
plot(Table3DFrames.CurrEstimatedPosition(1,:),Table3DFrames.CurrEstimatedPosition(2,:),'b-');
legend('Fusion of antiSlip','groundTruth');
%% functions 
%functiion: calc the slipratio
%input:yawrate from IMU and Encoder
    function slipratio = cal_slip(w_enc, w_imu)
      slipratio= abs((w_enc-w_imu)/w_imu);
    end
%function 
  function IndexofEncoder = findIndex(IMUtime,dmcTimeList)
 list = find(dmcTimeList<=IMUtime);
 IndexofEncoder =size(list,2);
  end
 
  %%Ackman model:update the postion and atttitude by the (w,v)
  %input:x=[x;y;z]  u=[v,w]
  function [x_new] = Ackermann(x,u)   
    t_diff=5e-3;  %duration of the updating
    %Ackerman motion model to compute new position after a command:yawrate  w and host speed v
    if u(2)==0
        u(2)=u(2)+1e-10;
    end
    x_new = x + [-(u(1)/u(2))*sin(x(3))+(u(1)/u(2))*sin(x(3)+u(2)*t_diff); (u(1)/u(2))*cos(x(3))-(u(1)/u(2))*cos(x(3)+u(2)*t_diff); u(2)*t_diff];
    %limit the car pose angle in [0,2*pi];
     x_new(3)=AngleLimit(x_new(3));
  end
  
  function x_new =simpleUpdatePos(x,u)
   t_diff=5e-3;  %duration of the updating
   x_new(3) =AngleLimit(x(3) + u(2)*t_diff);
   x_new(2) = x(2) + u(1)*t_diff*sin((x_new(3)+x(3))/2);
   x_new(1) = x(1) + u(1)*t_diff*cos((x_new(3)+x(3))/2);
  end

      
  %function AngleLimit limits the angel into the range(0,2*pi]
  function Angleout = AngleLimit(Anglein)
   if Anglein<0
        Angleout=Anglein+2*pi;
    elseif Anglein>2*pi
        Angleout = Anglein-2*pi;
    else
          Angleout = Anglein;
   end
  end

