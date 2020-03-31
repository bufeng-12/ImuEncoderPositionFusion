clc;
clear;
%% load the resources
load('..\main_indoor\main\DMC__A01.mat')
load('..\main_indoor\main\DMCVel01b.mat')
load('..\main_indoor\main\IMU_MS01.mat')
load('..\main_indoor\main\Table3DFrames.mat')
figure(1)
plot3(Table3DFrames.CurrEstimatedPosition(1,:),Table3DFrames.CurrEstimatedPosition(2,:),Table3DFrames.CurrEstimatedPosition(3,:))
hold on
%load('..\main_111202_2\main_111202_2\positionXYZ.mat')
%plot3(position.x ,position.y, position.z)
grid on;
daspect([1 1 1]);
%% Parameter Setting:
%unit :standard unit including meter(m),second(s)
slip_threshold = 1;
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
for i=2:IMU.N
    %get info from IMU
    x_acc = IMU.DATAf(1,i)*gravity;
    IMU_yawrate= -IMU.DATAf(6,i);%trun left ,positive,but imu is negative ,so 
    IMU_pitch = IMU.DATAf(5,i);
    pitch(i) = pitch(i-1) + IMU_pitch *1/IMU_freq;
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
        v =  (2*vwstate(2,i-1)+ x_acc*dt)/2;
    else
        disp('slip dont happen,fusion work!')
        %w = KalmanF(YawratefromEncoder,IMU_yawrate);
        %w =(YawratefromEncoder + IMU_yawrate)/2;  
        w = IMU_yawrate;
        v = VelfromEncoder;
    end
%     v= VelfromEncoder;
%     w = YawratefromEncoder;
    vwstate(:,i) =[v,w];    
    %Arkeman model to update the attitude and position  
    %pos(:,i) = Ackermann(pos(:,i-1),[v,w]);  
    pos(:,i) = simpleUpdatePos(pos(:,i-1),[v,w]);
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