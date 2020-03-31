function functionTest

dmclist=[12,33,45,66,77,99,100,102,122,144];
IMUtime=124;
a = findIndex(IMUtime,dmclist)
 function IndexofEncoder = findIndex(IMUtime,dmcTimeList)
 list = find(dmcTimeList<=IMUtime);
 IndexofEncoder =size(list,2);
 end
dmclist(a)
c = Ackermann([0;0;0],[1;0])
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

end
