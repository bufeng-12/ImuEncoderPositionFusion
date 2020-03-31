% Remove gravity from X, Y, Z components 
% https://stackoverrun.com/cn/q/7892932
function [a_processed] = removeG(ax,ay,az,roll,pitch)

r0=0;p0=0;h0=0;
refPlane = [r0 p0 h0]; % [2deg 4deg 60deg] IMU was not level during data collection. 

       deltaAngleX = roll - refPlane(1); %degrees
       deltaAngleY = pitch - refPlane(2); %degrees

       if ( deltaAngleX > 0) % roll ++, ay --, az ++
           cX_X = 0;
           cY_X = -sind ( deltaAngleX ) ;
           cZ_X = cosd ( deltaAngleX ) ;
       elseif ( deltaAngleX < 0 )
           cX_X = 0;
           cY_X = sind ( deltaAngleX ) ;
           cZ_X = cosd ( deltaAngleX ) ;
       end

       if ( deltaAngleY > 0 ) % roll ++, ay --, az ++
           cX_Y = sind ( deltaAngleY ) ;
           cY_Y = 0 ;
           cZ_Y = sind ( deltaAngleY ) ;
       elseif ( deltaAngleY < 0 )
           cX_Y = -sind ( deltaAngleY ) ;
           cY_Y = 0 ;
           cZ_Y = sind ( deltaAngleY ) ;
       end

       ax = ax + cX_X + cX_Y;
       ay = ay + cY_X + cY_Y;
       az = az + cZ_X + cZ_Y;
       a_processed=[ax;ay;az];

end