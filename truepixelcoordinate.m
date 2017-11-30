 vrep=remApi('remoteApi');
 vrep.simxFinish(-1);
 clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
[returnCode, camera] = vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
[returnCode, object] = vrep.simxGetObjectHandle(clientID,'Robbie_body#3',vrep.simx_opmode_blocking); 
[returnCode,resx]=vrep.simxGetObjectIntParameter(clientID,camera,vrep.sim_visionintparam_resolution_x,vrep.simx_opmode_streaming);
[returnCode,resy]=vrep.simxGetObjectIntParameter(clientID,camera,vrep.sim_visionintparam_resolution_y,vrep.simx_opmode_streaming);
[returnCode,perangle]=vrep.simxGetObjectFloatParameter(clientID,camera,1004,vrep.simx_opmode_streaming);
[returnCode,objposit]=vrep.simxGetObjectPosition(clientID,object,camera,vrep.simx_opmode_streaming); 
%  while true
%     [returnCode,objposit]=vrep.simxGetObjectPosition(clientID,object,camera,vrep.simx_opmode_buffer);  
%     px=0.5*double(resx)*(1+double(objposit(1))/double(objposit(3))/double(tan(0.5*perangle)));
%     py=0.5*double(resy)*(1+double(objposit(2))/double(objposit(3))/double(tan(0.5*perangle)));
%  end
figure(1)
plot(0,0,'.')
hold on
  while true
    resx=512;
    resy=512;
    perangle=60/180*pi;
    [returnCode,objposit]=vrep.simxGetObjectPosition(clientID,object,camera,vrep.simx_opmode_buffer);  
    px=0.5*double(resx)*(1-double(objposit(1))/double(objposit(3))/double(tan(0.5*perangle)));
    py=0.5*double(resy)*(1+double(objposit(2))/double(objposit(3))/double(tan(0.5*perangle)));
    plot(px,py,'.')
    pause(1)
 end
  