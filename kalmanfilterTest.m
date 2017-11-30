
kalmanObjTracking();

function kalmanObjTracking

 vrep=remApi('remoteApi');
 vrep.simxFinish(-1);
 clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
frame            = [];  % A video frame
detectedLocation = [];  % The detected location
trackedLocation  = [];  % The tracked location
label            = '';  % Label for the ball
utilities        = [];  % Utilities used to process the video
kalmanFilter = [];
preDetection = [];      % Location of dectected object in previous frame
adjSensibility = 200;
px= [];
py= [];
% Variables for camera rotation algorithm
cameraAngle = 0.0;
cameraAngleSpeed = 0.002;

 if (clientID>-1)      
      disp('Connected')
      
      %Handle
      [returnCode, camera] = vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
      [returnCode2, motor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_cameraMotor',vrep.simx_opmode_blocking);
      [returnCode3, prox]  = vrep.simxGetObjectHandle(clientID,'psensor#3',vrep.simx_opmode_blocking);
      [rc1, lmotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
      [rc1, rmotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
      
      [returnCode, object] = vrep.simxGetObjectHandle(clientID,'Robbie_body#3',vrep.simx_opmode_blocking); 
      [returnCode,perangle]=vrep.simxGetObjectFloatParameter(clientID,camera,1004,vrep.simx_opmode_streaming);
      [returnCode,objposit]=vrep.simxGetObjectPosition(clientID,object,camera,vrep.simx_opmode_streaming); 

     
            
      %Other code
      [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);
      param = getDefaultParameters();         % get parameters that work well
      param.motionModel = 'ConstantVelocity'; % switch from ConstantAcceleration
                                              % to ConstantVelocity
                                              % After switching motion models, drop noise specification entries
                                              % corresponding to acceleration.
      param.initialEstimateError = param.initialEstimateError(1:2);
      param.motionNoise          = param.motionNoise(1:2);
      
      utilities = createUtilities(param);
      isTrackInitialized = false;
      
      while true
           [returnCode,resolution,image] = vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_buffer);
           [clientIDandSensorHandle detectionState detectedPoint detectedObjectHandle detectedSurfaceNormalVector] = vrep.simxReadProximitySensor(clientID, prox, vrep.simx_opmode_streaming);
           dist = sqrt(detectedPoint(1)^2 + detectedPoint(2)^2 + detectedPoint(3)^2);
           param.image = image;
           trackSingleObject(param); % visualize the results
                      
           % Camera rotation algorithm
           if length(detectedLocation) == 2
               % disp( detectedLocation(1) );
               perangle=60/180*pi;
               [returnCode,objposit] = vrep.simxGetObjectPosition(clientID,object,camera,vrep.simx_opmode_buffer);  
               % px py : actual object pixel point
               px=0.5*double(512)*(1 - double(objposit(1))/double(objposit(3))/double(tan(0.5*perangle)));
               py=512 - 0.5*double(512)*(1 + double(objposit(2))/double(objposit(3))/double(tan(0.5*perangle)));
               disp( [px, py] )
               disp( detectedLocation )
               %disp( dist );
               if dist > 0.9
                   vrep.simxSetJointTargetVelocity(clientID,rmotor, 2+2*detectedPoint(1) ,vrep.simx_opmode_streaming);                   
                   vrep.simxSetJointTargetVelocity(clientID,lmotor, 2+2*detectedPoint(2) ,vrep.simx_opmode_streaming);
               elseif dist < 0.7
                   vrep.simxSetJointTargetVelocity(clientID,rmotor, -2-2*detectedPoint(1) ,vrep.simx_opmode_streaming);                   
                   vrep.simxSetJointTargetVelocity(clientID,lmotor, -2-2*detectedPoint(2) ,vrep.simx_opmode_streaming);
               elseif dist > 0
                   vrep.simxSetJointTargetVelocity(clientID,rmotor, 0 ,vrep.simx_opmode_streaming);                   
                   vrep.simxSetJointTargetVelocity(clientID,lmotor, 0 ,vrep.simx_opmode_streaming);
               end
               
               if detectedLocation(1) < 92
                   cameraAngle = cameraAngle - cameraAngleSpeed;
                   vrep.simxSetJointTargetPosition(clientID,motor,cameraAngle,vrep.simx_opmode_streaming);
               end
               if detectedLocation(1) > 420
                   cameraAngle = cameraAngle + cameraAngleSpeed;
                   vrep.simxSetJointTargetPosition(clientID,motor,cameraAngle,vrep.simx_opmode_streaming);
               end
           end
                      
      end
      
      showTrajectory();
      vrep.simxFinish(-1);
      
 end
 vrep.delete();
 
  %%
 function utilities = createUtilities(param)
  % Create System objects for reading video, displaying video, extracting
  % foreground, and analyzing connected components.
  utilities.videoPlayer = vision.VideoPlayer('Position', [100,100,500,400]);
  utilities.foregroundDetector = vision.ForegroundDetector('NumGaussians', 3, 'AdaptLearningRate', true, ...
    'NumTrainingFrames', 2, 'InitialVariance', param.segmentationThreshold);
  utilities.blobAnalyzer = vision.BlobAnalysis('AreaOutputPort', true, ...
    'MinimumBlobArea', 70, 'CentroidOutputPort', true, 'BoundingBoxOutputPort',true);

  utilities.accumulatedImage      = 0;
  utilities.accumulatedDetections = zeros(0, 2);
  utilities.accumulatedTrackings  = zeros(0, 2);
  utilities.foregroundMask = 0;
 end

function param = getDefaultParameters
  param.motionModel           = 'ConstantAcceleration';
  param.initialLocation       = 'Same as first detection';
  param.initialEstimateError  = 1E5 * ones(1, 3);
  param.motionNoise           = [25, 10, 1];
  param.measurementNoise      = 25;
  param.segmentationThreshold = 0.05;
end

function trackSingleObject(param)
  % Create utilities used for reading video, detecting moving objects,
  % and displaying the results.

    frame =  im2double(param.image);

    % Detect the ball.
    [detectedLocation, isObjectDetected] = detectObject(frame);

    if ~isTrackInitialized
      if isObjectDetected
        % Initialize a track by creating a Kalman filter when the ball is
        % detected for the first time.
        initialLocation = computeInitialLocation(param, detectedLocation);
        kalmanFilter = configureKalmanFilter(param.motionModel, ...
          initialLocation, param.initialEstimateError, ...
          param.motionNoise, param.measurementNoise);

        isTrackInitialized = true;
        trackedLocation = correct(kalmanFilter, detectedLocation);
        label = 'Initial';
      else
        trackedLocation = [];
        label = '';
      end

    else
      % Use the Kalman filter to track the ball.
      if isObjectDetected % The ball was detected.
        % Reduce the measurement noise by calling predict followed by
        % correct.
        predict(kalmanFilter);
        trackedLocation = correct(kalmanFilter, detectedLocation);
        label = 'Corrected';
      else % The ball was missing.
        % Predict the ball's location.
        trackedLocation = predict(kalmanFilter);
        label = 'Predicted';
      end
    end
    
    annotateTrackedObject();
end

%%
% Show the current detection and tracking results.
function annotateTrackedObject()
  accumulateResults();
  % Combine the foreground mask with the current video frame in order to
  % show the detection result.
  imshow(utilities.foregroundMask);
  combinedImage = max(repmat(utilities.foregroundMask, [1,1,3]), frame);

  if ~isempty(trackedLocation)
    shape = 'circle';
    region = trackedLocation;
    region(:, 3) = 5;
    combinedImage = insertObjectAnnotation(combinedImage, shape, ...
      region, {label}, 'Color', 'red');
    shape = 'circle';
    region = [px py];
    region(:, 3) = 5;
    combinedImage = insertObjectAnnotation(combinedImage, shape, ...
      region, {'GT'}, 'Color', 'red');
  end
  step(utilities.videoPlayer, combinedImage);
end

% Detect the ball in the current video frame.
function [detection, isObjectDetected] = detectObject(frame)
  grayImage = frame;
  utilities.foregroundMask = step(utilities.foregroundDetector, grayImage);
  [Area, detections,BB] = step(utilities.blobAnalyzer, utilities.foregroundMask);
  if isempty(detections)
      detection = [];
      isObjectDetected = false;
  else
    % Use the biggest detected object.
    if ~isempty(preDetection)
        validIndex =find(sqrt(sum((detections- repmat(preDetection,size(detections,1),1)).^2,2))<adjSensibility & double(max(BB(:,3:4),[],2)./min(BB(:,3:4),[],2))<5);
        detections = detections(validIndex,:); % Look for a object around the location of previous
        Area = Area(validIndex);
    end
    if isempty(detections)
      detection = [];
      isObjectDetected = false;
    else
        [~,biggestDectection] = max(Area);
        detection = detections(biggestDectection, :);
        preDetection = detection;
        isObjectDetected = true;
    end
  end
end

%%
% Accumulate video frames, detected locations, and tracked locations to
% show the trajectory of the ball.
function accumulateResults()
  utilities.accumulatedImage  = max(utilities.accumulatedImage, frame);
  utilities.accumulatedDetections ...
    = [utilities.accumulatedDetections; detectedLocation];
  utilities.accumulatedTrackings  ...
    = [utilities.accumulatedTrackings; trackedLocation];
end
%%
% For illustration purposes, select the initial location used by the Kalman
% filter.
function loc = computeInitialLocation(param, detectedLocation)
  if strcmp(param.initialLocation, 'Same as first detection')
    loc = detectedLocation;
  else
    loc = param.initialLocation;
  end
end

end