%% 1. READ map and the ground truth of the target 
% Navigation Toolbox map format 
mapOrig = load('factoryOctomap.mat').map;
map = load('factoryOctomapInflate8.mat').map;

xBound = [-190.6074 28.7521];
yBound = [-145.1625 18.6637];
zBound = [0 10];

% For the original, do the below 
inflateRad = 0.5;
inflate(map,inflateRad)

% Target movement (t,xq(t))
bagFile = 'MultiTarget.bag';
bag = rosbag(bagFile);
target_topic = '/airsim_node/Woman_15_pose';
bSel = select(bag,'Topic',target_topic);
msgStructs = readMessages(bSel,'DataFormat','struct');
xPoints = cellfun(@(m) double(m.Pose.Position.X),msgStructs)';
yPoints = cellfun(@(m) double(m.Pose.Position.Y),msgStructs)';
zPoints = cellfun(@(m) double(m.Pose.Position.Z),msgStructs)';

qxPoints = cellfun(@(m) double(m.Pose.Orientation.X),msgStructs)';
qyPoints = cellfun(@(m) double(m.Pose.Orientation.Y),msgStructs)';
qzPoints = cellfun(@(m) double(m.Pose.Orientation.Z),msgStructs)';
qwPoints = cellfun(@(m) double(m.Pose.Orientation.Z),msgStructs)';

translations = [xPoints ; yPoints; zPoints];
orientations = [qxPoints; qyPoints; qzPoints; qwPoints];
ts = cellfun(@(m) double(double(m.Header.Stamp.Sec) +double(m.Header.Stamp.Nsec)*1e-9),msgStructs);
ts  = ts - ts(1);


%% 1.1 Map - target movement overlay 
figure(1)
cla
show(mapOrig)
view([0 90])
hold on 
plot3 (xPoints,yPoints,zPoints,'k-')

set(gca,'XLim',xBound)
set(gca,'YLim',yBound)




%% 2. Setting offline prediction library 
vxMin = 0.1; vxMax = 1.5; vN = 20; vSet = linspace(vxMin,vxMax,vN);
axMin = 0.0; axMax = 0.2; axN = 10; axSet = linspace(axMin,axMax,axN);
ayMin = -0.4; ayMax = 0.4; ayN = 7; aySet = linspace(ayMin,ayMax,ayN);
T = 2.0; tN = 10; tSet= linspace(0.0,T,tN);
trajIdx = 1;
for v = vSet
    for ax = axSet
        for ay = aySet
            % drive 
            traj = zeros(3, length(tSet));
            tIdx = 1; 
            for t = tSet
                X = 0.5 * [ax ay 0.0]' * t^2 + [v 0 0 ]' * t;
                traj (:,tIdx) = X;
                tIdx = tIdx + 1; 
            end
            trajSet{trajIdx} = traj; 
            trajIdx = trajIdx + 1;
        end
    end
end

%% 2.1 Draw offline library 
figure(2);
cla 
hold on 
for trajIdx = 1:length(trajSet)
    traj = trajSet{trajIdx};
    plot3(traj(1,:),traj(2,:),traj(3,:),'k-')
end

%% 3. Evalute the prediction accuracy 
nO = 10; nEval = 10; nStride = 5; 
for tStep = nO: nStride :length(ts)-nEval
    
   
    
end

