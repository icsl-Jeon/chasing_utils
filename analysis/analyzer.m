%% Scenario generation (do not run when data exists)

% load obstacle
obstacleSet = load('map24\map.mat').obstacles;

% target random generator 
QvwSet = [linspace(0.1,1.0,10); ...
                  linspace(10.0,15.0,10)]; % variance of motion 
              
q0 = [0 5 0]'; % x,y, th
dt = 0.1; % integration
v0 = 1; % reference velocity 
w0 = pi/4'; % reference angle 
obstColor = 0.7*[1 1 1];
T = 50; % total horizon 
N = T/dt+1; % step number 
ts = linspace(0,T,N); 

for i = 1:length(QvwSet) % Only the final case 
    
    qSetX = {}; % points on the sampled traj. 
    qSetY = {};
    
    Qv = QvwSet(1,i); % variance of velocity 
    Qw = QvwSet(2,i); % variance of theta

    inputTraj = [normrnd(v0,Qv,1,N-1); normrnd(w0,Qw,1,N-1)]; 
    
    CovV = diag(Qv*ones(1,N-1)); % excluded the dist. of the first step 
    CovW = diag(Qw*ones(1,N-1));
    smoothing = 50; % For the smooth movement , we do smoothing on the randomized input trajectories
    Nsample = 100; % # of trajs to be generated 
    
    % Draw ellipse (dynamic obstacle deprecated) 
    figure(i)
    clf
    titleStr = sprintf('Qv = %f / Qw = %f ',Qv,Qw);
    title(titleStr)
    hold on
    t_future = linspace(0,T,10);
    for t = t_future        
        for n = 1:length(obstacleSet)
            rx = obstacleSet(n).c(1);
            ry = obstacleSet(n).c(2);
            R = eye(2);
            u = obstacleSet(n).u;        
            obstCenter = [polyval(rx,t),polyval(ry,t)];
            draw_ellipse(R,u,obstCenter,obstColor,'k');        
        end      
    end

    % Sample a feasible target trajectories over [0,T]
    nFeasible = 0;
    while nFeasible < Nsample
        isFeasible = true;
        inputTraj = [smooth(mvnrnd(v0*ones(1,N-1),CovV),smoothing)' ; smooth(mvnrnd(w0*ones(1,N-1),CovW),smoothing)'];
        q = q0;
        qTraj = q;
        for n = 1:N-1
            xdot = inputTraj(1,n)*cos(inputTraj(2,n));
            ydot = inputTraj(1,n)*sin(inputTraj(2,n));
            thdot = inputTraj(2,n);
            q = q + [xdot ydot thdot]'*dt;
            qTraj = [qTraj q];
            
            xy = q(1:2);
            % boundary check 
            if xy(1) < 0 || xy(2) < 0 || xy(1) > 45 || xy(2) > 45
               isFeasible = false;
               break; 
            end
                        
            % Collision check 
            for nr = 1:length(obstacleSet)
                obstacle = obstacleSet(nr);
                r = obstacle.c';
                % Compute ellipsoid (x-c)'A(x-c) = 1
                S = diag((obstacle.u).^(-1))*(eye(2))';
                A = S'*S;
                if (r - xy)'*A*(r-xy) <= 1.5
                    isFeasible = false;
                    break
                end            
            end

            if ~isFeasible 
                break; % Subset of the trajs can be have immature length
            end
        end

        if isFeasible
            figure(i)
            hold on
            grayScale = 0.8;
            h = plot(qTraj(1,:),qTraj(2,:),'k-','MarkerSize',1.5);
            
            % Collect 
            nFeasible = nFeasible + 1; 
            qSetX{nFeasible} = qTraj(1,:);
            qSetY{nFeasible} = qTraj(2,:);
            axis equal
        end
    end
    qTrajSet{i}.ts = ts;
    qTrajSet{i}.Xs = qSetX;
    qTrajSet{i}.Ys = qSetY;
end

save('targetTrajSet','qTrajSet')

%% 1. Load scenario and check scenraio  
qTrajSet = load('targetTrajSet.mat').qTrajSet;
obstacleSet = load('map24\map.mat').obstacles;
obstColor = 0.7*[1 1 1];

nScenario = length(qTrajSet);
figure
for n = 1:nScenario
    subplot(1,10,n)    
    hold on    
    for m = 1:length(obstacleSet)
        R = eye(2);
        u = obstacleSet(m).u;        
        draw_ellipse(R,u,obstacleSet(m).c,obstColor,'k');        
    end       

    trajStruct = qTrajSet(n);
    trajStruct = trajStruct{1};
    for m = 1:100
       Xs = trajStruct.Xs{m}; 
       Ys = trajStruct.Ys{m}; 
       plot(Xs,Ys,'k-')
    end
    axis equal
    axis([0 45 0 45])    
end


%% 2. Setting offline prediction library 
vxMin = 0.1; vxMax = 1.5; vN = 20; vSet = linspace(vxMin,vxMax,vN);
axMin = 0.0; axMax = 0.2; axN = 10; axSet = linspace(axMin,axMax,axN);
ayMin = -0.4; ayMax = 0.4; ayN = 7; aySet = linspace(ayMin,ayMax,ayN);
T = 2.0; tN = 10; tSet= linspace(0.0,T,tN);
trajIdx = 1;
coeffSet = {};
trajSet = {};
for v = vSet
    for ax = axSet
        for ay = aySet
            % drive 
            traj = zeros(2, length(tSet));
            tIdx = 1; 
            for t = tSet
                X = 0.5 * [ax ay]' * t^2 + [v 0]' * t;
                traj (:,tIdx) = X;
                tIdx = tIdx + 1; 
            end
            coeffSet{trajIdx} = [ax ay v];
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
    plot(traj(1,:),traj(2,:),'k-')
end
axis equal

%% 3. Evalute the prediction accuracy 

% pick scenario and trajectory 
scenarioIdx = 10; trajIdx = 1; 
qTraj = qTrajSet{scenarioIdx};
targetTraj = [qTraj.Xs{trajIdx} ; qTraj.Ys{trajIdx}]; % [ts ; xs ; ys]
ts = qTraj.ts;

% option 
draw = true; 
nStride = 5; % simulation time stride 

% prediction paramter 
no = 10; % number of observation 
nEval = 10; % number future step for error evaluation 
predictionCheckStride = 1; 

% online prediction 
for tStep = no + 1: nStride :length(ts)-nEval
    % past observation 
    positionObsrv = targetTraj(:,tStep - no : tStep-1);     
    tCur = ts(tStep);
    tObsrv = ts(tStep - no : tStep-1);
    
    positionFuture = targetTraj(:,tStep: tStep + nEval); 
    tFuture = ts(tStep: tStep + nEval);
    
    % current pose of the target 
    velocityDir = positionObsrv(:,end) - positionObsrv(:,end-1);
    velocityDir = velocityDir / norm(velocityDir);  
    velocityDirOrth = [-velocityDir(2); velocityDir(1)];
    Rw0 = [velocityDir velocityDirOrth]; Tw0 = [[Rw0 positionObsrv(:,end)] ; [0 0 1]];
    
    % predict 
    trajSetTrans = {}; % includes only feasible 
    obsrvErrorSet = []; % observation error of each traj in library  
    trajInsertIdx = 1;     
    for trajIdx = 1:length(trajSet)
        traj = trajSet{trajIdx};
        trajTrans = Tw0 * [traj ; ones(1, size(traj,2))];   
        trajTrans = trajTrans(1:2,:); 
        
        % feasibility check 
        isFeasible = true;
        for s = 1:predictionCheckStride:length(trajTrans)
            xy = trajTrans(:,s);
            for nr = 1:length(obstacleSet)
                obstacle = obstacleSet(nr);
                r = obstacle.c';
                % Compute ellipsoid (x-c)'A(x-c) = 1
                S = diag((obstacle.u).^(-1))*(eye(2))';
                A = S'*S;
                if (r - xy)'*A*(r-xy) <= 1.5
                    isFeasible = false;
                    break
                end            
            end
        end
        
        if isFeasible            
            trajSetTrans{trajInsertIdx} = trajTrans;
            
            % compute observation error 
            obsrvErrorSum = 0; 
            coeff = coeffSet{trajIdx}; 
            ax = coeff(1); ay = coeff(2); v = coeff(3);
            for nn = 1:no
               tEval = tObsrv(nn) - tCur; 
               qEval = 0.5 * [ax ay]' * tEval^2 + [v 0]' * tEval; % polynomial evaluation w.r.t ref frame
               qEval = Tw0 * [qEval ; 1]; qEval = qEval(1:2);
               
               qObsrv = positionObsrv(:,nn); % observation  
               obsrvErrorSum = obsrvErrorSum  + norm(qEval - qObsrv);              
            end
            obsrvErrorSet(trajInsertIdx) = obsrvErrorSum;                      
            trajInsertIdx = trajInsertIdx + 1;                        
        end               
    end     
    [~,bestTrajIdx] = min (obsrvErrorSet);
    
    % prediction error   
    predictionErrorSum = 0; 
    coeff = coeffSet{bestTrajIdx}; 
    ax = coeff(1); ay = coeff(2); v = coeff(3);    
    figure(3)
    cla
    hold on 
    for nn = 1:nEval
       tEval = tFuture(nn) - tCur; 
       qEval = 0.5 * [ax ay]' * tEval^2 + [v 0]' * tEval; % polynomial evaluation w.r.t ref frame
       qEval = Tw0 * [qEval ; 1]; qEval = qEval(1:2);
       qTrue = positionFuture(:,nn); % true future  
       predictionErrorSum = obsrvErrorSum  + norm(qEval - qTrue); 
       plot (qEval(1),qEval(2),'ro')
       plot (qTrue(1),qTrue(2),'ko')       
    end        
    fprintf('prediction error sum = %f\n',predictionErrorSum)
    
    % draw 
    if draw
        figure(1)
        cla
        hold on
        % obstacle
        for m = 1:length(obstacleSet)
            R = eye(2);
            u = obstacleSet(m).u;        
            draw_ellipse(R,u,obstacleSet(m).c,obstColor,'k');        
        end             
        
        % target trajectory 
        plot(targetTraj(1,:),targetTraj(2,:),'k-')
        
        % offline library 
        for trajIdx = 1:4:length(trajSetTrans)
            trajTrans = trajSetTrans{trajIdx};
            hLibrary = plot(trajTrans(1,:),trajTrans(2,:),'c-');
        end     
        
        % final prediction 
        predictionTraj = trajSetTrans{bestTrajIdx};
        hPrediction = plot(predictionTraj(1,:),predictionTraj(2,:),'r-','LineWidth',2);
        
        axis equal 
        axis ([0 45 0 45])        
    end

    
    
    
    pause
end

