%Ex-6
%Distance Analysis Example
%Compliant Four Bar Mechanism
%This script will:
%1 Create the mechanism (convert a link to compliant and add a moment). 
%2 The compliant link is represented with a PRB model.
%3 Add a target rotation and perform distance analysis.
%4 Update the moment magnitude with the value from last step.
%5 Analyze the mechanism in 10 steps.

clear all;
close all;
addpath(genpath('D:\Project\BCM_Method_Ultilize_DAS_2D\Kinematic_Solver\'));

%% Workspace Initialization
%first you need to create a workspace
% Input 1 - Size of the workspace in x (Do not effect analysis)
% Input 2 - Size of the workspace in y (Do not effect analysis)
% Input 3 - Length Unit
% Input 4 - Force Unit 
currentWorkspace=WorkSpace(10,10,LengthUnit.Millimeter,ForceUnit.Newton);

%% Nodes Initialization
%node coordinates array
nodeCoordinates=[-4 -2; -3 2; 5 5; 5 -5];
%setup the nodes
for i=1:size(nodeCoordinates,1)
    %create nodes from x y array
    %Input 1 - id
    %Input 2 - x
    %Input 3 - y
    nodes(i) = Node(i,nodeCoordinates(i,1),nodeCoordinates(i,2));
end

%% Links Initialization
%setup the links
%connectivity array
connections=[1 2; 2 3; 4 3;];
for i=1:size(connections,1)
    %create links from connectivity array
    %Input 1 - id
    %Input 2 - [node1,node2]
    links(i) = Link(i,connections(i,:));
end
%set up the joints
%joints for the previously defined links
jointList = [Joint.GroundPin Joint.Pin; ...
             Joint.Pin Joint.Pin; ...
             Joint.GroundWelded Joint.Pin;];

for i=1:size(jointList, 1)
    %create links from connectivity array
    %Input 1 - id
    %Input 2 - [node1,node2]
    links(i) = links(i).setJoints(jointList(i,:));
end
%convert one link to compliant
%make a link compliant
%Input 1 - in-plane width
%Input 2 - out-plane width
%Input 3 - E
%Input 4 - type (BeamType.PRB,BeamType.CBCM,BeamType.Mlinear)
for i=1:3
    links(i) = links(i).makeCompliant(1, 0.5, 69, BeamType.CBCM);
    links(i).geometry.segments = 1; % Use a single segment CBCM continuous element
end


%% Moments Initialization
%add a moment (in distance analysis, load to be computed will be made unknown later in the script)
%Moment(id,link,magnitude,distance)
%Input 1 - id
%Input 2 - link
%Input 3 - Magnitude
%Input 4 - Distance from node 1 of the link
moments(1) = Moment(1, 1, 1, 50);

%% Forces Initialization
%add a force (in distance analysis, load to be computed will be made unknown later in the script)
%Force(id,link,magnitude,distance)
%Input 1 - id
%Input 2 - link
%Input 3 - xValue
%Input 4 - yValue
%Input 5 - Magnitude
%Input 6 - Angle (radian)
%Input 7 - Distance
%Input 8 - Follower
forces(1) = Force(1, 2, 1.0, 0, 1.0, 0, 100, 0);

%% Static Analysis
%create distance analysis object
%Input 1 - links
%Input 2 - nodes
%Input 3 - forces
%Input 4 - moments
%Input 5 - torsion springs
%Input 6 - workspace
analysis=DistanceAnalysis(links,nodes,forces,[],[],currentWorkspace);

% Number of simulation steps
numSteps = 10;

% Initialize arrays to store results
displacements = zeros(numSteps,1);
computed_forces = zeros(numSteps,1);

%select the unknown load (in distance analysis, load to be computed should be made unknown)
%Input 1 - id
%Input 2 - type - 'force', 'moment'
analysis=analysis.selectLoad(1,'force');

disp('Starting Legacy Distance Analysis...');

% Start timing the solver
t_start = tic;

% Run analysis for multiple steps
for i = 1:numSteps
    %add target displacement
    %Input 1 - link (for moment) or node (for force)
    %Input 2 - magnitude
    %Input 3 - type - 'angle', 'x', 'y'
    %Input 4 - links
    targetDisp = 2 * i / numSteps;
    analysis=analysis.addInput(3, targetDisp, 'x', links);

    %plot the problem
    figure(1)
    %f=figure('Name','Distance Analysis Problem');
    hold on
    axis([-10 10 -10 10]);
    analysis.drawNoGUI(cla,10,currentWorkspace);
    drawnow;

    %run the distance analysis
    %Input 1 - workspace
    analysis=analysis.simulationNoGUI(currentWorkspace);
    % Store results
    displacements(i) = targetDisp; % Get applied displacement
    computed_forces(i) = analysis.newValues.newValue; % Get computed force/moment
    fprintf('Step %d: Required Force = %.3f N, Target Displ = %.3f mm\n', i, computed_forces(i), displacements(i));
end

% End timing the solver
elapsedTime = toc(t_start);
fprintf('Legacy Distance Analysis completed in %.3f seconds.\n', elapsedTime);

% Plot Force-Displacement Curve
figure(99);
plot(displacements, computed_forces, 'bo-', 'LineWidth', 2);
xlabel('Displacement (mm)');
ylabel('Force (N)');
title('Force-Displacement Curve (Legacy)');
grid on;
