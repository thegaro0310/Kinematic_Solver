clear all;
close all;
% addpath(genpath('D:\Project\BCM_Method_Ultilize_DAS_2D\DAS_2D_3D\version0_90\2DNEW\'));
addpath(genpath('D:\Project\BCM_Method_Ultilize_DAS_2D\KHOI_2D\'));

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
    links(i) = links(i).setJoints(jointList(i,:));
end

%convert all links to compliant
prb3r = [0.1000 Inf Inf; 0.3500 3.5100 Inf; 0.4000 2.9900 Inf; 0.1500 2.5800 Inf;];
for i=1:3
    links(i) = links(i).makeCompliant(1, 0.5, 69, BeamType.PRB);
    links(i).geometry.prbModel = prb3r;
end

%% Moments Initialization
moments(1) = Moment(1, 1, 1, 50);

%% Forces Initialization
%Force(id,link,magnitude,distance)
forces(1) = Force(1, 2, 1.0, 0, 1.0, 0, 100, 0);

%% Static Analysis
% Initialize core Statics and LoadAnalysis objects directly
stat = Statics(links, nodes, forces, [], [], currentWorkspace);
loadSim = LoadAnalysis([], [], [], [], [], currentWorkspace, stat);

% Create the new custom solver instance
solver = NewtonRaphsonSolver(loadSim, 0);

% Number of simulation steps
numSteps = 10;
% Initialize arrays to store results
displacements = zeros(numSteps,1);
computed_forces = zeros(numSteps,1);

disp('Starting Custom Newton-Raphson Solver...');

% Run analysis for multiple steps
for i = 1:numSteps
    targetDisp = 5 * i / numSteps; % Target: move exactly 5mm overall
    
    % Step the solver to find the force required for the target displacement
    [dispX, reqForce] = solver.step(targetDisp);
    
    displacements(i) = dispX;
    computed_forces(i) = reqForce;
    fprintf('Step %d: Required Force = %.3f N, Displaced = %.3f mm\n', i, reqForce, dispX);
end

% Plot Force-Displacement Curve
figure(99);
plot(displacements, computed_forces, 'bo-', 'LineWidth', 2);
xlabel('Displacement (mm)');
ylabel('Force (N)');
title('Force-Displacement Curve (Newton-Raphson)');
grid on;