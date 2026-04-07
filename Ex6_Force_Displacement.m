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
% addpath(genpath('D:\Project\BCM_Method_Ultilize_DAS_2D\DAS_2D_3D\version0_90_legacy\2D\'));
addpath(genpath('D:\Project\BCM_Method_Ultilize_DAS_2D\KHOI_2D\'));

%% workspace
%first you need to create a workspace
% Input 1 - Size of the workspace in x (Do not effect analysis)
% Input 2 - Size of the workspace in y (Do not effect analysis)
% Input 3 - Length Unit
% Input 4 - Force Unit 
currentWorkspace=WorkSpace(10,10,LengthUnit.Millimeter,ForceUnit.Newton);

%% honeycomb geometry (adapted from bcm_energy_honeycomb_tensile_3row_2column)
% parameters
L = 5; THETA = 30*pi/180; H = 5;   % element geometry
NROW = 3; NCOL = 2;                % cell counts (kept small for example)

% generate node coordinates and segment (links)
Xm = zeros(1, 2*NROW + 2);
Ym = zeros(1, 2*NROW + 2);
segment = [];
Xm(1) = 0; Ym(1) = 0;
for ii=1:2:NROW*2
    Xm(ii+1) = Xm(ii)-L*sin(THETA);
    Ym(ii+1) = Ym(ii)+L*cos(THETA);
    segment = [segment; ii, ii+1];
    Xm(ii+2) = Xm(ii+1)+L*sin(THETA);
    Ym(ii+2) = Ym(ii+1)+L*cos(THETA);
    segment = [segment; ii+1, ii+2];
end
hcncount = ii+2;
for k=1:hcncount
    if mod(k,2) == 1
        Xm(k+hcncount) = Xm(k) + H;
        Ym(k+hcncount) = Ym(k);
    else
        Xm(k+hcncount) = Xm(k) + H + 2*L*sin(THETA);
        Ym(k+hcncount) = Ym(k);
    end
    if mod(k,2) == 1 && k~=1 && k~=hcncount
        segment = [segment; k, k+hcncount];
    end
    if k > 1
        segment = [segment; hcncount+k-1, hcncount+k];
    end
end
cncount = 2*hcncount;
for jj=2:NCOL
    for k=1:cncount
        Xm(k+cncount*(jj-1)) = Xm(k) + (H+H+2*L*sin(THETA))*(jj-1);
        Ym(k+cncount*(jj-1)) = Ym(k);
        if k ~= 1 && k ~= 1+hcncount
            segment = [segment; k+cncount*(jj-1)-1, k+cncount*(jj-1)];
        end
        if mod(k,2)==0 && k~=1 && k~=hcncount && k < hcncount
            segment = [segment; k+cncount*(jj-1)-hcncount, k+cncount*(jj-1)];
        end
        if mod(k,2)==0 && k~=hcncount+1 && k~=cncount && k > hcncount
            segment = [segment; k+cncount*(jj-1)-hcncount, k+cncount*(jj-1)];
        end
    end
end

figure(1)
hnd1 = plot(Xm,Ym,'ob');
hold on;
for i=1:length(segment)
    plot([Xm(segment(i,1)) Xm(segment(i,2))],[Ym(segment(i,1)) Ym(segment(i,2))],'-r');
	text((Xm(segment(i,1))+Xm(segment(i,2)))/2,(Ym(segment(i,1))+Ym(segment(i,2)))/2,sprintf('  %d', i), 'VerticalAlignment', 'bottom', 'FontSize', 10,'Color', 'r');
end	
xlabel('X [mm]');
ylabel('Y [mm]');
axis equal
% Add point numbers
fcoords = fopen('coords.txt','w');
for i = 1:length(Xm)
    text(Xm(i), Ym(i), sprintf('  %d', i), 'VerticalAlignment', 'bottom', 'FontSize', 10);
	fprintf(fcoords,'%f %f\r\n',Xm(i),Ym(i));
end
fclose(fcoords);
flinks = fopen('links.txt','w');
for i = 1:length(segment)
    
	fprintf(flinks,'%d %d\r\n',segment(i,1),segment(i,2));
end
fclose(flinks);

%% Cross Section
YOUNG = 2.15e3; % MUST be in MPa (N/mm^2) to match FEA units and correct 1000x scaling error
IPDIM = 1; % 1mm 
OPDIM = 3; % 3mm 
% Create CrossSection 
crossSection = CrossSection(IPDIM, OPDIM, YOUNG, BeamType.CBCM);
crossSection.segments = 1; % 4 segments are necessary for accuracy but clutter drawNoGUI
crossSection.thickness = IPDIM; 
crossSection.width = OPDIM;     
crossSection.E = YOUNG;         
crossSection.type = BeamType.CBCM; 

%% Connections and Links
nNodes = length(Xm); 
nElements = size(segment, 1);

allNodes = Node.empty;
for i = 1:nNodes, allNodes(i) = Node(i, Xm(i), Ym(i)); end
    
% MOVE NODES 
movedNodes = [7, 14, 21, 28];
% MOVE LINKS 
movedLinks = [6, 14, 23, 31];
% FIX NODE 
fixedNodes = [1, 8, 15, 22];
% CREATE LINKS TO CONNECT NODES
allLinks = Link.empty;

% % Tie the top horizontal links to form a rigid shuttle
% % Identify segments between movedNodes [7, 14, 21, 28]
% for i = 1:nElements
%     n1 = segment(i, 1);
%     n2 = segment(i, 2);
%     if ismember(n1, movedNodes) && ismember(n2, movedNodes)
%         allLinks(i).group = 1; % Assign to group 1 (Rigid Plate)
%     end
% end

% for i = 1:nElements
%     allLinks(i) = Link(i, segment(i, :));
    
%     % --- Determine Joint Types ---
%     n1 = segment(i, 1);
%     n2 = segment(i, 2);
%     joints = [Joint.Welded, Joint.Welded];
    
%     if ismember(n1, fixedNodes), joints(1) = Joint.GroundWelded; end
%     if ismember(n2, fixedNodes), joints(2) = Joint.GroundWelded; end
    
%     allLinks(i) = allLinks(i).setJoints(joints);

%     % Standard lattice links are compliant
%     allLinks(i) = allLinks(i).setCrossSection(crossSection);
%     allLinks(i) = allLinks(i).setLinearSpring(0);
% end

% Build the mechanism with compliant links and rigid coupling
for i = 1:nElements
    allLinks(i) = Link(i, segment(i, :));
    allLinks(i) = allLinks(i).setJoints([Joint.Welded, Joint.Welded]);
    % Apply CBCM to ALL links
    allLinks(i) = allLinks(i).setCrossSection(crossSection);
    allLinks(i) = allLinks(i).setLinearSpring(0);
end

% Set joints for fixed nodes
for node_id = fixedNodes
    for i = 1:nElements
        if segment(i, 1) == node_id
            joints = allLinks(i).getJoints(); 
            joints(1) = Joint.GroundWelded;
            allLinks(i) = allLinks(i).setJoints(joints);
        elseif segment(i, 2) == node_id
            joints = allLinks(i).getJoints(); 
            joints(2) = Joint.GroundWelded;
            allLinks(i) = allLinks(i).setJoints(joints);
        end
    end
end

% Add rigid coupling links between moved nodes [7, 14, 21, 28] to form a "shuttle"
for i = 1:length(movedNodes)-1
    n1 = movedNodes(i);
    n2 = movedNodes(i+1);
    linkID = length(allLinks) + 1;
    allLinks(linkID) = Link(linkID, [n1, n2]);

    % To match FEA (nset_top, 2, 2, 0), we use a GroundSlider on the shuttle.
    % Because the group is rigid, constraining one node constrains all of them.
    % if i == 1
    %     allLinks(linkID) = allLinks(linkID).setJoints([Joint.GroundSlider, Joint.Welded]);
    %     allLinks(linkID).angleSlider = 0; % Force U2 = 0 (Horizontal motion only)
    % else
    %     allLinks(linkID) = allLinks(linkID).setJoints([Joint.Welded, Joint.Welded]);
    % end
    allLinks(linkID) = allLinks(linkID).setJoints([Joint.Welded, Joint.Welded]);

    % Comment this line to disable rigid coupling ~ make the rigid link non-compliant
    % allLinks(linkID) = allLinks(linkID).setCrossSection(crossSection);
    % Rigid links: do NOT assign a compliant cross-section.
    % Assigning to a 'group' tells the solver these move as one rigid body.
    allLinks(linkID).group = 1; 
    allLinks(linkID) = allLinks(linkID).setLinearSpring(0);
end

%% Forces setup 
% Apply a single force to the rigid shuttle. Solving this magnitude will give
% the total assembly reaction force (RF) comparable to your Abaqus results.
shuttleLinkID = length(allLinks); 
allForces = Force(1, shuttleLinkID, 1.0, 0, 1.0, 0, 100, 0);
allForces.active = 1;

%% Statics DistanceAnalysis
% Create distance analysis object
%Input 1 - links
%Input 2 - nodes
%Input 3 - forces
%Input 4 - moments
%Input 5 - torsion springs
%Input 6 - workspace
analysis = DistanceAnalysis(allLinks, allNodes, allForces, [], [], currentWorkspace);

% Select force 1 as unknown magnitude
analysis = analysis.selectLoad(1, 'force');

% Number of simulation steps
numSteps = 10;
x_displ = 5; % Total x-displacement to apply [mm]

% Initialize arrays to store results
displacements = zeros(numSteps + 1, 1);
forces = zeros(numSteps + 1, 1);
y_drifts = zeros(numSteps + 1, 1); % Array to track vertical deformation

% Run analysis for multiple steps
for i = 1:numSteps
    % Incremental x-displ [mm]
    displ = x_displ * i / numSteps;

    % Add target displacement to the last node (the whole group will follow)
    % Input 1 - nodeID
    % Input 2 - magnitude
    % Input 3 - type
    % Input 4 - links array
    analysis = analysis.addInput(movedNodes(end), displ, 'x', allLinks);
        
    % Run the distance analysis (optimizes force magnitude to meet target displ)
    analysis = analysis.simulationNoGUI(currentWorkspace);

    % Display and store results
    currentForce = analysis.newValues.newValue;
    forces(i + 1) = currentForce;
    displacements(i + 1) = displ;
    
    % Record Y-displacement of the control node (Node 28)
    initialY = allNodes(movedNodes(end)).getNode().y;
    currentY = analysis.static.nodes(movedNodes(end)).getNode().y;
    y_drifts(i + 1) = currentY - initialY;

    % plot the problem
    figure(2); cla; hold on; axis([-10 30 -5 30]);
    analysis.drawNoGUI(cla, 20, currentWorkspace);
    title(['Step ' num2str(i) ': Solved Total Force = ' num2str(currentForce) ' N']);
    drawnow;
    
    % % Clean Visualization: Manually plot physical links using updated node coordinates
    % % This avoids rendering the messy intermediate sub-segment joints
    % figure(2); cla; hold on; grid on; axis([-10 30 -5 30]);
    % updatedNodes = analysis.static.nodes;
    % for k = 1:length(allLinks)
    %     nID = allLinks(k).nodes;
    %     p1 = updatedNodes(nID(1)).getNode();
    %     p2 = updatedNodes(nID(2)).getNode();
        
    %     if allLinks(k).group == 1
    %         plot([p1.x p2.x], [p1.y p2.y], 'g-', 'LineWidth', 3); % Rigid Shuttle
    %     else
    %         plot([p1.x p2.x], [p1.y p2.y], 'r-', 'LineWidth', 1.5); % Honeycomb
    %     end
    % end
    % title(['Step ' num2str(i) ': Solved Total Force = ' num2str(currentForce) ' N']);
    % drawnow;

    disp(['Step ' num2str(i) ': Target Displ = ' num2str(displ) ', Required Total Force = ' num2str(currentForce)]);
end

load -ascii D:\Project\BCM_Method_Ultilize_DAS_2D\Workspace\Abaqus\latticeGXY.txt

% Plot Force-Displacement Curve
figure(3);
plot(displacements, forces, 'bo-', ...
    latticeGXY(:,1), abs(latticeGXY(:,2)), 'r-', ...
    'LineWidth', 2);
xlabel('Displacement (mm)');
ylabel('Force (N)');
title('Force-Displacement Curve');
grid on;

% Plot Vertical Drift (Y-Deformation) to prove the trend
figure(4);
plot(displacements, y_drifts, 'rs-', 'LineWidth', 2);
xlabel('Applied X-Displacement (mm)');
ylabel('Resulting Y-Drift (mm)');
title('Proof of Vertical Deformation Trend (Poisson Effect)');
grid on;
fprintf('Maximum vertical drift recorded: %f mm\n', max(abs(y_drifts)));