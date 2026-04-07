function [couplerCurve,isClosed,descriptors]=getCouplerCurve(kinematics,driverIndex,nodeIndex,varargin)
[nodes,beams]=getInputs(kinematics);
initialBeams=beams;
if isempty(varargin)%no input increment is 2 degrees
    increment=2*pi/180;
    noOfDesc=3;
else
    increment=varargin{1}*pi/180;
    noOfDesc=varargin{2};
end
anglesForward=beams(driverIndex).theta0+linspace(0,2*pi,round(2*pi/increment)+1);
anglesBackward=beams(driverIndex).theta0-linspace(0,2*pi,round(2*pi/increment)+1);
%profile on
%first go forward in direction
options=optimoptions('fsolve','Display','none','SpecifyObjectiveGradient',true,'OptimalityTolerance',1e-10,'TolX',1e-10,'MaxIter',100,'MaxFunEvals',100,'FunctionTolerance',1e-10,...
    'CheckGradients',false,'FiniteDifferenceType','forward');
couplerCurveForward=zeros(length(anglesForward),2);
couplerCurveBackward=zeros(length(anglesBackward),2);
isClosed=1;
for i=1:length(anglesForward)
    inputs=struct('index',driverIndex,'dof',1,'value',anglesForward(i));
    [beams,nodes,exitflag] = findNewConfig(beams,nodes,kinematics.equations,kinematics.uniqueConnections,kinematics.additionalDof,inputs,options);
    if exitflag> 0
        couplerCurveForward(i,:)=[nodes(nodeIndex).x,nodes(nodeIndex).y];
    else % failed, go in other direction
        isClosed=0;
        couplerCurveForward(i:end,:)=[]; %delete unused forward
        %try to go backward
        beams=initialBeams;
        for j=1:length(anglesBackward)
            inputs=struct('index',driverIndex,'dof',1,'value',anglesBackward(j));
            [beams,nodes,exitflag] = findNewConfig(beams,nodes,kinematics.equations,kinematics.uniqueConnections,kinematics.additionalDof,inputs,options);
            if exitflag> 0
               couplerCurveBackward(j,:)=[nodes(nodeIndex).x,nodes(nodeIndex).y];
            else
                couplerCurveBackward(j:end,:)=[];
                %disp('should come here');
                break;
            end
        end
        break;
    end
end
%profile viewer
if isClosed == 1 || size(couplerCurveBackward,1) < 2
    couplerCurve=couplerCurveForward;
elseif size(couplerCurveForward,1) < 2
    couplerCurve=flipud(couplerCurveBackward);
elseif size(couplerCurveForward,1) < 2 && size(couplerCurveBackward,1) < 2
    couplerCurve=[];
else
    couplerCurve=[flipud(couplerCurveBackward(2:end,:));couplerCurveForward];
    %plot(couplerCurveBackward(:,1),couplerCurveBackward(:,2),'red')
    %plot(couplerCurveForward(:,1),couplerCurveForward(:,2),'yellow')
end
descriptors=calculatePDescriptors(couplerCurve',noOfDesc);
end

function c=calculatePDescriptors(points,kLimit)
n=size(points,2);
c=zeros(1,kLimit*2);
for k=-kLimit:kLimit
    sum=0;
    for j=1:n-1
        point1=points(1,j)+1i*points(2,j) ;
        point2=points(1,j+1)+1i*points(2,j+1);
        slope=point2-point1;
        slope=slope/abs(slope);
        sum=sum+slope*exp(-2*pi*1i*(j-1)*k/(n-1));
    end
    c(1,k+kLimit+1)=sum/(n-1);
end
end

function [allBeams,nodes,exitflag] = findNewConfig(allBeams,nodes,equations,uniqueConnections,additionalDof,inputs,options)
%FINDNEWCONFG update the nodes according to the input
%add all initial conditions except the inputs
[~,~,initialGuess] = findInputMatrix(allBeams,zeros(allBeams(end).id+allBeams(end).degreesOfFreedom+3,1),inputs,additionalDof);
  % Option to display output
f=@(x)solveEquations(x,allBeams,equations,inputs,additionalDof);
try
    [x,~,exitflag] = fsolve(f,initialGuess,options);     % Call solver
    
catch
    exitflag=0;
    disp('Kinematics Failed');
    %                 err
    %                 [err.stack.line]
    %                 [err.stack.name]
end
if (exitflag>0)
    %obtain the new variables
    [newState,indexList,~] = findInputMatrix(allBeams,x,inputs,additionalDof);
    for i=1:length(allBeams)
        allBeams(i)=updateBeam(allBeams(i),newState,indexList);
    end
    nodes = updateNodes(allBeams,nodes,newState,uniqueConnections);
end
end

function [F,J] = solveEquations(x,allBeams,equations,inputs,additionalDof)
%SOLVEEQUATIONS solve the kinematic equations(for fsolve)
[outputMatrix,indexList,~] = findInputMatrix(allBeams,x,inputs,additionalDof);

j=1;
F=zeros(length(equations)*2,1);
J=zeros(length(equations)*2,length(x));
for i=1:length(equations)
    xValue=0;yValue=0;
    for k=1:length(equations(i).beamList)
        if logical(equations(i).beamList(k))
            xValue=xValue+getX(allBeams(k),outputMatrix,equations(i).beamList(k)*100);%obj.allBeams(k).getX(outputMatrix,obj.equations(i).beamList(k)*100);
            yValue=yValue+getY(allBeams(k),outputMatrix,equations(i).beamList(k)*100);%obj.allBeams(k).getY(outputMatrix,obj.equations(i).beamList(k)*100);
            if nargout > 1
                gradientX= getGradientX(zeros(length(equations)*2,1),allBeams(k),outputMatrix,indexList,equations(i).beamList(k)*100)';%obj.allBeams(k).getGradientX(outputMatrix,indexList,obj.equations(i).beamList(k)*100)';
                gradientY= getGradientY(zeros(length(equations)*2,1),allBeams(k),outputMatrix,indexList,equations(i).beamList(k)*100)';%obj.allBeams(k).getGradientY(outputMatrix,indexList,obj.equations(i).beamList(k)*100)';
                J(j,:)=J(j,:)+gradientX;
                J(j+1,:)=J(j+1,:)+gradientY;
            end
        end
    end
    F(j,1)=xValue;
    F(j+1,1)=yValue;
    j=j+2;
end
%if redundant remove one of the equations
if length(x)<size(F,1)
    F(1,:)=[];
end

end

%create the input file for the function
function [nodes,beams]=getInputs(kinematics)
noOfBeams=length(kinematics.allBeams);
empty=cell(1,noOfBeams);
beams=struct('id',empty,'class',empty,'frontMasterID',empty,'endMasterID',empty,'nodes',empty,'joints',empty,...
    'degreesOfFreedom',empty,'theta0',empty,'length0',empty,'initialGuess',empty,...
    'type',empty,'inPlaneThickness',empty,'outPlaneThickness',empty,...
    'E',empty,'childLengths',empty,'childIds',empty,'equations',empty,'fixed',empty,...
    'linearSpring',empty,'length',empty,'theta',empty,'prbBeams',empty,'torsionSprings',empty,'prbModel',empty);
for i=1:length(kinematics.allBeams)
    %common properties
    beams(i).id=kinematics.allBeams{i}.id;
    beams(i).frontMasterID=kinematics.allBeams{i}.frontMasterID;
    beams(i).endMasterID=kinematics.allBeams{i}.endMasterID;
    beams(i).degreesOfFreedom=kinematics.allBeams{i}.degreesOfFreedom;
    beams(i).theta0=kinematics.allBeams{i}.theta0;
    beams(i).length0=kinematics.allBeams{i}.length0;
    beams(i).joints=kinematics.allBeams{i}.joints;
    beams(i).nodes=kinematics.allBeams{i}.nodes;
    %kinematic beam properties
    beams(i).equations=kinematics.allBeams{i}.equation;
    beams(i).initialGuess=kinematics.allBeams{i}.calculateInitialGuess();
    beams(i).class='KinematicsBeam';
    beams(i).type=kinematics.allBeams{i}.type;
    beams(i).linearSpring=kinematics.allBeams{i}.linearSpring;
    beams(i).fixed=kinematics.allBeams{i}.fixed;
    beams(i).theta=kinematics.allBeams{i}.theta;
    beams(i).length=kinematics.allBeams{i}.length;
end
empty=cell(1,length(kinematics.nodes));
nodes=struct('id',empty,'x',empty,'y',empty,'updated',empty);
for i=1:length(kinematics.nodes)
    nodes(i).id=kinematics.nodes(i).id;
    nodes(i).x=kinematics.nodes(i).getNode().x;
    nodes(i).y=kinematics.nodes(i).getNode().y;
    nodes(i).updated=0;
end
end

function beam=updateBeam(beam,inputs,index)
%update the beam
if strcmp(beam.type,'slider')
    beam.length=inputs(beam.id);
elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider')
    beam.theta=inputs(beam.id);
    beam.length=inputs(beam.id+1);
elseif strcmp(beam.type,'allFixedSlider')
    beam.length=inputs(beam.id);
elseif  ~logical(beam.linearSpring)
    beam.theta=inputs(beam.id);
    beam.theta0=inputs(beam.id);
else
    beam.theta=inputs(beam.id);
    beam.length=inputs(beam.id+1);
end
end

function updatedNodes = updateNodes(allBeams,nodes,inputs,uniqueConnections )
%UPDATENODES updates the nodes with the recent coordinates


updatedNodes=nodes;
for i=1:length(nodes)
    updatedNodes(i).updated=0;
end
i=1;
loopCount=1;
while 1 && (loopCount < length(allBeams)*100)
    if(i > length(allBeams))
        i=1;
    end
    %check if all nodes are updated
    allUpdated=1;
    for j=1:length(uniqueConnections)
        if updatedNodes(uniqueConnections(j)).updated ~= 1
            allUpdated=0;
            break;
        end
    end
    if allUpdated == 1
        break;
    end
    if allBeams(i).joints(1) == Joint.GroundPin || allBeams(i).joints(1) == Joint.GroundWelded || updatedNodes(allBeams(i).nodes(1,1)).updated==1
        updatedNodes(allBeams(i).nodes(1,1)).updated=1;
        updatedNodes(allBeams(i).nodes(1,2)).updated=1;
        updatedNodes(allBeams(i).nodes(1,2)).x=updatedNodes(allBeams(i).nodes(1,1)).x+getX(allBeams(i),inputs,100);
        updatedNodes(allBeams(i).nodes(1,2)).y=updatedNodes(allBeams(i).nodes(1,1)).y+getY(allBeams(i),inputs,100);
    elseif updatedNodes(allBeams(i).nodes(1,2)).updated==1
        updatedNodes(allBeams(i).nodes(1,1)).updated=1;
        updatedNodes(allBeams(i).nodes(1,1)).x=updatedNodes(allBeams(i).nodes(1,2)).x-getX(allBeams(i),inputs,100);
        updatedNodes(allBeams(i).nodes(1,1)).y=updatedNodes(allBeams(i).nodes(1,2)).y-getY(allBeams(i),inputs,100);
    end
    i=i+1;
    loopCount=loopCount+1;
end

end

% kinematic equations
function x=getX(beam,outputList,distance)
value=getValues(beam,outputList);
x=beam.equations.xKnown+beam.equations.xPre*KinematicsBeam.preDefinedFunctions(beam.equations.xFunc,value,0);
x=distance/100*x;
end

function y=getY(beam,outputList,distance)
value=getValues(beam,outputList);
y=beam.equations.yKnown+beam.equations.yPre*KinematicsBeam.preDefinedFunctions(beam.equations.yFunc,value,0);
y=distance/100*y;
end

function value=getValues(beam,outputList)
%get values from the input matrix
if strcmp(beam.type,'slider')
    value(1)=outputList(beam.id);
    value(2)=beam.theta;
    value(3)=beam.theta0;
elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider')
    value(1)=outputList(beam.id);
    value(2)=outputList(beam.id+1);
elseif strcmp(beam.type,'allFixedSlider')
    value=outputList(beam.id);
elseif  ~logical(beam.linearSpring)
    value=outputList(beam.id);
else
    value(1)=outputList(beam.id);
    value(2)=outputList(beam.id+1);
end
end


function gradientX=getGradientX(gradientX,beam,outputList,indexList,distance)
value=getValues(beam,outputList);
xDerivative=beam.equations.xPre*KinematicsBeam.preDefinedFunctions(beam.equations.xFunc,value,1)*distance/100;
if strcmp(beam.type,'slider')
    if logical(indexList(beam.id))
        gradientX(indexList(beam.id),1)=gradientX(indexList(beam.id),1)+xDerivative(1);
    end
    if logical(indexList(beam.frontMasterID))
        gradientX(indexList(beam.frontMasterID),1)=gradientX(indexList(beam.frontMasterID),1)+xDerivative(2);
    end
elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
    if logical(indexList(beam.id))
        gradientX(indexList(beam.id),1)=gradientX(indexList(beam.id),1)+xDerivative(1);
    end
    if logical(indexList(beam.id+1))
        gradientX(indexList(beam.id+1),1)=gradientX(indexList(beam.id+1),1)+xDerivative(2);
    end
elseif strcmp(beam.type,'allFixedSlider')
    if logical(indexList(beam.id))
        gradientX(indexList(beam.id),1)=gradientX(indexList(beam.id),1)+xDerivative(1);
    end
else
    if logical(indexList(beam.id))
        gradientX(indexList(beam.id),1)=gradientX(indexList(beam.id),1)+xDerivative(1);
    end
    if logical(indexList(beam.id+1))
        gradientX(indexList(beam.id+1),1)=gradientX(indexList(beam.id+1),1)+xDerivative(2);
    end
end
end

function gradientY=getGradientY(gradientY,beam,outputList,indexList,distance)
value=getValues(beam,outputList);
yDerivative=beam.equations.yPre*KinematicsBeam.preDefinedFunctions(beam.equations.yFunc,value,1)*distance/100;
if strcmp(beam.type,'slider')
    if logical(indexList(beam.id))
        gradientY(indexList(beam.id),1)=gradientY(indexList(beam.id),1)+yDerivative(1);
    end
    if logical(indexList(beam.frontMasterID))
        gradientY(indexList(beam.frontMasterID),1)=gradientY(indexList(beam.frontMasterID),1)+yDerivative(2);
    end
elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
    if logical(indexList(beam.id))
        gradientY(indexList(beam.id),1)=gradientY(indexList(beam.id),1)+yDerivative(1);
    end
    if logical(indexList(beam.id+1))
        gradientY(indexList(beam.id+1),1)=gradientY(indexList(beam.id+1),1)+yDerivative(2);
    end
elseif strcmp(beam.type,'allFixedSlider')
    if logical(indexList(beam.id))
        gradientY(indexList(beam.id),1)=gradientY(indexList(beam.id),1)+yDerivative(1);
    end
else
    if logical(indexList(beam.id))
        gradientY(indexList(beam.id),1)=gradientY(indexList(beam.id),1)+yDerivative(1);
    end
    if logical(indexList(beam.id+1))
        gradientY(indexList(beam.id+1),1)=gradientY(indexList(beam.id+1),1)+yDerivative(2);
    end
end
end

%get the initial guess
function [outputList,indexList,initialGuess] = findInputMatrix(allBeams,inputMatrix,input,additionalDof)
%prepares the input matrix for solvers
degreesOfFreedom=allBeams(end).id+1;
initialGuess=[];
outputList=zeros(1,degreesOfFreedom);
indexList=zeros(1,degreesOfFreedom);
%assign the inputs
%input(length(inputs))=struct('index',[],'dof',[],'value',[]);
for i=1:length(input)
    if ~logical(input(i).value)
        input(i).value=1e-10;
    end
    if logical(allBeams(input(i).index).frontMasterID)
        outputList(allBeams(input(i).index).frontMasterID)=input(i).value-allBeams(input(i).index).theta0;
    elseif logical(allBeams(input(i).index).endMasterID)
        inputValue=input(i).value-allBeams(input(i).index).theta0;
        if ~logical(inputValue)
            inputValue=1e-6;
        end
        outputList(allBeams(input(i).index).endMasterID)=inputValue;
    else
        outputList(allBeams(input(i).index).id+input(i).dof-1)=input(i).value;
    end
    
end
inputPosition=1;
%add the additional inputs
for i=1:length(additionalDof)
    if ~logical(additionalDof(i).known) && ~logical(outputList(i))
        if isempty(additionalDof(i).value)
            initialGuess(end+1)=0;
        else
            initialGuess(end+1)=additionalDof(i).value;
        end
        additionalDof(i).value=inputMatrix(inputPosition);
        outputList(i)=inputMatrix(inputPosition);
        indexList(i)=inputPosition;
        inputPosition=inputPosition+1;
    end
end
for i=1:length(allBeams)
    if strcmp(allBeams(i).type,'slider')
        if ~logical(outputList(allBeams(i).id))
            %length
            outputList(allBeams(i).id)=inputMatrix(inputPosition);
            indexList(allBeams(i).id)=inputPosition;
            inputPosition=inputPosition+1;
            initialGuess(end+1)=allBeams(i).length;
        end
    elseif strcmp(allBeams(i).type,'allFixedSlider')
        if ~logical(outputList(allBeams(i).id))
            %length
            outputList(allBeams(i).id)=inputMatrix(inputPosition);
            indexList(allBeams(i).id)=inputPosition;
            inputPosition=inputPosition+1;
            initialGuess(end+1)=allBeams(i).initialGuess;
        end
    elseif allBeams(i).fixed == 1 || (allBeams(i).degreesOfFreedom == 1 &&  logical(allBeams(i).linearSpring)) || allBeams(i).degreesOfFreedom == 0
        if logical(allBeams(i).frontMasterID) || logical(allBeams(i).endMasterID)
            if logical(allBeams(i).frontMasterID)
                master=allBeams(i).frontMasterID;
            else
                master=allBeams(i).endMasterID;
            end
            outputList(allBeams(i).id)=allBeams(i).theta0+outputList(master);
            indexList(allBeams(i).id)=indexList(master);
        else
            outputList(allBeams(i).id)=allBeams(i).theta;
        end
        %check if linear spring
        if logical(allBeams(i).linearSpring)
            if ~logical(outputList(allBeams(i).id+1))
                outputList(allBeams(i).id+1)=inputMatrix(inputPosition);
                indexList(allBeams(i).id+1)=inputPosition;
                inputPosition=inputPosition+1;
                initialGuess(end+1)=allBeams(i).length;
            end
        end
    elseif allBeams(i).degreesOfFreedom >1
        if ~logical(outputList(allBeams(i).id))
            if logical(allBeams(i).theta)
                initialGuess(end+1)=allBeams(i).theta;
            else
                initialGuess(end+1)=1e-6;
            end
            outputList(allBeams(i).id)=inputMatrix(inputPosition);
            indexList(allBeams(i).id)=inputPosition;
            inputPosition=inputPosition+1;
        end
        if ~logical(outputList(allBeams(i).id+1))
            initialGuess(end+1)=allBeams(i).length;
            outputList(allBeams(i).id+1)=inputMatrix(inputPosition);
            indexList(allBeams(i).id+1)=inputPosition;
            inputPosition=inputPosition+1;
        end
    else
        if ~logical(outputList(allBeams(i).id))
            if logical(allBeams(i).frontMasterID) || logical(allBeams(i).endMasterID)
                if logical(allBeams(i).frontMasterID)
                    master=allBeams(i).frontMasterID;
                else
                    master=allBeams(i).endMasterID;
                end
                outputList(allBeams(i).id)=allBeams(i).theta0+outputList(master);
                indexList(allBeams(i).id)=indexList(master);
            else
                outputList(allBeams(i).id)=inputMatrix(inputPosition);
                indexList(allBeams(i).id)=inputPosition;
                inputPosition=inputPosition+1;
                if logical(allBeams(i).theta)
                    initialGuess(end+1)=allBeams(i).theta;
                else
                    initialGuess(end+1)=1e-6;
                end
            end
        end
    end
end
end


