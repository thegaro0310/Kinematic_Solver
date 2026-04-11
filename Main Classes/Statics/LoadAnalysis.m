classdef LoadAnalysis
    %LoadAnalysis Load Analysis Static Analysic
    properties
        static=Statics.empty;
        run=struct('state',[],'workspace',WorkSpace.empty,'power',[],'forces',Force.empty,'moments',Moment.empty,'torsionSprings',TorsionSpring.empty);
        runNumber=1;
        workspace=WorkSpace.empty;
        originalNodes;
    end
    
    methods
        
        function obj=LoadAnalysis(links,nodes,forces,moments,torsionSprings,workspace,varargin)
            %constructor
            %
            if isempty(varargin)
                obj.static=Statics(links,nodes,forces,moments,torsionSprings,workspace);
            else
                obj.static=varargin{1};
            end
            obj.originalNodes=obj.static.nodes;
            obj.workspace=workspace;
        end
        
        function obj=drawAll(obj,mainFig,limit,mode,parent,power)
            %draw all links
            obj.static.kinematic=obj.static.kinematic.drawAll(mainFig,limit,mode,parent);
            %plot forces
            for i=1:length(obj.static.forces)
                obj.static.forces(i)=obj.static.forces(i).drawStatic(parent,obj.static.nodes,obj.static.kinematic.allBeams,power);
            end
            %plot moments
            for i=1:length(obj.static.moments)
                obj.static.moments(i)=obj.static.moments(i).drawStatic(parent,obj.static.nodes,obj.static.kinematic.allBeams,power);
            end
            %plot torsion springs
            for i=1:length(obj.static.torsionSprings)
                obj.static.torsionSprings(i)=obj.static.torsionSprings(i).drawStatic(parent,obj.static.nodes,obj.static.kinematic.allBeams);
            end
        end
        
        function obj=drawNoGUI(obj,mainFig,limit,workspace,power)
            %draw in developer mode
            obj.static.kinematic=obj.static.kinematic.drawNoGUI(mainFig,limit,workspace);
            %plot torsion springs
            for i=1:length(obj.static.torsionSprings)
                obj.static.torsionSprings(i)=obj.static.torsionSprings(i).drawNoGUI(mainFig,workspace,obj.static.kinematic.nodes);
            end
            %plot moments
            for i=1:length(obj.static.moments)
                obj.static.moments(i)=obj.static.moments(i).drawNoGUI(mainFig,workspace,obj.static.kinematic.nodes,obj.static.kinematic.allBeams,power);
            end
            %plot forces
            for i=1:length(obj.static.forces)
                obj.static.forces(i).drawNoGUI(mainFig,workspace,obj.static.kinematic.nodes,obj.static.kinematic.allBeams,power);
            end
        end
        
        function obj=deleteAll(obj)
            %draw all links
            obj.static.kinematic=obj.static.kinematic.deleteAll();
            %delete forces
            for i=1:length(obj.static.forces)
                obj.static.forces(i)=obj.static.forces(i).deleteDrawing();
            end
            %delete moments
            for i=1:length(obj.static.moments)
                obj.static.moments(i)=obj.static.moments(i).deleteDrawing();
            end
            %delete torsion springs
            for i=1:length(obj.static.torsionSprings)
                obj.static.torsionSprings(i)=obj.static.torsionSprings(i).deleteDrawing();
            end
        end
        
        function obj=analysisNoGUI(obj,lowerBound,upperBound,increments,workspace)
            %the main analysis function
            %find the initial distances for non followers
            [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] );
            [outputMatrix,~,~] = obj.static.kinematic.findInputMatrix(initialGuess,[] );
            %store
            obj.run(obj.runNumber)=struct('workspace',workspace,'power',0,'forces',obj.static.forces,'moments',obj.static.moments,'torsionSprings',obj.static.torsionSprings,'state',[]);
            %
            power=linspace(lowerBound,upperBound,increments);
            for j=1:length(obj.static.forces)
                if ~logical(obj.static.forces(j).follower)
                    [ obj.static.forces(j).initialDistanceX, obj.static.forces(j).initialDistanceY,~,~ ] = obj.static.kinematic.solveStaticEquations(outputMatrix,obj.static.forces(j).kinematicEquations,[]);
                end
            end
            %create the input file for the function
            [allBeams,forces,moments]=getInputs(obj);
            for i=1:increments
                %find the initial distances
                [x,exitflag,~]= obj.fMinConMain( [],power(i),allBeams,forces,moments);
                if exitflag >0
                    %update beams
                    [newState,~,~] = obj.static.kinematic.findInputMatrix(x,[] );
                    for j=1:length(obj.static.kinematic.allBeams)
                        obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                    end
                    obj=obj.updateAdditionalDOF(x);
                    %update initial guess
                    allBeams=obj.updateInitialGuess(allBeams);
                    for j=1:length(obj.static.kinematic.allBeams)
                        obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                    end
                    obj.static.nodes=obj.static.kinematic.updateNodes(newState);
                    obj.static.kinematic.nodes=obj.static.nodes;
                    %save the data
                    obj.run(obj.runNumber).power(end+1)=power(i);
                    obj.run(obj.runNumber).state(end+1,:)=newState;
                else
                    break;
                end
            end
            %return to original
            for j=1:length(obj.static.kinematic.allBeams)
                obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(outputMatrix,outputMatrix);
            end
            obj.runNumber=obj.runNumber+1;
        end
        
        %update the addotional DOF
        function obj=updateAdditionalDOF(obj,newState)
            %add the additional inputs
            inputPosition=1;
            for i=1:length(obj.static.kinematic.additionalDof)
                if ~logical(obj.static.kinematic.additionalDof(i).known) 
                    obj.static.kinematic.additionalDof(i).value=newState(inputPosition);
                    inputPosition=inputPosition+1;
                end
            end
        end
        
        function obj=analysis(obj,lowerBound,upperBound,increments,mainFig,limit,mode,parent)
            %the main analysis function
            parent.setStop(0);
            %find the initial distances for non followers
            [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] );
            [outputMatrix,~,~] = obj.static.kinematic.findInputMatrix(initialGuess,[] );
            %store
            obj.run(obj.runNumber)=struct('workspace',parent.getWorkspace(),'power',0,'forces',obj.static.forces,'moments',obj.static.moments,'torsionSprings',obj.static.torsionSprings,'state',outputMatrix);
            %
            power=linspace(lowerBound,upperBound,increments);
            for j=1:length(obj.static.forces)
                if ~logical(obj.static.forces(j).follower)
                    [ obj.static.forces(j).initialDistanceX, obj.static.forces(j).initialDistanceY] = obj.static.kinematic.solveStaticEquations(outputMatrix,obj.static.forces(j).kinematicEquations);
                end
            end
            %create the input file for the function
            [allBeams,forces,moments]=getInputs(obj);
            for i=1:increments
                if ~logical(parent.getStop())
                    %find the initial distances
                    [x,exitflag,~]= obj.fMinConMain( [],power(i),allBeams,forces,moments);
                    if exitflag >0
                        %update beams
                        [newState,~,~] = obj.static.kinematic.findInputMatrix(x,[] );
                        for j=1:length(obj.static.kinematic.allBeams)
                            obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                        end
                        obj=obj.updateAdditionalDOF(x);
                        %update initial guess
                        allBeams=obj.updateInitialGuess(allBeams);
                        obj.static.nodes=obj.static.kinematic.updateNodes(newState);
                        obj.static.kinematic.nodes=obj.static.nodes;
                        %save the data
                        obj.run(obj.runNumber).power(end+1)=power(i);
                        obj.run(obj.runNumber).state(end+1,:)=newState;
                        %draw
                        obj=obj.drawAll(mainFig,limit,mode,parent,power(i));
                        %the strings
                        progressHandles=parent.getProgressBar();
                        try
                            progressHandles.progressText.String=['Increment:', num2str(i)];
                            patch([0,i/increments,i/increments,0],[0,0,1,1],Colors.StatusComplete.getColor(),'Parent',progressHandles.progress);
                        catch err
                            display(err);
                            display('Error displaying progress bar');
                        end
                        %draw
                        drawnow;
                    else
                        drawnow;
                        break;
                    end
                end
            end
            %return to original
            for j=1:length(obj.static.kinematic.allBeams)
                obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(outputMatrix,outputMatrix);
            end
            obj.runNumber=obj.runNumber+1;
        end
        
        %update initialGuess
        function beams=updateInitialGuess(obj,beams)
            for i=1:length(beams)
                switch beams(i).class
                    case 'CbcmBeam'
                        for j=1:length(beams(i).childLengths)
                            beams(i).initialGuess(j,:)=obj.static.kinematic.allBeams{i}.bcmBeams(j).getInitialGuess();
                        end
                     case 'MlinearBeam'
                        for j=1:length(beams(i).childLengths)
                            beams(i).initialGuess(j,:)=obj.static.kinematic.allBeams{i}.linearBeams(j).getInitialGuess();
                        end
                    case 'KinematicsBeam'
                        beams(i).initialGuess=obj.static.kinematic.allBeams{i}.calculateInitialGuess();
                end
            end
        end
        
        %create the input file for the function
        function [beams,forces,moments]=getInputs(obj)
            noOfBeams=length(obj.static.kinematic.allBeams);
            empty=cell(1,noOfBeams);
            beams=struct('id',empty,'class',empty,'frontMasterID',empty,'endMasterID',empty,'nodes',empty,'joints',empty,...
                'degreesOfFreedom',empty,'theta0',empty,'length0',empty,'initialGuess',empty,...
                'type',empty,'inPlaneThickness',empty,'outPlaneThickness',empty,...
                'E',empty,'childLengths',empty,'childIds',empty,'equations',empty,'fixed',empty,...
                'linearSpring',empty,'length',empty,'theta',empty,'prbBeams',empty,'torsionSprings',empty,'prbModel',empty);
            for i=1:length(obj.static.kinematic.allBeams)
                %common properties
                beams(i).id=obj.static.kinematic.allBeams{i}.id;
                beams(i).frontMasterID=obj.static.kinematic.allBeams{i}.frontMasterID;
                beams(i).endMasterID=obj.static.kinematic.allBeams{i}.endMasterID;
                beams(i).degreesOfFreedom=obj.static.kinematic.allBeams{i}.degreesOfFreedom;
                beams(i).theta0=obj.static.kinematic.allBeams{i}.theta0;
                beams(i).length0=obj.static.kinematic.allBeams{i}.length0;
                beams(i).joints=obj.static.kinematic.allBeams{i}.joints;
                beams(i).nodes=obj.static.kinematic.allBeams{i}.nodes;
                if isa(obj.static.kinematic.allBeams{i},'CbcmBeam')
                    %bcm properties
                    beams(i).inPlaneThickness=obj.static.kinematic.allBeams{i}.bcmBeams(1).crossSection.thickness;
                    beams(i).outPlaneThickness=obj.static.kinematic.allBeams{i}.bcmBeams(1).crossSection.width;
                    beams(i).E=obj.static.kinematic.allBeams{i}.bcmBeams(1).crossSection.E;
                    for j=1:obj.static.kinematic.allBeams{i}.segments
                        beams(i).childLengths(j)=obj.static.kinematic.allBeams{i}.bcmBeams(j).length0;
                        beams(i).childIds(j)=obj.static.kinematic.allBeams{i}.bcmBeams(j).id;
                        beams(i).initialGuess(j,:)=obj.static.kinematic.allBeams{i}.bcmBeams(j).getInitialGuess();
                    end
                    beams(i).class='CbcmBeam';
                elseif isa(obj.static.kinematic.allBeams{i},'MlinearBeam')
                    %mlinear properties
                    beams(i).inPlaneThickness=obj.static.kinematic.allBeams{i}.linearBeams(1).crossSection.thickness;
                    beams(i).outPlaneThickness=obj.static.kinematic.allBeams{i}.linearBeams(1).crossSection.width;
                    beams(i).E=obj.static.kinematic.allBeams{i}.linearBeams(1).crossSection.E;
                    for j=1:obj.static.kinematic.allBeams{i}.segments
                        beams(i).childLengths(j)=obj.static.kinematic.allBeams{i}.linearBeams(j).length0;
                        beams(i).childIds(j)=obj.static.kinematic.allBeams{i}.linearBeams(j).id;
                        beams(i).initialGuess(j,:)=obj.static.kinematic.allBeams{i}.linearBeams(j).getInitialGuess();
                    end
                    beams(i).class='MlinearBeam';
                elseif isa(obj.static.kinematic.allBeams{i},'PrbBeam')
                    %prb beam
                    beams(i).inPlaneThickness=obj.static.kinematic.allBeams{i}.crossSection.thickness;
                    beams(i).outPlaneThickness=obj.static.kinematic.allBeams{i}.crossSection.width;
                    beams(i).E=obj.static.kinematic.allBeams{i}.crossSection.E;
                    beams(i).prbModel=obj.static.kinematic.allBeams{i}.crossSection.prbModel;
                    beams(i).prbBeams=obj.static.kinematic.allBeams{i}.prbBeams;
                    beams(i).torsionSprings=obj.static.kinematic.allBeams{i}.torsionSprings;
                    beams(i).initialGuess=obj.static.kinematic.allBeams{i}.getInitialGuess();
                    beams(i).class='PrbBeam';
                else
                    %kinematic beam properties
                    beams(i).equations=obj.static.kinematic.allBeams{i}.equation;
                    beams(i).initialGuess=obj.static.kinematic.allBeams{i}.calculateInitialGuess();
                    beams(i).class='KinematicsBeam';
                    beams(i).type=obj.static.kinematic.allBeams{i}.type;
                    beams(i).linearSpring=obj.static.kinematic.allBeams{i}.linearSpring;
                    beams(i).fixed=obj.static.kinematic.allBeams{i}.fixed;
                    beams(i).theta=obj.static.kinematic.allBeams{i}.theta;
                    beams(i).length=obj.static.kinematic.allBeams{i}.length;
                end
            end
            %get forces
            empty=cell(1,length(obj.static.forces));
            forces=struct('xValue',empty,'yValue',empty,'active',empty,...
                'initialDistanceX',empty,'initialDistanceY',empty,...
                'kinematicEquations',empty);
            for i=1:length(obj.static.forces)
                forces(i).xValue=obj.static.forces(i).xValue;
                forces(i).yValue=obj.static.forces(i).yValue;
                forces(i).active=obj.static.forces(i).active;
                forces(i).initialDistanceX=obj.static.forces(i).initialDistanceX;
                forces(i).initialDistanceY=obj.static.forces(i).initialDistanceY;
                forces(i).kinematicEquations=obj.static.forces(i).kinematicEquations;
            end
            %get moments
            empty=cell(1,length(obj.static.moments));
            moments=struct('magnitude',empty,'link',empty,'distance',empty,...
                'angle0',empty,'active',empty);
            for i=1:length(obj.static.moments)
                moments(i).magnitude=obj.static.moments(i).magnitude;
                moments(i).link=obj.static.moments(i).link;
                moments(i).distance=obj.static.moments(i).distance;
                moments(i).angle0=obj.static.moments(i).angle0;
                moments(i).active=obj.static.moments(i).active;
            end
        end
        
        function [newState,exitflag,stable ] = fMinConMain(obj,input,power,allBeams,forces,moments)
            %run the optimization
            [newState,exitflag,stable]=solveStaticAnalysis(allBeams,obj.static.torsionSprings,obj.static.kinematic.equations,...
                forces,moments,input,power,obj.static.kinematic.additionalDof,...
                obj.workspace.lengthFactor(),obj.workspace.forceFactor(),obj.workspace.EFactor());
        end
        
        function [f,g] = fMinConFunc(obj,x,input,power)
            %FMINCONFUNC this functions calculates potantial energy which will be
            %minimized
            %             x(end)=pi/15;
            %             x(end-1)=pi/15;
            %             x(end-2)=pi/15;
            %             x(end-3)=pi/15;
            [outputMatrix,indexList,~] = obj.static.kinematic.findInputMatrix(x,input );
            g=zeros(max(indexList),1);
            %
            %beams
            %update beams
            %             for i=1:length(obj.static.kinematic.allBeams)
            %                 obj.static.kinematic.allBeams{i}=obj.static.kinematic.allBeams{i}.updateBeam(outputMatrix,indexList);
            %             end
            energy=0;
            for i=1:length(obj.static.kinematic.allBeams)
                %energy stored in beams
                g=g+obj.static.kinematic.allBeams{i}.getGradient(outputMatrix,indexList,obj.workspace);
                energy=energy+obj.static.kinematic.allBeams{i}.getEnergy(outputMatrix,indexList,obj.workspace);
            end
            %torsion springs
            for i=1:length(obj.static.torsionSprings)
                if logical(obj.static.torsionSprings(i).active)
                    magnitude=obj.static.torsionSprings(i).stiffness*obj.workspace.torsionSpringFactor();
                    node=obj.static.torsionSprings(i).node;
                    link1=obj.static.torsionSprings(i).link1;
                    link2=obj.static.torsionSprings(i).link2;
                    %check link 1
                    if link1 >0
                        if node == obj.static.kinematic.allBeams{link1}.nodes(1,1)
                            [angle1,index1]=obj.static.kinematic.allBeams{link1}.getAngle(0.0);
                        elseif node == obj.static.kinematic.allBeams{link1}.nodes(1,2)
                            [angle1,index1]=obj.static.kinematic.allBeams{link1}.getAngle(100.0);
                        end
                    end
                    %check link 2
                    if link2 >0
                        if node == obj.static.kinematic.allBeams{link2}.nodes(1,1)
                            [angle2,index2]=obj.static.kinematic.allBeams{link2}.getAngle(0.0);
                        elseif node == obj.static.kinematic.allBeams{link2}.nodes(1,2)
                            [angle2,index2]=obj.static.kinematic.allBeams{link2}.getAngle(100.0);
                        end
                    end
                    if obj.static.torsionSprings(i).link1==0 && obj.static.torsionSprings(i).link2 ==0
                        continue;
                    elseif obj.static.torsionSprings(i).link1==0
                        energy=energy+1/2*magnitude*(angle2 -obj.static.torsionSprings(i).theta0(1,2))^2;
                        %gradient
                        if logical(index2)
                            g(index2,1)=g(index2,1)+magnitude*(angle2 -obj.static.torsionSprings(i).theta0(1,2));
                        end
                    elseif obj.static.torsionSprings(i).link2==0
                        energy=energy+1/2*magnitude*(  angle1 -obj.static.torsionSprings(i).theta0(1,1))^2;
                        %gradient
                        if logical(index1)
                            g(index1,1)=g(index1,1)+magnitude*(  angle1 -obj.static.torsionSprings(i).theta0(1,1));
                        end
                    else
                        energy=energy+1/2*magnitude*( ( angle1 -obj.static.torsionSprings(i).theta0(1,1))-( angle2-obj.static.torsionSprings(i).theta0(1,2)))^2;
                        %gradient
                        if logical(index1)
                            g(index1,1)=g(index1,1)+magnitude*( ( angle1 -obj.static.torsionSprings(i).theta0(1,1))-( angle2-obj.static.torsionSprings(i).theta0(1,2)));
                        end
                        if logical(index2)
                            g(index2,1)=g(index2,1)-magnitude*( ( angle1 -obj.static.torsionSprings(i).theta0(1,1))-( angle2-obj.static.torsionSprings(i).theta0(1,2)));
                        end
                    end
                end
            end
            
            
            forceEnergy=0;
            for i=1:length(obj.static.forces)
                if logical(obj.static.forces(i).active)
                    [ xValue,yValue,gradient,~] = obj.static.kinematic.solveStaticEquations(outputMatrix,obj.static.forces(i).kinematicEquations,indexList);
                    if ~logical(obj.static.forces(i).follower)
                        %not follower
                        magnitudeX=obj.static.forces(i).xValue*obj.workspace.forceFactor();
                        magnitudeY=obj.static.forces(i).yValue*obj.workspace.forceFactor();
                        xDistance=(xValue-obj.static.forces(i).initialDistanceX)*obj.workspace.lengthFactor();
                        yDistance=(yValue-obj.static.forces(i).initialDistanceY)*obj.workspace.lengthFactor();
                        forceEnergy=forceEnergy+xDistance*magnitudeX*power/100+yDistance*magnitudeY*power/100;
                        g=g-gradient(:,1)*obj.workspace.lengthFactor()*magnitudeX*power/100-gradient(:,2)*obj.workspace.lengthFactor()*magnitudeY*power/100;
                    end
                end
            end
            
            for i=1:length(obj.static.moments)
                if logical(obj.static.moments(i).active)
                    [angle,index]=obj.static.kinematic.allBeams{obj.static.moments(i).link}.getAngle(obj.static.moments(i).distance);
                    magnitude=obj.static.moments(i).magnitude*obj.workspace.momentFactor() ;
                    forceEnergy=forceEnergy+magnitude*(angle-obj.static.moments(i).angle0)*power/100;
                if index > 0
                    g(index,1)=g(index,1)-magnitude*power/100;
                end
                end
            end
            f=(energy-forceEnergy)/(obj.workspace.lengthFactor()*obj.workspace.forceFactor());
            g=g/(obj.workspace.lengthFactor()*obj.workspace.forceFactor());
        end
        
        function [ c,ceq,gradc,gradceq ] = fMinConCostraint(obj,x,input)
            %FMINCONCOSTRAINT Constraint function for fMinCon
            %             x(end)=pi/15;
            %             x(end-1)=pi/15;
            %             x(end-2)=pi/15;
            %             x(end-3)=pi/15;
            [outputMatrix,indexList,~] = obj.static.kinematic.findInputMatrix(x,input );
            [ xValue,yValue,gradient,~ ] = obj.static.kinematic.solveStaticEquations(outputMatrix,obj.static.kinematic.equations,indexList);
            c=[];
            ceq=zeros(length(obj.static.kinematic.equations)*2,1);
            for i=1:length(obj.static.kinematic.equations)
                ceq(2*i-1,1)=xValue(i);
                ceq(2*i,1)=yValue(i);
            end
            if nargout > 2
                gradc = [];
                gradceq = gradient;
            end
        end
        
        function Hout = hessianfcn(obj,x,lambda,input,power)
            %hessian matrix during energy minimization
            %             x(end)=pi/15;
            %             x(end-1)=pi/15;
            %             x(end-2)=pi/15;
            %             x(end-3)=pi/15;
            Hout=sparse(length(x),length(x));
            [inputMatrix,indexList,~] = obj.static.kinematic.findInputMatrix(x,input );
            %beams
            %             %update beams
            %             for i=1:length(obj.static.kinematic.allBeams)
            %                 obj.static.kinematic.allBeams{i}=obj.static.kinematic.allBeams{i}.updateBeam(inputMatrix,indexList);
            %             end
            for i=1:length(obj.static.kinematic.allBeams)
                %energy stored in beams
                Hout=Hout+obj.static.kinematic.allBeams{i}.getHessian(inputMatrix,indexList,obj.workspace);
            end
            %torsion springs
            for i=1:length(obj.static.torsionSprings)
                if logical(obj.static.torsionSprings(i).active)
                    magnitude=obj.static.torsionSprings(i).stiffness*obj.workspace.torsionSpringFactor();
                    node=obj.static.torsionSprings(i).node;
                    link1=obj.static.torsionSprings(i).link1;
                    link2=obj.static.torsionSprings(i).link2;
                    %check link 1
                    if link1 >0
                        if node == obj.static.kinematic.allBeams{link1}.nodes(1,1)
                            [angle1,index1]=obj.static.kinematic.allBeams{link1}.getAngle(0.0);
                        elseif node == obj.static.kinematic.allBeams{link1}.nodes(1,2)
                            [angle1,index1]=obj.static.kinematic.allBeams{link1}.getAngle(100.0);
                        end
                    end
                    %check link 2
                    if link2 >0
                        if node == obj.static.kinematic.allBeams{link2}.nodes(1,1)
                            [angle2,index2]=obj.static.kinematic.allBeams{link2}.getAngle(0.0);
                        elseif node == obj.static.kinematic.allBeams{link2}.nodes(1,2)
                            [angle2,index2]=obj.static.kinematic.allBeams{link2}.getAngle(100.0);
                        end
                    end
                    if obj.static.torsionSprings(i).link1==0 && obj.static.torsionSprings(i).link2 ==0
                        continue;
                    elseif obj.static.torsionSprings(i).link1==0
                        %hessian
                        if logical(index2)
                            Hout(index2,index2)=Hout(index2,index2)+magnitude;
                        end
                    elseif obj.static.torsionSprings(i).link2==0
                        %hessian
                        if logical(index1)
                            Hout(index1,index1)=Hout(index1,index1)+magnitude;
                        end
                    else
                        %hessian
                        if logical(index2)
                            Hout(index2,index2)=Hout(index2,index2)+magnitude;
                        end
                        if logical(index1)
                            Hout(index1,index1)=Hout(index1,index1)+magnitude;
                        end
                        if logical(index1) && logical(index2)
                            Hout(index1,index2)=Hout(index1,index2)-magnitude;
                            Hout(index2,index1)=Hout(index2,index1)-magnitude;
                        end
                    end
                end
            end
            
            
            % for i=1:length(forces)
            %     [ forces(i).initialDistanceX, forces(i).initialDistanceY ] = solveKinematicEquations(inputMatrix,rigidBeamList,forces(i).kinematicEquations);
            % end
            for i=1:length(obj.static.forces)
                if logical(obj.static.forces(i).active)
                    [ ~,~,~,hessian] = obj.static.kinematic.solveStaticEquations(inputMatrix,obj.static.forces(i).kinematicEquations,indexList);
                    if ~logical(obj.static.forces(i).follower)
                        %not follower
                        magnitudeX=obj.static.forces(i).xValue*obj.workspace.forceFactor();
                        magnitudeY=obj.static.forces(i).yValue*obj.workspace.forceFactor();
                        Hout=Hout-hessian{1}*obj.workspace.lengthFactor()*magnitudeX*power/100-hessian{2}*obj.workspace.lengthFactor()*magnitudeY*power/100;
                    end
                end
            end
            Hout=Hout/(obj.workspace.lengthFactor()*obj.workspace.forceFactor());
            [ ~,~,~,hessian ] = obj.static.kinematic.solveStaticEquations(inputMatrix,obj.static.kinematic.equations,indexList);
            for i=1:length(lambda.eqnonlin)
                Hout=Hout+lambda.eqnonlin(i)*hessian{i};
            end
        end
        
    end
    
end
