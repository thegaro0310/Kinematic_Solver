classdef BiStable
    %bistable analysis
    
    properties
        static=Statics.empty;
        distance=struct('type',[],'member',0,'target',[],'line',[],'selected',0);
        selectedLink=0;
        selectedNode=0;
        busy=0;
        run=struct('state',[],'workspace',WorkSpace.empty,'power',[],'forces',Force.empty,'moments',Moment.empty,'torsionSprings',TorsionSpring.empty,'distance',[],'load',[],...
            'energy',[]);
        runNumber=1;
    end
    
    methods
        function obj=BiStable(links,nodes,torsionSprings,workspace)
            %constructor
            obj.static=Statics(links,nodes,[],[],torsionSprings,workspace);
        end
        
        function obj=drawDistance(obj,mainFig,limit)
            %draw the distance
            target=obj.distance.target(1,2:3);
            if obj.distance.type == 1
                %node type
                link=obj.getSlider(obj.distance.member);
                theta=obj.static.kinematic.allBeams{link}.sliderAngle;
                node=obj.static.nodes(obj.distance.member).getNode();
                %x motion
                obj.distance.line(1)=plot(mainFig,target(1)*cos(theta)+[node.x node.x],target(1)*sin(theta)+[node.y node.y]+[limit*0.2 -limit*0.2],'LineStyle','--','Color',Colors.NormalLink.getColor());
                obj.distance.line(2)=plot(mainFig,target(2)*cos(theta)+[node.x node.x],target(2)*sin(theta)+[node.y node.y]+[limit*0.2 -limit*0.2],'LineStyle','--','Color',Colors.NormalLink.getColor());
                %y motion
                obj.distance.line(3)=plot(mainFig,target(1)*cos(theta)+[node.x node.x]+[limit*0.2 -limit*0.2],target(1)*sin(theta)+[node.y node.y],'LineStyle','--','Color',Colors.NormalLink.getColor());
                obj.distance.line(4)=plot(mainFig,target(2)*cos(theta)+[node.x node.x]+[limit*0.2 -limit*0.2],target(2)*sin(theta)+[node.y node.y],'LineStyle','--','Color',Colors.NormalLink.getColor());
            else
                %link type
                link=obj.static.kinematic.allBeams{obj.distance.member};
                linkLength=max(link.length0,limit/3);
                linkAngle=link.theta0;
                node=obj.static.nodes(link.nodes(1,1)).getNode();
                if obj.distance.target(1,1) == 1
                    %CW motion
                    newAngle=linkAngle-target(1)*pi/180;
                    newAngle2=linkAngle-target(2)*pi/180;
                else
                    %CCW motion
                    newAngle=linkAngle+target(1)*pi/180;
                    newAngle2=linkAngle+target(2)*pi/180;
                end
                obj.distance.line(1)=plot(mainFig,[node.x node.x+linkLength*cos(newAngle) ],[node.y node.y+linkLength*sin(newAngle) ],'LineStyle','--','Color',Colors.NormalLink.getColor());
                obj.distance.line(2)=plot(mainFig,[node.x node.x+linkLength*cos(newAngle2) ],[node.y node.y+linkLength*sin(newAngle2) ],'LineStyle','--','Color',Colors.NormalLink.getColor());
            end
        end
        
        function link=getSlider(obj,nodeID)
            %get link from node id
            link=0;
            for i=1:length(obj.static.kinematic.allBeams)
                if obj.static.kinematic.allBeams{i}.nodes(1,2) == nodeID && obj.static.kinematic.allBeams{i}.joints(1,2) == Joint.GroundSlider
                    link=i;
                end
            end
        end
        
        function obj=drawNoGUI(obj,mainFig,limit,workspace)
            %draw in developer mode
            obj.static.kinematic=obj.static.kinematic.drawNoGUI(mainFig,limit,workspace);
            %plot torsion springs
            for i=1:length(obj.static.torsionSprings)
                obj.static.torsionSprings(i)=obj.static.torsionSprings(i).drawNoGUI(mainFig,workspace,obj.static.kinematic.nodes);
            end
            %plot moments
            for i=1:length(obj.static.moments)
                obj.static.moments(i)=obj.static.moments(i).drawNoGUI(mainFig,workspace,obj.static.kinematic.nodes,obj.static.kinematic.allBeams,100);
            end
            %plot forces
            for i=1:length(obj.static.forces)
                obj.static.forces(i).drawNoGUI(mainFig,workspace,obj.static.kinematic.nodes,obj.static.kinematic.allBeams,100);
            end
            %draw distance
            obj.drawDistance(mainFig,limit);
        end
        
        function obj=drawAll(obj,mainFig,limit,mode,parent)
            %draw all links
            obj.static.kinematic=obj.static.kinematic.drawAll(mainFig,limit,mode,parent);
            %plot torsion springs
            for i=1:length(obj.static.torsionSprings)
                obj.static.torsionSprings(i)=obj.static.torsionSprings(i).drawStatic(parent,obj.static.nodes,obj.static.kinematic.allBeams);
            end
            %plot moments
            for i=1:length(obj.static.moments)
                obj.static.moments(i)=obj.static.moments(i).drawStatic(parent,obj.static.nodes,obj.static.kinematic.allBeams,100);
            end
            %plot forces
            for i=1:length(obj.static.forces)
                obj.static.forces(i)=obj.static.forces(i).drawStatic(parent,obj.static.nodes,obj.static.kinematic.allBeams,100);
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
            %delete distance
            if isgraphics(obj.distance.line)
                delete(obj.distance.line);
            end
            %delete nodes
            for i=1:length(obj.static.nodes)
                obj.static.nodes(i)=obj.static.nodes(i).deleteNodeDrawing;
            end
        end
        
        function obj=updateDistance(obj)
            %update the initial nodes and links
            if obj.distance.type == 1
                %node type
                if obj.distance.target(1) == 1
                    for i=1:length(obj.static.nodes)
                        obj.static.nodes(i)=obj.static.nodes(i).setSelected(0);
                    end
                    obj.static.nodes(obj.distance.member)=obj.static.nodes(obj.distance.member).setSelected(1);
                end
            else
                %link type
                for i=1:length(obj.static.kinematic.allBeams)
                    obj.static.kinematic.allBeams{i}.selected=0;
                end
                obj.static.kinematic.allBeams{obj.distance.member}.selected=1;
            end
        end
        
        function obj=setSelectedLink(obj,id,select)
            %set the selected link
            obj.static.kinematic.allBeams{id}.selected=select;
        end
        
        function obj=setSelectedNode(obj,id,select)
            %set the selected link
            obj.static.nodes(id)=obj.static.nodes(id).setSelected(select);
        end
        
        function drawDistanceNoGUI(obj,limit)
            %draw the distance
            target=obj.distance.target(1,2);
            if obj.distance.type == 1
                %node type
                node=obj.static.nodes(obj.distance.member).getNode();
                if obj.distance.target(1,1) == 1
                    %x motion
                    plot(target+[node.x node.x],node.y+[limit*0.2 -limit*0.2],'LineStyle','--','Color',Colors.NormalLink.getColor());
                else
                    %y motion
                    plot(node.x+[limit*0.2 -limit*0.2],target+[node.y node.y],'LineStyle','--','Color',Colors.NormalLink.getColor());
                end
            else
                %link type
                link=obj.static.kinematic.allBeams{obj.distance.member};
                linkLength=max(link.initialLength,limit/3);
                linkAngle=link.angle0;
                node=obj.static.nodes(link.nodes(1,1)).getNode();
                if obj.distance.target(1,1) == 1
                    %CW motion
                    newAngle=linkAngle-obj.distance.target(1,2)*pi/180;
                else
                    %CCW motion
                    newAngle=linkAngle+obj.distance.target(1,2)*pi/180;
                end
                plot([node.x node.x+linkLength*cos(newAngle) ],[node.y node.y+linkLength*sin(newAngle) ],'LineStyle','--','Color',Colors.NormalLink.getColor());
            end
        end
        
        
        function obj=addInput(obj,member,target,type,links)
            %add range input
            switch type
                case 'slider'
                    obj.distance=struct('type',1,'member',member,'target',[1 target],'line',[],'selected',0);
                case 'angle'
                    linkID=Helper.findLink(member,links,obj.static.kinematic.allBeams,3);
                    if target < 0
                        obj.distance=struct('type',2,'member',linkID,'target',[1 abs(target)],'line',[],'selected',0);
                    else
                        obj.distance=struct('type',2,'member',linkID,'target',[2 target],'line',[],'selected',0);
                    end
            end
        end
        
        function obj=simulationNoGUI(obj,workspace,numberOfDistance)
            %main analysis function
            %
            if obj.distance.type == 1
                %node type
                selectedLink=0;
                for i=1:length(obj.static.kinematic.allBeams)
                    if obj.static.kinematic.allBeams{i}.joints(1,2) == Joint.GroundSlider && obj.static.kinematic.allBeams{i}.nodes(1,2) == obj.distance.member
                        selectedLink=i;
                        break;
                    end
                end
                initialDistance=obj.distance.target(1,2);
                targetDistance=obj.distance.target(1,3);
            else
                %link type
                selectedLink=obj.distance.member;
                if obj.distance.target(1,1) == 1
                    %CW motion
                    initialDistance=obj.static.kinematic.allBeams{selectedLink}.theta-obj.distance.target(1,2)*pi/180;
                    targetDistance=obj.static.kinematic.allBeams{selectedLink}.theta-obj.distance.target(1,3)*pi/180;
                else
                    %CCW motion
                    initialDistance=obj.static.kinematic.allBeams{selectedLink}.theta+obj.distance.target(1,2)*pi/180;
                    targetDistance=obj.static.kinematic.allBeams{selectedLink}.theta+obj.distance.target(1,3)*pi/180;
                end
            end
            %original nodes
            originalNodes=obj.static.nodes;
            %save
            obj.run(obj.runNumber)=struct('state',[],'workspace',workspace,'power',100,'forces',Force.empty,'moments',Moment.empty,'torsionSprings',obj.static.torsionSprings,'distance',obj.distance,'load',[],...
                'energy',[]);
            %initial state
            [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] );
            [initialState,~,~] = obj.static.kinematic.findInputMatrix(initialGuess,[] );
            %obj.run(obj.runNumber).state(1,:)=initialState;
            %angle conditions
            targetDistances=linspace(initialDistance,targetDistance,numberOfDistance);
            deltaDistance=targetDistances(2)-targetDistances(1);
            %initial
            input.index=selectedLink;
            input.dof=1;
            loadList=zeros(1,numberOfDistance-1);
            energyList=zeros(1,numberOfDistance);
            energyDistances=zeros(1,numberOfDistance);
            if  obj.distance.type == 1
                energyDistances(1)=initialDistance;
            else
                %link
                energyDistances(1)=initialDistance*180/pi;
            end
            [~,~,initialGuess0] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] ) ;
            [initialState,~,~] = obj.static.kinematic.findInputMatrix(initialGuess0,[] );
            %main loop
            tic
            for i=1:numberOfDistance
                disp('****************************');
                %input value
                if obj.distance.type == 2
                    %link type
                    input.value=targetDistances(i);
                else
                    %node type
                    theta=obj.static.kinematic.allBeams{selectedLink}.sliderAngle;
                    node1=originalNodes(obj.static.kinematic.allBeams{selectedLink}.nodes(1,1)).getNode();
                    node2i=originalNodes(obj.static.kinematic.allBeams{selectedLink}.nodes(1,2)).getNode();
                    node2=Point(node2i.x+targetDistances(i)*cos(theta),node2i.y+targetDistances(i)*sin(theta));
                    newLength=sqrt((node2.y-node1.y)^2+(node2.x-node1.x)^2);
                    angle=atan2((node2.y-node1.y),(node2.x-node1.x));
                    input.value=(newLength*cos(angle))*cos(theta)+(newLength*sin(angle))*sin(theta);
                end
                fMincon=@(x)obj.mainFunctionfunction(x,input,workspace);
                gMinCon=@(x)obj.constraintFunction( x,input);
                opt=optimoptions(@fmincon,'Display','iter-detailed','Algorithm','interior-point','MaxFunEvals',5e3,'MaxIter',5e3',...
                    'ConstraintTolerance',1e-8,'FunctionTolerance',1e-12,'OptimalityTolerance',1e-12,....
                    'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true...
                    ,'HessianFcn',@(x,lambda)obj.hessianfcn(x,lambda,input,workspace)...
                    ,'FiniteDifferenceType','central','CheckGradients',false);
                %initial guess
                [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),input ) ;
                [x,fval,exitflag,output,lambda,grad,hessian] = fmincon(fMincon,initialGuess,[],[],[],[],[],[],gMinCon,opt);
                if exitflag <=0
                    exitflag
                    energyList(i:end)=[];
                    loadList(i-1:end)=[];
                    break;
                end
                energy=obj.mainFunctionfunction(x,input,workspace);
                [newState,~,~] = obj.static.kinematic.findInputMatrix(x,input ) ;
                for j=1:length(obj.static.kinematic.allBeams)
                    obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                end
                updatedNodes=obj.static.kinematic.updateNodes(newState);
                obj.static.nodes=updatedNodes;
                obj.static.kinematic.nodes=updatedNodes;
                %calculate energy
                energyList(i)=energy(1);
                if obj.distance.type == 2
                    %link type
                    energyDistances(i)=targetDistances(i)*180/pi;
                else
                    energyDistances(i)=targetDistances(i);
                end
                if i>2
                    loadList(i-1)=(energyList(i)-energyList(i-2))/(2*deltaDistance);
                elseif i == 2
                    if energyList(1)> 1e-10
                        loadList(1)=(energyList(2)-energyList(1))/(deltaDistance);
                    else
                        loadList(1)=0;
                    end
                end
                %save
                obj.run(obj.runNumber).state(end+1,:)=newState;
                obj.run(obj.runNumber).power(end+1)=100;
                
            end
            toc
            for j=1:length(obj.static.kinematic.allBeams)
                obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(initialState,initialState);
            end
            obj.static.nodes=originalNodes;
            obj.static.kinematic.nodes=originalNodes;
            obj.run(obj.runNumber).load=loadList;
            obj.run(obj.runNumber).energy=energyList;
            obj.runNumber=obj.runNumber+1;
        end
        
        
        %create the input file for the function
        function beams=getInputs(obj)
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
        end
        
        
        
        function obj=energyPlot(obj,mainFig,limit,mode,parent)
            %main analysis function
            parent.setStop(0);
            %
            if obj.distance.type == 1
                %node type
                selectedLink=0;
                for i=1:length(obj.static.kinematic.allBeams)
                    if obj.static.kinematic.allBeams{i}.joints(1,2) == Joint.GroundSlider && obj.static.kinematic.allBeams{i}.nodes(1,2) == obj.distance.member
                        selectedLink=i;
                        break;
                    end
                end
                initialDistance=obj.distance.target(1,2);
                targetDistance=obj.distance.target(1,3);
            else
                %link type
                selectedLink=obj.distance.member;
                if obj.distance.target(1,1) == 1
                    %CW motion
                    initialDistance=obj.static.kinematic.allBeams{selectedLink}.theta-obj.distance.target(1,2)*pi/180;
                    targetDistance=obj.static.kinematic.allBeams{selectedLink}.theta-obj.distance.target(1,3)*pi/180;
                else
                    %CCW motion
                    initialDistance=obj.static.kinematic.allBeams{selectedLink}.theta+obj.distance.target(1,2)*pi/180;
                    targetDistance=obj.static.kinematic.allBeams{selectedLink}.theta+obj.distance.target(1,3)*pi/180;
                end
            end
            %original nodes
            originalNodes=obj.static.nodes;
            %save
            obj.run(obj.runNumber)=struct('state',[],'workspace',parent.getWorkspace(),'power',100,'forces',Force.empty,'moments',Moment.empty,'torsionSprings',obj.static.torsionSprings,'distance',obj.distance,'load',[],...
                'energy',[]);
            %initial state
            [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] );
            [initialState,~,~] = obj.static.kinematic.findInputMatrix(initialGuess,[] );
            %obj.run(obj.runNumber).state(1,:)=initialState;
            %angle conditions
            numberOfDistance=100;
            targetDistances=linspace(initialDistance,targetDistance,numberOfDistance);
            deltaDistance=targetDistances(2)-targetDistances(1);
            %initial
            input.index=selectedLink;
            input.dof=1;
            loadList=zeros(1,numberOfDistance-1);
            energyList=zeros(1,numberOfDistance);
            energyDistances=zeros(1,numberOfDistance);
            if  obj.distance.type == 1
                energyDistances(1)=initialDistance;
            else
                %link
                energyDistances(1)=initialDistance*180/pi;
            end
            [~,~,initialGuess0] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] ) ;
            [initialState,~,~] = obj.static.kinematic.findInputMatrix(initialGuess0,[] );
            %plot preparation
            scrsz = get(groot,'ScreenSize');
            energyPlot=figure();
            reset(energyPlot);
            energyPlot.Position=[1 scrsz(4)/3 scrsz(3)/3 scrsz(4)/3];
            if obj.distance.type == 2
                %link type
                plot1=subplot(2,1,1);
                xlabel('input link angle(deg)');
                xlim([min([initialDistance targetDistances(end)]) max([initialDistance targetDistances(end)])]*180/pi );
                if parent.getWorkspace().momentFactor() == 1e6
                    ylabel('Megajoule');
                elseif parent.getWorkspace().momentFactor() == 1e3
                    ylabel('Kilojoule');
                elseif parent.getWorkspace().momentFactor() == 1
                    ylabel('Joule');
                elseif parent.getWorkspace().momentFactor() == 1e-3
                    ylabel('Millijoule');
                elseif parent.getWorkspace().momentFactor() == 1e-6
                    ylabel('Microjoule');
                elseif parent.getWorkspace().momentFactor() == 1e-9
                    ylabel('Nanojoule');
                else
                    lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                    loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
                    ylabel([loadString '.' lengthString]);
                end
                title('Total Potential Energy');
                plot2=subplot(2,1,2);
                xlabel('input link angle(deg)');
                xlim([min([initialDistance targetDistances(end)]) max([initialDistance targetDistances(end)])]*180/pi );
                lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
                ylabel([loadString '.' lengthString]);
                title('Torque Magnitude');
            else
                %node type
                plot1=subplot(2,1,1);
                lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                xlim([min([initialDistance targetDistances(end)]) max([initialDistance targetDistances(end)])] );
                xlabel(['slider motion','(',lengthString,')']);
                if parent.getWorkspace().momentFactor() == 1
                    ylabel('Joule');
                elseif parent.getWorkspace().momentFactor() == 1e-3
                    ylabel('mili-Joule');
                else
                    lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                    loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
                    ylabel([loadString '.' lengthString]);
                end
                title('Total Potential Energy');
                plot2=subplot(2,1,2);
                xlim([min([initialDistance targetDistances(end)]) max([initialDistance targetDistances(end)])] );
                xlabel(['slider motion','(',lengthString,')']);
                ylabel(ForceUnit.getString(parent.getWorkspace().unitForce));
                title('Force Magnitude');
            end
            hold(plot1,'on');hold(plot2,'on');
            grid(plot1,'on');grid(plot2,'on');
            grid(plot1,'minor');grid(plot2,'minor');
            %main loop
            tic
            for i=1:numberOfDistance
                disp('****************************');
                if ~logical(parent.getStop())
                    %input value
                    if obj.distance.type == 2
                        %link type
                        input.value=targetDistances(i);
                    else
                        %node type
                        theta=obj.static.kinematic.allBeams{selectedLink}.sliderAngle;
                        node1=originalNodes(obj.static.kinematic.allBeams{selectedLink}.nodes(1,1)).getNode();
                        node2i=originalNodes(obj.static.kinematic.allBeams{selectedLink}.nodes(1,2)).getNode();
                        node2=Point(node2i.x+targetDistances(i)*cos(theta),node2i.y+targetDistances(i)*sin(theta));
                        newLength=sqrt((node2.y-node1.y)^2+(node2.x-node1.x)^2);
                        angle=atan2((node2.y-node1.y),(node2.x-node1.x));
                        input.value=(newLength*cos(angle))*cos(theta)+(newLength*sin(angle))*sin(theta);
                    end
                    %                     fMincon=@(x)obj.mainFunctionfunction(x,input,parent.getWorkspace());
                    %                     gMinCon=@(x)obj.constraintFunction( x,input);
                    %                     opt=optimoptions(@fmincon,'Display','iter-detailed','Algorithm','interior-point','MaxFunEvals',5e3,'MaxIter',5e3',...
                    %                         'ConstraintTolerance',1e-8,'FunctionTolerance',1e-12,'OptimalityTolerance',1e-12,....
                    %                         'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true...
                    %                         ,'HessianFcn',@(x,lambda)obj.hessianfcn(x,lambda,input,parent.getWorkspace())...
                    %                         ,'FiniteDifferenceType','central','CheckGradients',false);
                    %                     %initial guess
                    %                     [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),input ) ;
                    %                     [x,fval,exitflag,output,lambda,grad,hessian] = fmincon(fMincon,initialGuess,[],[],[],[],[],[],gMinCon,opt);
                    %                     if exitflag <=0
                    %                         exitflag
                    %                         energyList(i:end)=[];
                    %                         loadList(i-1:end)=[];
                    %                         break;
                    %                     end
                    %                     energy=obj.mainFunctionfunction(x,input,parent.getWorkspace());
                    allBeams=obj.getInputs();
                    [newState,exitflag,energy]=bistableAnalysis(allBeams,obj.static.torsionSprings,obj.static.kinematic.equations,...
                        input,obj.static.kinematic.additionalDof,...
                        obj.static.workspace.lengthFactor(),obj.static.workspace.forceFactor(),obj.static.workspace.EFactor());
                    if exitflag <=0
                        exitflag
                        energyList(i:end)=[];
                        loadList(i-1:end)=[];
                        break;
                    end
                    [newState,~,~] = obj.static.kinematic.findInputMatrix(newState,input ) ;
                    for j=1:length(obj.static.kinematic.additionalDof)
                        obj.static.kinematic.additionalDof(j).value=newState(j);
                    end
                    for j=1:length(obj.static.kinematic.allBeams)
                        obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                    end
                    updatedNodes=obj.static.kinematic.updateNodes(newState);
                    obj.static.nodes=updatedNodes;
                    obj.static.kinematic.nodes=updatedNodes;
                    obj=obj.drawAll(mainFig,limit,mode,parent);
                    %calculate energy
                    energyList(i)=energy(1);
                    if obj.distance.type == 2
                        %link type
                        energyDistances(i)=targetDistances(i)*180/pi;
                    else
                        energyDistances(i)=targetDistances(i);
                    end
                    if i>2
                        loadList(i-1)=(energyList(i)-energyList(i-2))/(2*deltaDistance);
                    elseif i == 2
                        if energyList(1)> 1e-10
                            loadList(1)=(energyList(2)-energyList(1))/(deltaDistance);
                        else
                            loadList(1)=0;
                        end
                    end
                    figure(energyPlot);
                    plot(plot1,energyDistances(1:i),energyList(1:i),'Color',Colors.Text.getColor());
                    if i>3
                        plot(plot2,energyDistances(1:i-1),loadList(1:i-1),'Color',Colors.Constrained.getColor());
                    end
                    %update the progress bar
                    drawnow;
                    %save
                    obj.run(obj.runNumber).state(end+1,:)=newState;
                    obj.run(obj.runNumber).power(end+1)=100;
                end
            end
            toc
            for j=1:length(obj.static.kinematic.allBeams)
                obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(initialState,initialState);
            end
            obj.static.nodes=originalNodes;
            obj.static.kinematic.nodes=originalNodes;
            obj.run(obj.runNumber).load=loadList;
            obj.run(obj.runNumber).energy=energyList;
            obj.runNumber=obj.runNumber+1;
        end
        
        
        function [f,g]=mainFunctionfunction(obj,x,input,currentWorkspace)
            %main function for optimization
            [outputMatrix,indexList,~] = obj.static.kinematic.findInputMatrix(x,input );
            g=zeros(max(indexList),1);
            %
            %
            %beams
            %update beams
            for i=1:length(obj.static.kinematic.allBeams)
                obj.static.kinematic.allBeams{i}=obj.static.kinematic.allBeams{i}.updateBeam(outputMatrix,indexList);
            end
            energy=0;
            for i=1:length(obj.static.kinematic.allBeams)
                %energy stored in beams
                g=g+obj.static.kinematic.allBeams{i}.getGradient(indexList,currentWorkspace);
                energy=energy+obj.static.kinematic.allBeams{i}.getEnergy(currentWorkspace);
            end
            %torsion springs
            for i=1:length(obj.static.torsionSprings)
                if logical(obj.static.torsionSprings(i).active)
                    magnitude=obj.static.torsionSprings(i).stiffness*currentWorkspace.torsionSpringFactor();
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
            f=energy/(currentWorkspace.lengthFactor()*currentWorkspace.forceFactor());
            g=g/(currentWorkspace.lengthFactor()*currentWorkspace.forceFactor());
        end
        
        
        function [ c,ceq,gradc,gradceq  ] = constraintFunction(obj,x,input)
            %constraint function for optimization
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
        
        function Hout = hessianfcn(obj,x,lambda,input,currentWorkspace)
            %hessian matrix during energy minimization
            %             x(end)=pi/15;
            %             x(end-1)=pi/15;
            %             x(end-2)=pi/15;
            %             x(end-3)=pi/15;
            Hout=sparse(length(x),length(x));
            [inputMatrix,indexList,~] = obj.static.kinematic.findInputMatrix(x,input );
            %
            %update beams
            for i=1:length(obj.static.kinematic.allBeams)
                obj.static.kinematic.allBeams{i}=obj.static.kinematic.allBeams{i}.updateBeam(inputMatrix,indexList);
            end
            for i=1:length(obj.static.kinematic.allBeams)
                %energy stored in beams
                Hout=Hout+obj.static.kinematic.allBeams{i}.getHessian(indexList,currentWorkspace);
            end
            %torsion springs
            for i=1:length(obj.static.torsionSprings)
                if logical(obj.static.torsionSprings(i).active)
                    magnitude=obj.static.torsionSprings(i).stiffness*currentWorkspace.torsionSpringFactor();
                    node=obj.static.torsionSprings(i).node;
                    link1=obj.static.torsionSprings(i).link1;
                    link2=obj.static.torsionSprings(i).link2;
                    %check link 1
                    if link1 >0
                        if node == obj.static.kinematic.allBeams{link1}.nodes(1,1)
                            [~,index1]=obj.static.kinematic.allBeams{link1}.getAngle(0.0);
                        elseif node == obj.static.kinematic.allBeams{link1}.nodes(1,2)
                            [~,index1]=obj.static.kinematic.allBeams{link1}.getAngle(100.0);
                        end
                    end
                    %check link 2
                    if link2 >0
                        if node == obj.static.kinematic.allBeams{link2}.nodes(1,1)
                            [~,index2]=obj.static.kinematic.allBeams{link2}.getAngle(0.0);
                        elseif node == obj.static.kinematic.allBeams{link2}.nodes(1,2)
                            [~,index2]=obj.static.kinematic.allBeams{link2}.getAngle(100.0);
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
            
            
            Hout=Hout/(currentWorkspace.lengthFactor()*currentWorkspace.forceFactor());
            [ ~,~,~,hessian ] = obj.static.kinematic.solveStaticEquations(inputMatrix,obj.static.kinematic.equations,indexList);
            for i=1:length(lambda.eqnonlin)
                Hout=Hout+lambda.eqnonlin(i)*hessian{i};
            end
        end
        
        
        function stablePoints=bistableAnalysisLink(obj,driverLink,mode,parent)
            %find the bistable positions
            initialDistance=obj.static.kinematic.allBeams{driverLink}.theta;
            targetDistance=initialDistance+2*pi*mode;
            %angle conditions
            numberOfDistance=100;
            targetDistances=linspace(initialDistance,targetDistance,numberOfDistance);
            deltaDistance=targetDistances(2)-targetDistances(1);
            %initial
            input.index=driverLink;
            input.dof=1;
            loadList=zeros(1,numberOfDistance-1);
            energyList=zeros(1,numberOfDistance);
            energyDistances=zeros(1,numberOfDistance);
            energyDistances(1)=initialDistance;
            %main loop
            stableIndex=1;
            success=1;
            stablePoints=struct('distance',[],'energy',[],'type',[],'maxLoad',[],'position',[]);
            for i=1:numberOfDistance
                %input value
                input.value=targetDistances(i);
                fMincon=@(x)obj.mainFunctionfunction(x,input,parent.getWorkspace());
                gMinCon=@(x)obj.constraintFunction( x,input);
                opt=optimoptions(@fmincon,'Display','none','Algorithm','interior-point','MaxFunEvals',5e3,'MaxIter',5e3',...
                    'ConstraintTolerance',1e-8,'FunctionTolerance',1e-10,'OptimalityTolerance',1e-10,....
                    'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true...
                    ,'HessianFcn',@(x,lambda)obj.hessianfcn(x,lambda,input,parent.getWorkspace())...
                    ,'FiniteDifferenceType','central','CheckGradients',false);
                %initial guess
                [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),input ) ;
                [x,fval,exitflag,output,lambda,grad,hessian] = fmincon(fMincon,initialGuess,[],[],[],[],[],[],gMinCon,opt);
                if exitflag <=0
                    success=0;
                    energyList(i:end)=[];
                    loadList(i-1:end)=[];
                    break;
                end
                energy=obj.mainFunctionfunction(x,input,parent.getWorkspace());
                [newState,~,~] = obj.static.kinematic.findInputMatrix(x,input ) ;
                for j=1:length(obj.static.kinematic.allBeams)
                    obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                end
                updatedNodes=obj.static.kinematic.updateNodes(newState);
                obj.static.nodes=updatedNodes;
                obj.static.kinematic.nodes=updatedNodes;
                %calculate energy
                energyList(i)=energy(1);
                energyDistances(i)=targetDistances(i);
                if i>2
                    loadList(i-1)=(energyList(i)-energyList(i-2))/(2*deltaDistance);
                    %check if stable bistable
                    if loadList(i-1)*loadList(i-2) <0
                        %stablePoints=struct('distance',[],'energy',[],'type',[],'maxLoad',[],'position',[]);
                        stablePoints(stableIndex).distance = interp1([loadList(i-1) loadList(i-2)],[energyDistances(i-1) energyDistances(i-2)],0)*180/pi;
                        stablePoints(stableIndex).energy=interp1([energyDistances(i-1) energyDistances(i-2)],[energyList(i-1) energyList(i-2)],stablePoints(stableIndex).distance);
                        if stablePoints(stableIndex).energy <=1e-5
                            stablePoints(stableIndex).type=1;
                        else
                            stablePoints(stableIndex).type=2;
                        end
                        if stableIndex== 1
                            stablePoints(stableIndex).maxLoad=max(abs(loadList(1:i-1)));
                        else
                            stablePoints(stableIndex).maxLoad2=max(abs(loadList(stablePoints(stableIndex-1).position:i-1)));
                        end
                        stablePoints(stableIndex).position=i-1;
                        stableIndex=stableIndex+1;
                    end
                elseif i == 2
                    if energyList(1)> 1e-10
                        loadList(1)=(energyList(2)-energyList(1))/(deltaDistance);
                    else
                        loadList(1)=0;
                    end
                end
                if stableIndex > 2
                    %if bi stable position is found stop
                    stablePoints(1).loadList=loadList(1:i-1);
                    stablePoints(1).energyList=energyList(1:i-1);
                    stablePoints(1).input=targetDistances(1:i-1)*180/pi;
                    break;
                end
            end
        end
        
        function stablePoints=bistableAnalysisSlider(obj,sliderLink,mode,parent)
            %find the bistable positions
            initialDistance=0;
            targetDistance=5*obj.static.kinematic.allBeams{sliderLink}.length*mode;
            %angle conditions
            numberOfDistance=100;
            targetDistances=linspace(initialDistance,targetDistance,numberOfDistance);
            deltaDistance=targetDistances(2)-targetDistances(1);
            %initial
            input.index=sliderLink;
            input.dof=1;
            loadList=zeros(1,numberOfDistance-1);
            energyList=zeros(1,numberOfDistance);
            energyDistances=zeros(1,numberOfDistance);
            %main loop
            stableIndex=1;
            %original nodes
            originalNodes=obj.static.nodes;
            stablePoints=struct('distance',[],'energy',[],'type',[],'maxLoad',[],'position',[]);
            for i=1:numberOfDistance
                if ~logical(parent.getStop())
                    %input value
                    %node type
                    theta=obj.static.kinematic.allBeams{sliderLink}.sliderAngle;
                    node1=originalNodes(obj.static.kinematic.allBeams{sliderLink}.nodes(1,1)).getNode();
                    node2i=originalNodes(obj.static.kinematic.allBeams{sliderLink}.nodes(1,2)).getNode();
                    node2=Point(node2i.x+targetDistances(i)*cos(theta),node2i.y+targetDistances(i)*sin(theta));
                    newLength=sqrt((node2.y-node1.y)^2+(node2.x-node1.x)^2);
                    angle=atan2((node2.y-node1.y),(node2.x-node1.x));
                    input.value=(newLength*cos(angle))*cos(theta)+(newLength*sin(angle))*sin(theta);
                    fMincon=@(x)obj.mainFunctionfunction(x,input,parent.getWorkspace());
                    gMinCon=@(x)obj.constraintFunction( x,input);
                    opt=optimoptions(@fmincon,'Display','none','Algorithm','interior-point','MaxFunEvals',5e3,'MaxIter',5e3',...
                        'ConstraintTolerance',1e-8,'FunctionTolerance',1e-10,'OptimalityTolerance',1e-10,....
                        'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true...
                        ,'HessianFcn',@(x,lambda)obj.hessianfcn(x,lambda,input,parent.getWorkspace())...
                        ,'FiniteDifferenceType','central','CheckGradients',false);
                    %initial guess
                    [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),input ) ;
                    [x,fval,exitflag,output,lambda,grad,hessian] = fmincon(fMincon,initialGuess,[],[],[],[],[],[],gMinCon,opt);
                    if exitflag <=0
                        success=0;
                        energyList(i:end)=[];
                        loadList(i-1:end)=[];
                        break;
                    end
                    energy=obj.mainFunctionfunction(x,input,parent.getWorkspace());
                    [newState,~,~] = obj.static.kinematic.findInputMatrix(x,input ) ;
                    for j=1:length(obj.static.kinematic.allBeams)
                        obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                    end
                    updatedNodes=obj.static.kinematic.updateNodes(newState);
                    obj.static.nodes=updatedNodes;
                    obj.static.kinematic.nodes=updatedNodes;
                    %calculate energy
                    energyList(i)=energy(1);
                    energyDistances(i)=targetDistances(i);
                    if i>2
                        loadList(i-1)=(energyList(i)-energyList(i-2))/(2*deltaDistance);
                        %check if stable bistable
                        if loadList(i-1)*loadList(i-2) <0
                            %stablePoints=struct('distance',[],'energy',[],'type',[],'maxLoad',[],'position',[]);
                            stablePoints(stableIndex).distance = interp1([loadList(i-1) loadList(i-2)],[energyDistances(i-1) energyDistances(i-2)],0);
                            stablePoints(stableIndex).energy=interp1([energyDistances(i-1) energyDistances(i-2)],[energyList(i-1) energyList(i-2)],stablePoints(stableIndex).distance);
                            if stablePoints(stableIndex).energy <=1e-5
                                stablePoints(stableIndex).type=1;
                            else
                                stablePoints(stableIndex).type=2;
                            end
                            if stableIndex== 1
                                stablePoints(stableIndex).maxLoad=max(abs(loadList(1:i-1)));
                            else
                                stablePoints(stableIndex).maxLoad2=max(abs(loadList(stablePoints(stableIndex-1).position:i-1)));
                            end
                            stablePoints(stableIndex).position=i-1;
                            stableIndex=stableIndex+1;
                        end
                    elseif i == 2
                        if energyList(1)> 1e-10
                            loadList(1)=(energyList(2)-energyList(1))/(deltaDistance);
                        else
                            loadList(1)=0;
                        end
                    end
                    if stableIndex > 2
                        %if bi stable position is found stop
                        stablePoints(1).loadList=loadList(1:i-1);
                        stablePoints(1).energyList=energyList(1:i-1);
                        stablePoints(1).input=targetDistances(1:i-1)*180/pi;
                        break;
                    end
                end
            end
        end
        
        
    end
    
end

