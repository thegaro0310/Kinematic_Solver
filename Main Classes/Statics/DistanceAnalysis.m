classdef DistanceAnalysis
    properties
        static=Statics.empty;
        distance=struct('type',[],'member',0,'target',[],'line',[],'selected',0);
        run=struct('state',[],'workspace',WorkSpace.empty,'magnitudeList',[],'forces',Force.empty,'moments',Moment.empty,'torsionSprings',TorsionSpring.empty,'distance',[],'inputLoad',[]);
        runNumber=1;
        selectedLink=0;
        selectedNode=0;
        inputLoad=[0 0];
        busy=0;
        originalNodes=Node.empty;
        originalForces=Force.empty;
        originalMoments=Force.empty;
        initialState=[];
        state=[];
        newValues=struct('newValue',[],'type',[]);
    end
    
    methods
        % Constructor
        function obj=DistanceAnalysis(links,nodes,forces,moments,torsionSprings,currentWorkspace)
            %constructor
            %make forces inactive
            for i=1:length(forces)
                forces(i).active=0;
            end
            %make moments inactive
            for i=1:length(moments)
                moments(i).active=0;
            end
            obj.static=Statics(links,nodes,forces,moments,torsionSprings,currentWorkspace);
            obj.originalNodes=obj.static.nodes;
            obj.originalForces=obj.static.forces;
            obj.originalMoments=obj.static.moments;
            %initial state
            [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] ); 
            [obj.initialState,~,~] = obj.static.kinematic.findInputMatrix(initialGuess,[] ); 
        end
       
        % Use for non GUI
        function obj=drawDistance(obj,mainFig,limit)
            %draw the distance
            target=obj.distance.target(1,2);
            if obj.distance.type == 1
                %node type
                node=obj.static.nodes(obj.distance.member).getNode();
                if obj.distance.target(1,1) == 1
                    %x motion
                    obj.distance.line(1)=plot(mainFig,target(1)+[node.x node.x],node.y+[limit*0.2 -limit*0.2],'LineStyle','--','Color',Colors.NormalLink.getColor());
                else
                    %y motion
                    obj.distance.line(1)=plot(mainFig,node.x+[limit*0.2 -limit*0.2],target(1)+[node.y node.y],'LineStyle','--','Color',Colors.NormalLink.getColor());
                end
            else
                %link type
                link=obj.static.kinematic.allBeams{obj.distance.member};
                linkLength=max(link.length0,limit/3);
                linkAngle=link.theta0;
                node=obj.static.nodes(link.nodes(1,1)).getNode();
                if obj.distance.target(1,1) == 1
                    %CW motion
                    newAngle=linkAngle-target(1)*pi/180;
                else
                    %CCW motion
                    newAngle=linkAngle+target(1)*pi/180;
                end
                obj.distance.line(1)=plot(mainFig,[node.x node.x+linkLength*cos(newAngle) ],[node.y node.y+linkLength*sin(newAngle) ],'LineStyle','--','Color',Colors.NormalLink.getColor());
            end
        end
        
        % Use for non GUI
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
        
        % Use for non GUI
        function obj=selectLoad(obj,member,type)
            %select the input force or moment
            if strcmp(type,'force')
                obj.inputLoad=[1 member];
            else
                obj.inputLoad=[2 member];
            end
        end
        
        % Use for non GUI
        function obj=addInput(obj,member,target,type,links)
            %add range input
            switch type
                case 'x'
                    obj.distance=struct('type',1,'member',member,'target',[1 target],'line',[],'selected',0);
                case 'y'
                    obj.distance=struct('type',1,'member',member,'target',[2 target],'line',[],'selected',0);
                case 'angle'
                    linkID=Helper.findLink(member,links,obj.static.kinematic.allBeams,3);
                    if target < 0
                        obj.distance=struct('type',2,'member',linkID,'target',[1 abs(target)],'line',[],'selected',0);
                    else
                        obj.distance=struct('type',2,'member',linkID,'target',[2 abs(target)],'line',[],'selected',0);
                    end
            end
        end
        
        % Use for GUI
        % function obj=analyze(obj,parent)
        %     %main function
        %     parent.setStop(0);
        %     %initial guess for the optimization
        %     if obj.inputLoad(1,1) == 1
        %         %force
        %         if logical(obj.static.forces(obj.inputLoad(1,2)).follower)
        %             %follower
        %             initialGuess(1)=0;
        %             type=1;
        %         else
        %             if logical(obj.static.forces(obj.inputLoad(1,2)).xValue) && logical(obj.static.forces(obj.inputLoad(1,2)).yValue)
        %                 initialGuess(1)=0;
        %                 initialGuess(2)=0;
        %                 type=3;
        %             elseif logical(obj.static.forces(obj.inputLoad(1,2)).xValue) && ~logical(obj.static.forces(obj.inputLoad(1,2)).yValue)
        %                 initialGuess(1)=0;
        %                 type=4;
        %             elseif ~logical(obj.static.forces(obj.inputLoad(1,2)).xValue) && logical(obj.static.forces(obj.inputLoad(1,2)).yValue)
        %                 initialGuess(1)=0;
        %                 type=5;
        %             end
        %         end
        %     else
        %         %moment
        %         initialGuess(1)=0;
        %         type=2;
        %     end
        %     follower=0;
        %     for i=1:length(obj.static.forces)
        %         if logical(obj.static.forces(i).active) && logical(obj.static.forces(i).follower)
        %             follower=1;
        %         end
        %     end
        %     loadSimulation=LoadAnalysis([],[],[],[],[],parent.getWorkspace(),obj.static);
        %     if obj.inputLoad(1,1) == 1
        %         loadSimulation.static.forces(obj.inputLoad(1,2)).active=1;
        %     else
        %         loadSimulation.static.moments(obj.inputLoad(1,2)).active=1;
        %     end
        %     originalLinks=obj.static.kinematic.allBeams;       
        %     %initial config
        %     fMincon=@(x)obj.findDistanceAnalysis( x,obj.originalNodes,originalLinks,follower,loadSimulation,type,parent);
        %     if obj.distance.type == 1
        %         initial=obj.distance.target(1,2)^2;
        %     else
        %         initial=(obj.distance.target(1,2)*pi/180)^2;
        %     end
        %     outputFnc=@(x, optimValues, state)obj.outputFunction(x, optimValues, state,parent,initial);
        %     opt=optimset('Display','none','MaxFunEvals',1e4,'MaxIter',1e4,'OutputFcn',outputFnc);
        %     [x,fval,exitflag,output] =  fminsearch(fMincon,initialGuess,opt);
        %     if fval <= 1e-5
        %         uiwait(msgbox('Analysis successful. Load magnitude saved.','Success','modal'));
        %     else
        %         uiwait(msgbox('Analysis failed but last load magnitude is saved.','Error','error','modal'));
        %     end
        %     %save data
        %     obj=parent.getDistanceAnalysis();
        %     obj.run(obj.runNumber).magnitudeList=vertcat([0 0],obj.run(obj.runNumber).magnitudeList);
        %     obj.run(obj.runNumber).distance=obj.distance;
        %     obj.run(obj.runNumber).inputLoad=obj.inputLoad;
        %     obj.newValues=struct('newValue',x,'type',type);
        %     %return
        %     obj.runNumber=obj.runNumber+1;
        %     obj.run(obj.runNumber)=struct('state',[],'workspace',WorkSpace.empty,'magnitudeList',[],'forces',Force.empty,'moments',Moment.empty,'torsionSprings',TorsionSpring.empty,'distance',[],'inputLoad',[]);
        % end
        
        % Use for non GUI
        function obj=simulationNoGUI(obj,workspace)
            %main function
            %initial guess for the optimization
            if obj.inputLoad(1,1) == 1
                %force
                if logical(obj.static.forces(obj.inputLoad(1,2)).follower)
                    %follower
                    initialGuess(1)=0;
                    type=1;
                else
                    if logical(obj.static.forces(obj.inputLoad(1,2)).xValue) && logical(obj.static.forces(obj.inputLoad(1,2)).yValue)
                        initialGuess(1)=0;
                        initialGuess(2)=0;
                        type=3;
                    elseif logical(obj.static.forces(obj.inputLoad(1,2)).xValue) && ~logical(obj.static.forces(obj.inputLoad(1,2)).yValue)
                        initialGuess(1)=0;
                        type=4;
                    elseif ~logical(obj.static.forces(obj.inputLoad(1,2)).xValue) && logical(obj.static.forces(obj.inputLoad(1,2)).yValue)
                        initialGuess(1)=0;
                        type=5;
                    end
                end
            else
                %moment
                initialGuess(1)=0;
                type=2;
            end
            follower=0;
            for i=1:length(obj.static.forces)
                if logical(obj.static.forces(i).active) && logical(obj.static.forces(i).follower)
                    follower=1;
                end
            end
            loadSimulation=LoadAnalysis([],[],[],[],[],workspace,obj.static);
            if obj.inputLoad(1,1) == 1
                loadSimulation.static.forces(obj.inputLoad(1,2)).active=1;
            else
                loadSimulation.static.moments(obj.inputLoad(1,2)).active=1;
            end
            originalLinks=obj.static.kinematic.allBeams;       
            %initial config
            fMincon=@(x)obj.findDistanceAnalysis( x,obj.originalNodes,originalLinks,follower,loadSimulation,type,[]);
            opt=optimset('Display','iter-detailed','MaxFunEvals',1e4,'MaxIter',1e4);
            [x,fval,exitflag,output] =  fminsearch(fMincon,initialGuess,opt);
            %save data
            obj.run(obj.runNumber).magnitudeList=vertcat([0 0],obj.run(obj.runNumber).magnitudeList);
            obj.run(obj.runNumber).distance=obj.distance;
            obj.run(obj.runNumber).inputLoad=obj.inputLoad;
            obj.newValues=struct('newValue',x,'type',type);
            %return
            obj.runNumber=obj.runNumber+1;
            obj.run(obj.runNumber)=struct('state',[],'workspace',WorkSpace.empty,'magnitudeList',[],'forces',Force.empty,'moments',Moment.empty,'torsionSprings',TorsionSpring.empty,'distance',[],'inputLoad',[]);
        end
        
        % Use for non GUI
        % Main function for minimization in distance analysis
        function f=findDistanceAnalysis(obj,x,originalNodes,originalLinks,follower,loadSimulation,type,parent)
            if logical(follower)
                increments=10;
            else
                increments=1;
            end
            power=linspace(0,100,increments);
            %change force magnitudes
            switch(type)
                case 1
                    loadSimulation.static.forces(obj.inputLoad(1,2)).magnitude=x;
                case 2
                    loadSimulation.static.moments(obj.inputLoad(1,2)).magnitude=x;
                case 3
                    loadSimulation.static.forces(obj.inputLoad(1,2)).xValue=x(1);
                    loadSimulation.static.forces(obj.inputLoad(1,2)).yValue=x(2);
                case 4
                    loadSimulation.static.forces(obj.inputLoad(1,2)).xValue=x(1);
                case 5
                    loadSimulation.static.forces(obj.inputLoad(1,2)).yValue=x(1);
            end
            
            %the main analysis function
            %find the initial distances for non followers
            [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] ); 
            [outputMatrix,~,~] = obj.static.kinematic.findInputMatrix(initialGuess,[] ); 
            for j=1:length(loadSimulation.static.forces)
                if ~logical(loadSimulation.static.forces(j).follower)
                    [ loadSimulation.static.forces(j).initialDistanceX, loadSimulation.static.forces(j).initialDistanceY] = loadSimulation.static.kinematic.solveStaticEquations(outputMatrix,loadSimulation.static.forces(j).kinematicEquations);
                end
            end
            for i=1:increments
                [allBeams,forces,moments]=loadSimulation.getInputs();
                [x,exitflag,~]= loadSimulation.fMinConMain( [],power(i),allBeams,forces,moments); 
                if exitflag >0
                    [newState,~,~] = loadSimulation.static.kinematic.findInputMatrix(x,[] );
                    for j=1:length(loadSimulation.static.kinematic.allBeams)
                        loadSimulation.static.kinematic.allBeams{j}=loadSimulation.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                    end
                    updatedNodes=loadSimulation.static.kinematic.updateNodes(newState);
                end
                if exitflag<=0
                    f=1000;
                else
                    f=0;
                    if ~isempty(parent)
                        parent.updateDistanceNodeForce(updatedNodes,loadSimulation.static.kinematic.allBeams,loadSimulation.static.forces,loadSimulation.static.moments,newState);
                    end
                    %distance=struct('type',[],'member',0,'target',[],'line',[],'selected',0);
                    if obj.distance.type == 1
                        %node distance analysis
                        if obj.distance.target(1,1) == 1
                            %x distance
                            original=originalNodes(obj.distance.member).getNode().x;
                            current=updatedNodes(obj.distance.member).getNode().x;
                        else
                            %y distance
                            original=originalNodes(obj.distance.member).getNode().y;
                            current=updatedNodes(obj.distance.member).getNode().y;
                        end
                        value=obj.distance.target(1,2);
                        f=f+((current-original)-value)^2;
                    else
                        %link rotation
                        original=originalLinks{obj.distance.member}.theta;
                        current=loadSimulation.static.kinematic.allBeams{obj.distance.member}.theta;
                        if obj.distance.target(1,1) == 1
                            %CW Rotation
                            value=-obj.distance.target(1,2)*pi/180;
                        else
                            %CCW Rotation
                            value=obj.distance.target(1,2)*pi/180;
                        end
                        f=f+((current-original)-value)^2;
                    end
                end
            end
            
        end
        
    end
end
