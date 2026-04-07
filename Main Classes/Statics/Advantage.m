classdef Advantage
    %class for mechanical advantage    
    properties
        static=Statics.empty;
        inputLoad=[0 0];
        outputLoad=[0 0];
        busy=0;
        follower=0;
        originalNodes;
        run=struct('state',[],'workspace',WorkSpace.empty,'power',[],'forces',Force.empty,'moments',Moment.empty,'torsionSprings',TorsionSpring.empty);
        runNumber=1;
        initialState;
    end
    
    methods
        function obj=Advantage(links,nodes,forces,moments,workspace)
            %constructor
            %make forces inactive
            for i=1:length(forces)
                forces(i).active=0;
            end
            %make moments inactive
            for i=1:length(moments)
                moments(i).active=0;
            end
            obj.static=Statics(links,nodes,forces,moments,TorsionSpring.empty,workspace);
            obj=obj.addTorsionSprings();
            %initial angles
            for i=1:length(obj.static.torsionSprings)
                obj.static.torsionSprings(i)=obj.static.torsionSprings(i).setInitialAngle(obj.static.kinematic.allBeams);
            end
             %initial state
            [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] ); 
            [obj.initialState,~,~] = obj.static.kinematic.findInputMatrix(initialGuess,[] ); 
            obj.originalNodes=obj.static.nodes;
            obj.run(obj.runNumber)=struct('state',obj.initialState,'workspace',workspace,'power',0,'forces',obj.static.forces,'moments',obj.static.moments,'torsionSprings',obj.static.torsionSprings);
        end
        
        
        function obj=addTorsionSprings(obj)
            %add torsion springs before the analysis
            index=1;
            for i=1:length(obj.static.kinematic.allBeams)
                if obj.static.kinematic.allBeams{i}.joints(1,1) == Joint.GroundPin
                    obj.static.torsionSprings(index)=TorsionSpring(index,i,0,obj.static.kinematic.allBeams{i}.nodes(1,1),100);
                    obj.static.torsionSprings(index).drawSpring=0;
                    index=index+1;
                end
            end
            
        end
        
        function obj=setInputForce(obj)
            %set the input force
            if obj.inputLoad(1,1) == 1
                %force
                obj.static.forces(obj.inputLoad(1,2)).active=1;
                if logical(obj.static.forces(obj.inputLoad(1,2)).follower)
                    %follower
                    obj.static.forces(obj.inputLoad(1,2)).magnitude =1;
                    obj.follower=1;
                else
                    if logical(obj.static.forces(obj.inputLoad(1,2)).xValue) && logical(obj.static.forces(obj.inputLoad(1,2)).yValue)
                        obj.static.forces(obj.inputLoad(1,2)).xValue =1/sqrt(2);
                        obj.static.forces(obj.inputLoad(1,2)).yValue =1/sqrt(2);
                    elseif logical(obj.static.forces(obj.inputLoad(1,2)).xValue) && ~logical(obj.static.forces(obj.inputLoad(1,2)).yValue)
                        obj.static.forces(obj.inputLoad(1,2)).xValue =1;
                    elseif ~logical(obj.static.forces(obj.inputLoad(1,2)).xValue) && logical(obj.static.forces(obj.inputLoad(1,2)).yValue)
                        obj.static.forces(obj.inputLoad(1,2)).yValue =1;
                    end
                end
            else
                %moment
                obj.static.moments(obj.inputLoad(1,2)).magnitude=1;
                obj.static.moments(obj.inputLoad(1,2)).active=1;
            end
        end
        
        function obj=restore(obj,parent)
            %restore the nodes
            obj.static.nodes=obj.originalNodes;
            for i=1:length(obj.static.forces)
                obj.static.forces(i).active=0;
            end
            %make moments inactive
            for i=1:length(obj.static.moments)
                obj.static.moments(i).active=0;
            end
            obj.busy=0;
            obj.inputLoad=[0 0];
            obj.outputLoad=[0 0];
        end
        
        function obj=analyze(obj,parent)
            %analysis function for mechanical advantage
            popHandles=parent.getPopHandles();
            parent.setStop(0);
            obj=obj.setInputForce();
            %initial guess
            if obj.outputLoad(1,1) == 1
                %force
                if logical(obj.static.forces(obj.outputLoad(1,2)).follower)
                    %follower
                    initialGuess=1;
                    obj.follower=1;
                else
                    if logical(obj.static.forces(obj.outputLoad(1,2)).xValue) && logical(obj.static.forces(obj.outputLoad(1,2)).yValue)
                        initialGuess=[1/sqrt(2) 1/sqrt(2)];
                    elseif logical(obj.static.forces(obj.outputLoad(1,2)).xValue) && ~logical(obj.static.forces(obj.outputLoad(1,2)).yValue)
                        initialGuess=1;
                    elseif ~logical(obj.static.forces(obj.outputLoad(1,2)).xValue) && logical(obj.static.forces(obj.outputLoad(1,2)).yValue)
                        initialGuess=1;
                    end
                end
            else
                %moment
                initialGuess=1;
            end
            loadSimulation=LoadAnalysis([],[],[],[],[],parent.getWorkspace(),obj.static); 
            %initial config
            fMincon=@(x)obj.mainFunction( x,loadSimulation);
            outputFnc=@(x, optimValues, state)obj.outputFunction(x, optimValues, state,parent);
            opt=optimset('Display','iter-detailed','MaxFunEvals',1e4,'MaxIter',1e4,'OutputFcn',outputFnc,'TolFun',1e-6,'TolX',1e-6);
            %set the string
            popHandles.advantage.String='Mechanical Advantage:...';
            drawnow;
            [x,fval,exitflag,~] =  fminsearch(fMincon,initialGuess,opt);
            if exitflag > 0 && abs(fval) < 1e-4
                %success
                if obj.outputLoad(1,1) == 1
                    %force
                    obj.static.forces(obj.outputLoad(1,2)).active =1;
                    if logical(obj.static.forces(obj.outputLoad(1,2)).follower)
                        %follower
                        obj.static.forces(obj.outputLoad(1,2)).magnitude =x;
                        popHandles.advantage.String=['Mechanical Advantage:' num2str(abs(x))];
                    else
                        if logical(obj.static.forces(obj.outputLoad(1,2)).xValue) && logical(obj.static.forces(obj.outputLoad(1,2)).yValue)
                            obj.static.forces(obj.outputLoad(1,2)).xValue =x(1);
                            obj.static.forces(obj.outputLoad(1,2)).yValue =x(2);
                            popHandles.advantage.String=['Mechanical Advantage:' num2str(num2str(sqrt(x(1)^2+x(2)^2)))];
                        elseif logical(obj.static.forces(obj.outputLoad(1,2)).xValue) && ~logical(obj.static.forces(obj.outputLoad(1,2)).yValue)
                            obj.static.forces(obj.outputLoad(1,2)).xValue =x;
                            popHandles.advantage.String=['Mechanical Advantage:' num2str(abs(x))];
                        elseif ~logical(obj.static.forces(obj.outputLoad(1,2)).xValue) && logical(obj.static.forces(obj.outputLoad(1,2)).yValue)
                            obj.static.forces(obj.outputLoad(1,2)).yValue =x;
                            popHandles.advantage.String=['Mechanical Advantage:' num2str(abs(x))];
                        end
                    end
                else
                    %moment
                    obj.static.moments(obj.outputLoad(1,2)).magnitude=x;
                    popHandles.advantage.String=['Mechanical Advantage:' num2str(abs(x))];
                    obj.static.moments(obj.outputLoad(1,2)).active=1;
                end
            else
                popHandles.advantage.String='Mechanical Advantage:Failed';
            end
            drawnow;
            %save
            obj.run(obj.runNumber).forces=obj.static.forces;
            obj.run(obj.runNumber).power(2)=100;
            obj.run(obj.runNumber).moments=obj.static.moments;
            obj.run(obj.runNumber).state(2,:)=obj.initialState;
            obj.runNumber=obj.runNumber+1;
            obj.run(obj.runNumber)=struct('state',obj.initialState,'workspace',parent.getWorkspace(),'power',0,'forces',obj.static.forces,'moments',obj.static.moments,'torsionSprings',obj.static.torsionSprings);
        end
        
        function f=mainFunction(obj,x,loadSimulation)
            %main function
            %set the output force
            if obj.outputLoad(1,1) == 1
                %force
                loadSimulation.static.forces(obj.outputLoad(1,2)).active =1;
                if logical(obj.static.forces(obj.outputLoad(1,2)).follower)
                    %follower
                    loadSimulation.static.forces(obj.outputLoad(1,2)).magnitude =x;
                else
                    if logical(obj.static.forces(obj.outputLoad(1,2)).xValue) && logical(obj.static.forces(obj.outputLoad(1,2)).yValue)
                        loadSimulation.static.forces(obj.outputLoad(1,2)).xValue =x(1);
                        loadSimulation.static.forces(obj.outputLoad(1,2)).yValue =x(2);
                    elseif logical(obj.static.forces(obj.outputLoad(1,2)).xValue) && ~logical(obj.static.forces(obj.outputLoad(1,2)).yValue)
                        loadSimulation.static.forces(obj.outputLoad(1,2)).xValue =x;
                    elseif ~logical(obj.static.forces(obj.outputLoad(1,2)).xValue) && logical(obj.static.forces(obj.outputLoad(1,2)).yValue)
                        loadSimulation.static.forces(obj.outputLoad(1,2)).yValue =x;
                    end
                end
            else
                %moment
                loadSimulation.static.moments(obj.outputLoad(1,2)).magnitude=x;
                loadSimulation.static.moments(obj.outputLoad(1,2)).active=1;
            end
            %number of increments
            if logical(obj.follower)
                increments=10;
            else
                increments=1;
            end
            power=linspace(0,100,increments);
            %the main analysis function\
            %find the initial distances for non followers
            [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] ); 
            [outputMatrix,~,~] = obj.static.kinematic.findInputMatrix(initialGuess,[] ); 
            for j=1:length(loadSimulation.static.forces)
                if ~logical(loadSimulation.static.forces(j).follower)
                    [ loadSimulation.static.forces(j).initialDistanceX, loadSimulation.static.forces(j).initialDistanceY ,~,~] = loadSimulation.static.kinematic.solveStaticEquations(outputMatrix,loadSimulation.static.forces(j).kinematicEquations,[]);
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
                    for j=1:length(obj.originalNodes)
                        oldNode=obj.originalNodes(j).getNode() ;
                        newNode=updatedNodes(j).getNode() ;
                        f=f+sqrt((oldNode.x-newNode.x)^2+(oldNode.y-newNode.y)^2);
                    end
                end
                
            end
        end
        
        %function for output every iteration
        function stop = outputFunction(obj,x, optimValues, state,parent)
            progressHandles=parent.getProgressBar();
            if logical(parent.getStop())
                stop=true;
                return;
            else
                %update the progress bar
                %the strings
                try
                    patch([0,abs(1-optimValues.fval)/1,abs(1-optimValues.fval)/1,0],[0,0,1,1],Colors.StatusComplete.getColor(),'Parent',progressHandles.progress);
                    figure(progressHandles.progressGUI);
                catch err
                    display(err);
                    display('Error displaying progress bar');
                end
                drawnow;
            end
            if  isempty(optimValues.fval)
                stop=false;
                return;
            end
            stop=false;
            progressHandles.progressText.String=['Increment:', num2str(optimValues.iteration)];
            drawnow;
        end
        
        function obj=drawAll(obj,mainFig,limit,mode,parent)
            %draw all links
            obj.static.kinematic=obj.static.kinematic.drawAll(mainFig,limit,mode,parent);
            %plot forces
            for i=1:length(obj.static.forces)
                obj.static.forces(i)=obj.static.forces(i).drawStatic(parent,obj.static.nodes,obj.static.kinematic.allBeams,100);
            end
            %plot moments
            for i=1:length(obj.static.moments)
                obj.static.moments(i)=obj.static.moments(i).drawStatic(parent,obj.static.nodes,obj.static.kinematic.allBeams,100);
            end
            %plot torsion springs
            for i=1:length(obj.static.torsionSprings)
                obj.static.torsionSprings(i)=obj.static.torsionSprings(i).drawStatic(parent,obj.static.nodes,obj.static.kinematic.allBeams);
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
            %delete nodes
            for i=1:length(obj.static.nodes)
                obj.static.nodes(i)=obj.static.nodes(i).deleteNodeDrawing;
            end
        end
    end
    
end

