classdef FlexuralStiffnessSynthesis
    %flexural synthesis class
    properties
        static=Statics.empty;
        distance=struct('type',[],'member',0,'target',[],'line',[],'selected',0);
        selectedLink=0;
        selectedNode=0;
        unknowns=[];
        originalLinks=Link.empty;
        originalRigidLinks=rigidBody.empty;
        originalNodes=Node.empty;
        firstNodes=Node.empty;
        originalSprings=TorsionSpring.empty;
        originalForces=Force.empty;
        originalMoments=Moment.empty;
        newValues=[];
    end
    
    methods
        function obj=FlexuralStiffnessSynthesis(links,nodes,forces,moments,torsionSprings,parent)
            %constructor
            %make forces inactive
            for i=1:length(forces)
                forces(i).active=1;
            end
            %make moments inactive
            for i=1:length(moments)
                moments(i).active=1;
            end
            obj.static=Statics(links,nodes,forces,moments,torsionSprings,parent.getWorkspace());
            %add compliant members to the group
            for i=1:length(obj.static.kinematic.allBeams)
                if ~isempty(obj.static.kinematic.allBeams{i}.crossSection)
                    obj.unknowns(end+1)=obj.static.kinematic.allBeams{i}.id;
                end
            end
            obj.originalLinks=links;
            obj.firstNodes=nodes;
            obj.originalNodes=obj.static.nodes;
            obj.originalSprings=torsionSprings;
            obj.originalForces=forces;
            obj.originalMoments=moments;     
        end
        
        function obj=setSelectedLink(obj,id,select)
            %set the selected link
            obj.static.kinematic.allBeams{id}.selected=select;
        end
        
        function obj=setSelectedNode(obj,id,select)
            %set the selected link
            obj.static.nodes(id)=obj.static.nodes(id).setSelected(select);
        end
        
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
        
        function obj=synthesize(obj,parent)
            %main function
            %check for followers
            follower=0;
            for i=1:length(obj.static.forces)
                if logical(obj.static.forces(i).active) && logical(obj.static.forces(i).follower)
                    follower=1;
                end
            end
            links=obj.findLinksFromRigidLink();
            %set the initial conditions
            initialGuess=zeros(1,length(links));
            for i=1:length(links)
                initialGuess(i)=obj.static.kinematic.allBeams{links(i)}.crossSection.E*obj.static.kinematic.allBeams{links(i)}.crossSection.getI();
            end
            %strings
            if obj.distance.type == 1
                initial=obj.distance.target(1,2)^2;
            else
                initial=(obj.distance.target(1,2)*pi/180)^2;
            end
            %initial config
            fMincon=@(x)obj.flexuralSynthesisMain(x,obj.originalNodes,obj.originalLinks,follower,links,parent);
            outputFnc=@(x, optimValues, state)obj.outputFunction(x, optimValues, state,parent,initial);
            opt=optimset('Display','iter-detailed','MaxFunEvals',1e4,'MaxIter',1e4,'OutputFcn',outputFnc);
            [x,fval,exitflag,output] =  fminsearch(fMincon,initialGuess,opt) 
            %save data
            obj=parent.getFlexuralStiffnessSynthesis();
            obj.newValues=x;
            %return
            %restore
            obj=obj.restore(parent);
        end
        
        
        function links=findLinksFromRigidLink(obj)
            %converts rigidlinklist id to link id
            links=zeros(1,length(obj.unknowns));
            for j=1:length(obj.unknowns)
                for i=1:length(obj.static.kinematic.allBeams)
                    if obj.static.kinematic.allBeams{i}.id == obj.unknowns(j)
                        links(j)=i;
                        break;
                    end
                end
            end
        end
        
        %function for output every iteration
        function stop = outputFunction(obj,x, optimValues, state,parent,original)
            %draw the config
            parent.plotEverything();
            progressHandles=parent.getProgressBar();
            if logical(parent.getStop())
                stop=true;
                handles=parent.getHandles();
                handles.targetDistanceStatic2.String='aborted';
                return;
            else
                %update the progress bar
                %the strings
                try
                    patch([0,(original-optimValues.fval)/original,(original-optimValues.fval)/original,0],[0,0,1,1],Colors.StatusComplete.getColor(),'Parent',progressHandles.progress);
                    figure(progressHandles.progressGUI);
                catch err
                    display(err);
                    progressHandles
                    display('Error displaying progress bar');
                end
                drawnow;
            end
            if  isempty(optimValues.fval)
                stop=false;
                return;
            elseif  abs(optimValues.fval) <1e-6
                stop=true;
                progressHandles.progressText.String=['Distance to Target:', num2str(optimValues.fval)];
                return;
            end
            stop=false;
            progressHandles.progressText.String=['Distance to Target:', num2str(optimValues.fval)];
            drawnow;
        end
        
        function f=flexuralSynthesisMain(obj,x,originalNodes,originalLinks,follower,unknowns,parent)
            %main function for flexural synthesis
            if logical(follower)
                increments=10;
            else
                increments=1;
            end
            power=linspace(0,100,increments);
            %update original links
            for i=1:length(unknowns)
                xSection=originalLinks(unknowns(i)).getCrossSection();
                xSection.width=12;
                xSection.thickness=1;
                xSection.E=abs(x(i));
                originalLinks(unknowns(i))=originalLinks(unknowns(i)).setCrossSection(xSection);
            end
            %load analysis
            loadSimulation=LoadAnalysis([],[],[],[],[],parent.getWorkspace(),Statics(originalLinks,obj.firstNodes,obj.originalForces,obj.originalMoments,obj.originalSprings,parent.getWorkspace()));
            %the main analysis function
            [~,~,initialGuess] = obj.static.kinematic.findInputMatrix(zeros(obj.static.kinematic.allBeams{end}.id+obj.static.kinematic.allBeams{end}.degreesOfFreedom+3,1),[] );
            [outputMatrix,~,~] = obj.static.kinematic.findInputMatrix(initialGuess,[] );
            for j=1:length(loadSimulation.static.forces)
                if ~logical(loadSimulation.static.forces(j).follower)
                    [ loadSimulation.static.forces(j).initialDistanceX, loadSimulation.static.forces(j).initialDistanceY ,~,~] = loadSimulation.static.kinematic.solveStaticEquations(outputMatrix,loadSimulation.static.forces(j).kinematicEquations,[]);
                end
            end
            for i=1:increments
                [x,exitflag,~]= loadSimulation.fMinConMain( [],power(i));
                if exitflag >0
                    [newState,~,~] = loadSimulation.static.kinematic.findInputMatrix(x,[] );
                    for j=1:length(loadSimulation.static.kinematic.allBeams)
                        loadSimulation.static.kinematic.allBeams{j}=loadSimulation.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                        obj.static.kinematic.allBeams{j}=obj.static.kinematic.allBeams{j}.updateBeam(newState,newState);
                    end
                    updatedNodes=loadSimulation.static.kinematic.updateNodes(newState);
                end
                if exitflag<=0
                    f=1000;
                else
                    parent.updateFlexuralNodeForce(updatedNodes,obj.static.kinematic.allBeams);
                    f=0;
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
                        original=obj.static.kinematic.allBeams{obj.distance.member}.theta;
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
        
        function obj=restore(obj,parent)
            %restore the flexural synthesis
            parent.setBusy(0);
            obj=obj.deleteAll();
            obj.static.nodes=obj.originalNodes;
            obj.static.kinematic.nodes=obj.originalNodes;
            parent.saveFlexuralSynthesis(obj.findLinksFromRigidLink(),obj.newValues);
            obj=parent.getFlexuralStiffnessSynthesis();
            obj=obj.updateDistance();
            obj=obj.drawDistance(parent.getFigure(),parent.getWorkspace.getLimit()*0.5);
            %panel
        end
        
    end
end