classdef Kinematics
    %kinematics class
    properties
        allBeams=cell(0);
        connections=Connection.empty;
        nodes=Node.empty;
        rigidPlates=[];
        adjacencyMatrix;
        independentLoops;
        loops;
        equations=struct('beamList',[]);
        uniqueConnections;
        additionalDof=struct([]);
        inputs=struct('linkID',[],'inputType',[],'target',[],'isRelative',0,'initialValue',[],'type',[]);
        stateList=cell(0);
        nodeList=[];
        originalNodes=Node.empty;
        trackedNodes=[];
        trackedNodesLine=gobjects(0);
        workspace;
        busy=0;
    end
    
    methods
        
        function obj=Kinematics(links,nodes,workspace)
            %constructor
            obj.nodes=nodes;
            obj.workspace=workspace;
            obj=obj.constructLinksNodes(links,nodes);
            obj=obj.mainPreparation();
            obj.nodeList(1,:)=obj.addAllNodes(nodes);
            %add state
            if ~isempty(obj.allBeams)
                [~,~,initialGuess] = obj.findInputMatrix(zeros(obj.allBeams{end}.id+obj.allBeams{end}.degreesOfFreedom+3,1),[] );
                [newState,~,~] = obj.findInputMatrix(initialGuess,[] );
                obj=obj.addState(newState);
            end
        end
        
        
        function obj=deleteAll(obj)
            %delete all links
            for i=1:length(obj.allBeams)
                %delete drawing
                obj.allBeams{i}=obj.allBeams{i}.deleteLinkDrawing();
            end
            for i=1:length(obj.nodes)
                %delete drawing
                obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
            end
            delete(obj.trackedNodesLine);
            obj.nodeList=[];
        end
        
        
        function obj=drawNoGUI(obj,mainFig,limit,workspace)
            %draw everything in developer mode
            for i=1:length(obj.allBeams)
                %plot links
                obj.allBeams{i}=obj.allBeams{i}.drawLinkNoGUI(mainFig,obj.nodes,limit,workspace);
            end
        end
        
        function obj=drawAll(obj,mainFig,limit,mode,parent)
            %draw all links
            for i=1:length(obj.allBeams)
                %plot links
                obj.allBeams{i}=obj.allBeams{i}.drawLink(mainFig,obj.nodes,limit,mode,parent);
            end
            delete(obj.trackedNodesLine);
            %plot nodes
            obj.trackedNodesLine=gobjects(length(obj.trackedNodes));
            try
                for i=1:length(obj.trackedNodes)
                    x=obj.nodeList(:,2*obj.trackedNodes(i)-1);
                    y=obj.nodeList(:,2*obj.trackedNodes(i));
                    obj.trackedNodesLine(i)=plot(mainFig,x,y,'LineStyle','none','Marker','.','Color',Colors.Blue.getColor());
                end
            catch err
                disp(err);
                disp('Cannot track');
            end
        end
        
        function selected=getSelected(obj)
            %get the selected link
            selected=0;
            for i=1:length(obj.allBeams)
                if obj.allBeams{i}.selected == 1
                    selected=i;
                    break;
                end
            end
        end
        
        function obj=setSelectedLink(obj,id,selected)
            %set selected link
            obj.allBeams{id}.selected=selected;
        end
        
        function groups=getGroups(~,links)
            %get the groups
            groups=[];
            for i=1:length(links)
                if logical(links(i).getGroup())
                    groups(end+1)=links(i).getGroup();
                end
            end
            groups=unique(groups);
        end
        
        function links=getLinksFromGroup(~,allLinks,group)
            %get links with the same group
            links=[];
            for i=1:length(allLinks)
                if logical(allLinks(i).getGroup()) && allLinks(i).getGroup() == group
                    links(end+1)=i;
                end
            end
        end
        
        function degree=getDegreesOfFreedom(obj)
            %get the degrees of freedom
            [~,indexList,~] = obj.findInputMatrix(zeros(obj.allBeams{end}.id+obj.allBeams{end}.degreesOfFreedom+3,1),[]);
            totalEquations=length( obj.equations)*2;
            degree=max(indexList)-totalEquations;
        end
        
        function obj=mainPreparation(obj)
            %obtain the kinematic equations
            %calculate the initial connectivity matrix
            obj = getAdjacencyMatrix(obj);
            %find loops
            obj.independentLoops=obj.independentLoops-length(obj.rigidPlates);
            obj = obj.findLoops();
            %find kinematic equations
            obj.equations=obj.kinematicEquations(obj.loops,obj.independentLoops);
            allConnections=zeros(length(obj.connections),2);
            for i=1:length(obj.connections)
                allConnections(i,:)=obj.connections(i).nodes;
            end
            obj.uniqueConnections=unique(allConnections);
        end
        
        function obj=constructLinksNodes(obj,links,nodes)
            %constructthe links and nodes
            if isempty(links) ||  isempty(nodes)
                return;
            end
            %clear
            obj.allBeams=cell(0);
            obj.connections=Connection.empty;
            obj.nodes=nodes;
            %find ground nodes
            groundNodes=zeros(1,length(nodes));
            for i=1:length(links)
                %check if a node is used as a ground node
                if links(i).getJoint(1) == Joint.GroundPin || links(i).getJoint(1)  == Joint.GroundWelded
                    groundNodes(links(i).getNode(1))=1;
                end
                if links(i).getJoint(2) == Joint.GroundPin  || links(i).getJoint(2)  == Joint.GroundWelded
                    groundNodes(links(i).getNode(2))=1;
                end
            end
            grounds=find(groundNodes == 1);
            additional=Connection.empty;
            %generate connections
            for i=1:length(links)
                %this member will be divided into two components
                if links(i).getJoint(1)==Joint.GroundSlider &&  links(i).getJoint(2)==Joint.Pin
                    %first add -3 -1 link
                    additional(end+1)=Connection([links(i).getNode(1),grounds(1)],[Joint.GroundSlider Joint.GroundWelded],links(i).getAngleSlider(),CrossSection.empty,links(i).getLinearSpringStiffness());
                    %add 2 2 link
                    obj.connections(end+1)=Connection(links(i).getNodes(),[Joint.Pin Joint.Pin],[],links(i).getCrossSection(),links(i).getLinearSpringStiffness());
                elseif links(i).getJoint(2)==Joint.GroundSlider &&  links(i).getJoint(1)==Joint.Pin
                    %first add -1 -3 link
                    additional(end+1)=Connection([grounds(1) links(i).getNode(2)],[Joint.GroundWelded Joint.GroundSlider],links(i).getAngleSlider(),CrossSection.empty,links(i).getLinearSpringStiffness());
                    %add 2 2 link
                    obj.connections(end+1)=Connection(links(i).getNodes(),[Joint.Pin Joint.Pin],[],links(i).getCrossSection(),links(i).getLinearSpringStiffness());
                elseif links(i).getJoint(1)==Joint.GroundSlider &&  links(i).getJoint(2)==Joint.Welded
                    %first add -3 -1 link
                    additional(end+1)=Connection([links(i).getNode(1),grounds(1)],[Joint.GroundSlider Joint.GroundWelded],links(i).getAngleSlider(),CrossSection.empty,links(i).getLinearSpringStiffness());
                    %add 2 1 link
                    obj.connections(end+1)=Connection(links(i).getNodes(),[Joint.Welded Joint.Welded],[],links(i).getCrossSection(),links(i).getLinearSpringStiffness());
                elseif  links(i).getJoint(2)==Joint.GroundSlider &&  links(i).getJoint(1)==Joint.Welded
                    %first add -1 -3 link
                    additional(end+1)=Connection([grounds(1) links(i).getNode(2)],[Joint.GroundWelded Joint.GroundSlider],links(i).getAngleSlider(),CrossSection.empty,links(i).getLinearSpringStiffness());
                    %add 1 2 link
                    obj.connections(end+1)=Connection(links(i).getNodes(),[Joint.Welded Joint.Welded],[],links(i).getCrossSection(),links(i).getLinearSpringStiffness());
                else
                    obj.connections(end+1)=Connection(links(i).getNodes(),links(i).getJoints(),links(i).getAngleSlider(),links(i).getCrossSection(),links(i).getLinearSpringStiffness());
                end
            end
            %add ground links
            for i=1:length(grounds)-1
                obj.connections(end+1)=Connection([grounds(i) grounds(i+1)],[Joint.GroundPin Joint.GroundPin],[],CrossSection.empty,0);
            end
            for i=1:length(additional)
                obj.connections(end+1)=additional(i);
            end
            %prepare the beams
            ID=0;
            for i=1:length(obj.connections)
                obj.allBeams{i}=obj.connections(i).convert2Beam(ID+i,obj.nodes,obj.workspace);
                ID=ID+obj.connections(i).getVariableCount()-1;
            end
            %add tied joints
            groups=zeros(length(links),1);
            for i=1:length(links)
                if logical(links(i).group)
                    groups(i)=links(i).group;
                end
            end
            if ~isempty(find(groups > 0, 1))
                obj.additionalDof=struct('id',[],'known',0,'value',[],'parent',0,'initialGuess',0);
                groupIDs=unique(groups(groups>0));
                for i=1:length(groupIDs)
                    groupLinks=find(groups == groupIDs(i));
                    obj.additionalDof(i)=struct('id',i,'known',0,'value',[],'parent',0,'initialGuess',0);
                    %to check if rigid joints form a closed plate
                    plateConnections=zeros(length(groupIDs)*2,1);
                    for j=1:length(groupLinks)
                        plateConnections(2*j-1)=obj.allBeams{groupLinks(j)}.nodes(1);
                        plateConnections(2*j)=obj.allBeams{groupLinks(j)}.nodes(2);
                        obj.allBeams{groupLinks(j)}.frontMasterID=i;
                        obj.allBeams{groupLinks(j)}.drawJoints=[0 0];
                    end
                    if length(groupLinks) == length(unique(plateConnections))
                        obj.rigidPlates(end+1)=groupLinks(1);
                    end
                end
                
            end
            %post-process joints
            [joints,nodes]=obj.allJoints();
            weldedNodes=unique(nodes(joints==Joint.Welded));
            if ~isempty(weldedNodes)
                nodesWelded=zeros(length(weldedNodes),1);
                if isempty(obj.additionalDof)
                    obj.additionalDof=struct('id',[],'known',0,'value',[],'parent',0,'initialGuess',0);
                    index=0;
                else
                    index=length(obj.additionalDof);
                end
                for i=1:length(weldedNodes)
                    obj.additionalDof(i+index)=struct('id',i+index,'known',0,'value',[],'parent',0,'initialGuess',0);
                    nodesWelded(i,1)=weldedNodes(i);
                    [row,col] = find(nodes == weldedNodes(i));
                    %tag all the beams
                    for j=1:length(row)
                        if col(j) == 1
                            obj.allBeams{row(j)}.frontMasterID=i+index;
                        else
                            obj.allBeams{row(j)}.endMasterID=i+index;
                        end
                    end
                end
                %check for welded welded beams
                weldedWeldedBeams=[];
                for i=1:length(obj.allBeams)
                    if isa(obj.allBeams{i},'KinematicsBeam') && obj.allBeams{i}.joints(1,1) == Joint.Welded && obj.allBeams{i}.joints(1,2) == Joint.Welded
                        weldedWeldedBeams(end+1,:)=obj.allBeams{i}.nodes;
                    end
                end
                for i=1:size(weldedWeldedBeams,1)
                    id1=find(nodesWelded == weldedWeldedBeams(i,1))+index;
                    id2=find(nodesWelded == weldedWeldedBeams(i,2))+index;
                    if logical(obj.additionalDof(id1).parent) && logical(obj.additionalDof(id2).parent)
                        target=obj.additionalDof(id1).parent;
                        current=obj.additionalDof(id2).parent;
                    elseif logical(obj.additionalDof(id1).parent)
                        target=obj.additionalDof(id1).parent;
                        current=id2;
                    elseif logical(obj.additionalDof(id2).parent)
                        target=obj.additionalDof(id2).parent;
                        current=id1;
                    else
                        target=id1;
                        current=id2;
                    end
                    obj.additionalDof(current).parent=target;
                    %update the beams
                    for j=1:length(obj.allBeams)
                        if obj.allBeams{j}.frontMasterID == current
                            obj.allBeams{j}.frontMasterID=target;
                        end
                        if obj.allBeams{j}.endMasterID == current
                            obj.allBeams{j}.endMasterID=target;
                        end
                    end
                end
                %delete unused additional degrees of freedom
                parents=[obj.additionalDof.parent];
                for j=1:length(obj.allBeams)
                    obj.allBeams{j}.frontMasterID=obj.allBeams{j}.frontMasterID-length(find(parents(1,1:obj.allBeams{j}.frontMasterID-1) >0));
                    obj.allBeams{j}.endMasterID=obj.allBeams{j}.endMasterID-length(find(parents(1,1:obj.allBeams{j}.endMasterID-1) >0));
                end
                obj.additionalDof(parents >0)=[];
            end
            if ~isempty(obj.additionalDof)
                for i=1:length(obj.allBeams)
                    obj.allBeams{i}.id=obj.allBeams{i}.id+length(obj.additionalDof);
                    obj.allBeams{i}=obj.allBeams{i}.updateInternalID(length(obj.additionalDof));
                end
            end
        end
        
        function obj = getAdjacencyMatrix(obj)
            %ADJACENCYMATRIX obtain the adjacency matrix from node connections
            allConnections=zeros(length(obj.connections),2);
            for i=1:length(obj.connections)
                allConnections(i,:)=obj.connections(i).nodes;
            end
            uniqueNodes=unique(allConnections);
            nodeMatrixLength=length(uniqueNodes);
            obj.adjacencyMatrix=zeros(nodeMatrixLength,nodeMatrixLength);
            for i=1:nodeMatrixLength
                [a,b]=find(allConnections==uniqueNodes(i));
                for k=1:length(a)
                    if b(k) ==1
                        neigbour=allConnections(a(k),2);
                    else
                        neigbour=allConnections(a(k),1);
                    end
                    obj.adjacencyMatrix(i,neigbour)=1;
                end
            end
            %check if matrix is symmetric
            if (isequal(obj.adjacencyMatrix,obj.adjacencyMatrix')) ==0
                disp('Adjacency Matrix is not Symmetric');
            end
            %find number of independent obj.loops
            diffNodes=unique(allConnections);
            links=length(allConnections);
            joints=0;
            for i=1:length(diffNodes)
                joints=joints+length(find(allConnections==diffNodes(i)))-1;
            end
            obj.independentLoops=joints-links+1;
        end
        
        function  obj = findLoops(obj)
            %FINDLOOPS find the obj.loops and store in the obj.loops matrix
            allConnections=zeros(length(obj.connections),2);
            for i=1:length(obj.connections)
                allConnections(i,:)=obj.connections(i).nodes;
            end
            diffNodes=unique(allConnections);
            adjMatrix=obj.adjacencyMatrix;
            
            for i=1:length(obj.rigidPlates)
                nodes(1,1)=obj.allBeams{obj.rigidPlates(i)}.nodes(1) ;
                nodes(1,2)=obj.allBeams{obj.rigidPlates(i)}.nodes(2) ;
                adjMatrix(nodes(1,1),nodes(1,2))=0;
                adjMatrix(nodes(1,2),nodes(1,1))=0;
                %delete from connections
                index=ismember(allConnections,[nodes(1,1) nodes(1,2)],'rows');
                allConnections(index==1,:)=[];
                index=ismember(allConnections,[nodes(1,2) nodes(1,1)],'rows');
                allConnections(index==1,:)=[];
            end
            
            %remove the elements with just one neigbour
            for j =1:length(adjMatrix)
                for i=1:length(adjMatrix)
                    neighbors=length(find(adjMatrix(i,:)== 1));
                    if neighbors ==1
                        %delete from connections
                        [row , ~]=find(allConnections == i);
                        allConnections(row,:)=[];
                        adjMatrix(:,i)=0;
                        adjMatrix(i,:)=0;
                        diffNodes(diffNodes==i)=[];
                    end
                end
            end
            
            if ~isempty(adjMatrix)
                paths=Graph(adjMatrix).base;
                obj.loops=paths.getPath(length(adjMatrix)+1);
            else
                obj.loops=zeros(obj.independentLoops,length(adjMatrix)+1);
            end
        end
        
        function [joints,nodes]=allJoints(obj)
            %return all joints
            nodes=zeros(length(obj.allBeams),2);
            for i=1:length(obj.allBeams)
                joints(i,:)=obj.allBeams{i}.joints;
                nodes(i,:)=obj.allBeams{i}.nodes;
            end
        end
        
        function row = findBeam(obj,node1,node2 )
            %FINDBEAM find index of the beam which is node1 to node2
            %   returns minus if beam is node2 to node1
            row=0;
            allConnections=zeros(length(obj.allBeams),2);
            for i=1:length(obj.allBeams)
                allConnections(i,:)=obj.allBeams{i}.nodes;
            end
            [a,b]=find(allConnections==node1);
            for i=1:length(a)
                if b(i)== 1 && allConnections(a(i),2) == node2
                    row=a(i);
                else
                    if b(i)== 2 && allConnections(a(i),1) == node2
                        row=-a(i);
                    end
                end
            end
            if row == 0
                errordlg('Could not find the beam');
            end
        end
        
        function  equations= kinematicEquations(obj,loops,requiredEquations)
            %KINEMATICEQUATIONS find kinematic equations for the loops
            equations=struct('beamList',[]);
            for i=1:requiredEquations
                A=zeros(1,length(obj.allBeams));
                equations(i)=struct('beamList',A);
            end
            for i=1:length(loops(:,1))
                j=1;
                while j~=length(loops(1,:))
                    node1=loops(i,j);
                    node2=loops(i,j+1);
                    if node2 == 0
                        break;
                    end
                    beamIndex=obj.findBeam(node1,node2);
                    equations(i).beamList(abs(beamIndex))=sign(beamIndex);
                    j=j+1;
                end
            end
        end
        
        function realInputs=getInputs(~,inputs)
            %get the real inputs necessary for kinematics
            realInputs(length(inputs))=struct('index',[],'dof',[],'value',[]);
            omittedInputs=[];
            for i=1:length(inputs)
                if isempty(inputs(i).linkID)
                    omittedInputs(end+1)=i;
                    continue;
                end
                realInputs(i).index=inputs(i).linkID;
                realInputs(i).dof=inputs(i).inputType;
                %check if relative
                if inputs(i).isRelative == 0
                    if strcmp(inputs(i).type,'Angle')
                        realInputs(i).value=inputs(i).target(1,1);
                    else
                        realInputs(i).value=inputs(i).target(1,1);
                    end
                else
                    if strcmp(inputs(i).type,'Angle')
                        realInputs(i).value=inputs(i).initialValue+inputs(i).target(1,1);
                    else
                        realInputs(i).value=inputs(i).initialValue+inputs(i).target(1,1);
                    end
                end
            end
            realInputs(omittedInputs)=[];
        end
        
        function [outputMatrix,indexList,initialGuess] = findInputMatrix(obj,inputMatrix,realInputs )
            %prepares the realInputs matrix for solvers
            switch class(obj.allBeams{end})
                case 'KinematicsBeam'
                    degreesOfFreedom=obj.allBeams{end}.id+1;
                otherwise
                    degreesOfFreedom=obj.allBeams{end}.id+obj.allBeams{end}.degreesOfFreedom-1;
            end
            initialGuess=[];
            outputMatrix=zeros(1,degreesOfFreedom);
            indexList=zeros(1,degreesOfFreedom);
            %assign the inputs
            %realInputs(length(inputs))=struct('index',[],'dof',[],'value',[]);
            for i=1:length(realInputs)
                if realInputs(i).index > 0
                    switch class(obj.allBeams{realInputs(i).index})
                        case 'KinematicsBeam'
                            if ~logical(realInputs(i).value)
                                realInputs(i).value=1e-10;
                            end
                            if logical(obj.allBeams{realInputs(i).index}.frontMasterID)
                                outputMatrix(obj.allBeams{realInputs(i).index}.frontMasterID)=realInputs(i).value-obj.allBeams{realInputs(i).index}.theta0;
                            elseif logical(obj.allBeams{realInputs(i).index}.endMasterID)
                                inputValue=realInputs(i).value-obj.allBeams{realInputs(i).index}.theta0;
                                if ~logical(inputValue)
                                    inputValue=1e-6;
                                end
                                outputMatrix(obj.allBeams{realInputs(i).index}.endMasterID)=inputValue;
                            else
                                outputMatrix(obj.allBeams{realInputs(i).index}.id+realInputs(i).dof-1)=realInputs(i).value;
                            end
                    end
                end
            end
            inputPosition=1;
            %add the additional inputs
            for i=1:length(obj.additionalDof)
                if ~logical(obj.additionalDof(i).known) && ~logical(outputMatrix(i))
                    obj.additionalDof(i).value=inputMatrix(inputPosition);
                    outputMatrix(i)=inputMatrix(inputPosition);
                    indexList(i)=inputPosition;
                    inputPosition=inputPosition+1;
                    initialGuess(end+1)=obj.additionalDof(i).initialGuess;
                end
            end
            for i=1:length(obj.allBeams)
                switch class(obj.allBeams{i})
                    case 'KinematicsBeam'
                        if strcmp(obj.allBeams{i}.type,'slider')
                            if ~logical(outputMatrix(obj.allBeams{i}.id))
                                %length
                                outputMatrix(obj.allBeams{i}.id)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=obj.allBeams{i}.length;
                            end
                        elseif strcmp(obj.allBeams{i}.type,'allFixedSlider')
                            if ~logical(outputMatrix(obj.allBeams{i}.id))
                                %length
                                outputMatrix(obj.allBeams{i}.id)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=obj.allBeams{i}.calculateInitialGuess();
                            end
                        elseif obj.allBeams{i}.fixed == 1 || (obj.allBeams{i}.degreesOfFreedom == 1 &&  logical(obj.allBeams{i}.linearSpring)) || obj.allBeams{i}.degreesOfFreedom == 0
                            if logical(obj.allBeams{i}.frontMasterID) || logical(obj.allBeams{i}.endMasterID)
                                if logical(obj.allBeams{i}.frontMasterID)
                                    master=obj.allBeams{i}.frontMasterID;
                                else
                                    master=obj.allBeams{i}.endMasterID;
                                end
                                outputMatrix(obj.allBeams{i}.id)=obj.allBeams{i}.theta0+outputMatrix(master);
                                indexList(obj.allBeams{i}.id)=indexList(master);
                            else
                                outputMatrix(obj.allBeams{i}.id)=obj.allBeams{i}.theta;
                            end
                            %check if linear spring
                            if logical(obj.allBeams{i}.linearSpring)
                                if ~logical(outputMatrix(obj.allBeams{i}.id+1))
                                    outputMatrix(obj.allBeams{i}.id+1)=inputMatrix(inputPosition);
                                    indexList(obj.allBeams{i}.id+1)=inputPosition;
                                    inputPosition=inputPosition+1;
                                    initialGuess(end+1)=obj.allBeams{i}.length;
                                end
                            end
                        elseif obj.allBeams{i}.degreesOfFreedom >1
                            if ~logical(outputMatrix(obj.allBeams{i}.id))
                                if logical(obj.allBeams{i}.theta)
                                    initialGuess(end+1)=obj.allBeams{i}.theta;
                                else
                                    initialGuess(end+1)=1e-6;
                                end
                                outputMatrix(obj.allBeams{i}.id)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id)=inputPosition;
                                inputPosition=inputPosition+1;
                            end
                            if ~logical(outputMatrix(obj.allBeams{i}.id+1))
                                initialGuess(end+1)=obj.allBeams{i}.length;
                                outputMatrix(obj.allBeams{i}.id+1)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+1)=inputPosition;
                                inputPosition=inputPosition+1;
                            end
                        else
                            if ~logical(outputMatrix(obj.allBeams{i}.id))
                                if logical(obj.allBeams{i}.frontMasterID) || logical(obj.allBeams{i}.endMasterID)
                                    if logical(obj.allBeams{i}.frontMasterID)
                                        master=obj.allBeams{i}.frontMasterID;
                                    else
                                        master=obj.allBeams{i}.endMasterID;
                                    end
                                    outputMatrix(obj.allBeams{i}.id)=obj.allBeams{i}.theta0+outputMatrix(master);
                                    indexList(obj.allBeams{i}.id)=indexList(master);
                                else
                                    outputMatrix(obj.allBeams{i}.id)=inputMatrix(inputPosition);
                                    indexList(obj.allBeams{i}.id)=inputPosition;
                                    inputPosition=inputPosition+1;
                                    if logical(obj.allBeams{i}.theta)
                                        initialGuess(end+1)=obj.allBeams{i}.theta;
                                    else
                                        initialGuess(end+1)=1e-6;
                                    end
                                end
                            end
                        end
                    case 'PrbBeam'
                        initialGuessBeam=obj.allBeams{i}.getInitialGuess();
                        if logical(obj.allBeams{i}.frontMasterID) && logical(obj.allBeams{i}.endMasterID) && ~isinf(obj.allBeams{i}.crossSection.prbModel(end,end))
                            %two end masters and linear spring at the end
                            %first prb angle
                            outputMatrix(obj.allBeams{i}.id)=obj.allBeams{i}.prbBeams(1).theta0+outputMatrix(obj.allBeams{i}.frontMasterID);
                            indexList(obj.allBeams{i}.id)=indexList(obj.allBeams{i}.frontMasterID);
                            %other dof
                            for j=1:obj.allBeams{i}.degreesOfFreedom-3
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                            %final prb angle
                            outputMatrix(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-2)=obj.allBeams{i}.prbBeams(end).theta0+outputMatrix(obj.allBeams{i}.endMasterID);
                            indexList(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-2)=indexList(obj.allBeams{i}.endMasterID);
                            %final length
                            outputMatrix(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-1)=inputMatrix(inputPosition);
                            indexList(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-1)=inputPosition;
                            inputPosition=inputPosition+1;
                            initialGuess(end+1)=initialGuessBeam(end);
                        elseif logical(obj.allBeams{i}.frontMasterID) && logical(obj.allBeams{i}.endMasterID)
                            %two end masters
                            outputMatrix(obj.allBeams{i}.id)=obj.allBeams{i}.prbBeams(1).theta0+outputMatrix(obj.allBeams{i}.frontMasterID);
                            indexList(obj.allBeams{i}.id)=indexList(obj.allBeams{i}.frontMasterID);
                            %other dof
                            for j=1:obj.allBeams{i}.degreesOfFreedom-2
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                            %final prb angle
                            outputMatrix(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-1)=obj.allBeams{i}.prbBeams(end).theta0+outputMatrix(obj.allBeams{i}.endMasterID);
                            indexList(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-1)=indexList(obj.allBeams{i}.endMasterID);
                        elseif logical(obj.allBeams{i}.frontMasterID)
                            %just front master
                            %first prb angle
                            outputMatrix(obj.allBeams{i}.id)=obj.allBeams{i}.prbBeams(1).theta0+outputMatrix(obj.allBeams{i}.frontMasterID);
                            indexList(obj.allBeams{i}.id)=indexList(obj.allBeams{i}.frontMasterID);
                            %other dof
                            for j=1:obj.allBeams{i}.degreesOfFreedom-1
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                        elseif logical(obj.allBeams{i}.endMasterID) && ~isinf(obj.allBeams{i}.crossSection.prbModel(end,end))
                            %just end master and linear spring at the end
                            %other dof
                            for j=0:obj.allBeams{i}.degreesOfFreedom-3
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                            %final prb angle
                            outputMatrix(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-2)=obj.allBeams{i}.prbBeams(end).theta0+outputMatrix(obj.allBeams{i}.endMasterID);
                            indexList(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-2)=indexList(obj.allBeams{i}.endMasterID);
                            %final length
                            outputMatrix(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-1)=inputMatrix(inputPosition);
                            indexList(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-1)=inputPosition;
                            inputPosition=inputPosition+1;
                            initialGuess(end+1)=initialGuessBeam(end);
                        elseif logical(obj.allBeams{i}.endMasterID)
                            %just end master
                            %other dof
                            for j=0:obj.allBeams{i}.degreesOfFreedom-2
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                            %final prb angle
                            outputMatrix(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-1)=obj.allBeams{i}.prbBeams(end).theta0+outputMatrix(obj.allBeams{i}.endMasterID);
                            indexList(obj.allBeams{i}.id+obj.allBeams{i}.degreesOfFreedom-1)=indexList(obj.allBeams{i}.endMasterID);
                        else
                            %no masters
                            for j=0:obj.allBeams{i}.degreesOfFreedom-1
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                        end
                    case {'BcmBeam','LinearBeam'}
                        initialGuessBeam=obj.allBeams{i}.getInitialGuess();
                        if logical(obj.allBeams{i}.frontMasterID) && logical(obj.allBeams{i}.endMasterID)
                            %master angle
                            outputMatrix(obj.allBeams{i}.id)=outputMatrix(obj.allBeams{i}.frontMasterID);
                            indexList(obj.allBeams{i}.id)=indexList(obj.allBeams{i}.frontMasterID);
                            %delta x,delta y
                            for j=1:2
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                            %theta
                            outputMatrix(obj.allBeams{i}.id+3)=outputMatrix(obj.allBeams{i}.endMasterID);
                            indexList(obj.allBeams{i}.id+3)=indexList(obj.allBeams{i}.endMasterID);
                        elseif logical(obj.allBeams{i}.frontMasterID)
                            %master angle
                            outputMatrix(obj.allBeams{i}.id)=outputMatrix(obj.allBeams{i}.frontMasterID);
                            indexList(obj.allBeams{i}.id)=indexList(obj.allBeams{i}.frontMasterID);
                            %delta x,delta y,delta theta
                            for j=1:3
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                        elseif logical(obj.allBeams{i}.endMasterID)
                            %master angle
                            outputMatrix(obj.allBeams{i}.id)=0.0;
                            indexList(obj.allBeams{i}.id)=0;
                            %delta x,delta y
                            for j=1:2
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                            %theta
                            outputMatrix(obj.allBeams{i}.id+3)=outputMatrix(obj.allBeams{i}.endMasterID);
                            indexList(obj.allBeams{i}.id+3)=indexList(obj.allBeams{i}.endMasterID);
                        else
                            %master angle
                            outputMatrix(obj.allBeams{i}.id)=0.0;
                            indexList(obj.allBeams{i}.id)=0;
                            %delta x,delta y,delta theta
                            for j=1:3
                                outputMatrix(obj.allBeams{i}.id+j)=inputMatrix(inputPosition);
                                indexList(obj.allBeams{i}.id+j)=inputPosition;
                                inputPosition=inputPosition+1;
                                initialGuess(end+1)=initialGuessBeam(j+1);
                            end
                        end
                    case {'CbcmBeam','MlinearBeam'}
                        id=obj.allBeams{i}.id;
                        if logical(obj.allBeams{i}.frontMasterID) && logical(obj.allBeams{i}.endMasterID)
                            %tranformation angle
                            for j=1:obj.allBeams{i}.segments
                                initialGuessBeam=obj.allBeams{i}.getInitialGuess(j);
                                %master angle
                                if j == 1
                                    outputMatrix(id+(j-1)*4)=outputMatrix(obj.allBeams{i}.frontMasterID);
                                    indexList(id+(j-1)*4)=indexList(obj.allBeams{i}.frontMasterID);
                                else
                                    outputMatrix(id+(j-1)*4)=outputMatrix(id+(j-2)*4+3);
                                    indexList(id+(j-1)*4)=indexList(id+(j-2)*4+3);
                                end
                                %delta x,delta y
                                for k=1:2
                                    outputMatrix(id+(j-1)*4+k)=inputMatrix(inputPosition);
                                    indexList(id+(j-1)*4+k)=inputPosition;
                                    inputPosition=inputPosition+1;
                                    initialGuess(end+1)=initialGuessBeam(k+1);
                                end
                                %theta
                                if j == obj.allBeams{i}.segments
                                    outputMatrix(id+(j-1)*4+3)=outputMatrix(obj.allBeams{i}.endMasterID);
                                    indexList(id+(j-1)*4+3)=indexList(obj.allBeams{i}.endMasterID);
                                else
                                    outputMatrix(id+(j-1)*4+3)=inputMatrix(inputPosition);
                                    indexList(id+(j-1)*4+3)=inputPosition;
                                    inputPosition=inputPosition+1;
                                    initialGuess(end+1)=initialGuessBeam(end);
                                end
                            end
                        elseif logical(obj.allBeams{i}.frontMasterID)
                            %tranformation angle
                            for j=1:obj.allBeams{i}.segments
                                initialGuessBeam=obj.allBeams{i}.getInitialGuess(j);
                                %master angle
                                if j == 1
                                    outputMatrix(id+(j-1)*4)=outputMatrix(obj.allBeams{i}.frontMasterID);
                                    indexList(id+(j-1)*4)=indexList(obj.allBeams{i}.frontMasterID);
                                else
                                    outputMatrix(id+(j-1)*4)=outputMatrix(id+(j-2)*4+3);
                                    indexList(id+(j-1)*4)=indexList(id+(j-2)*4+3);
                                end
                                %delta x,delta y,theta
                                for k=1:3
                                    outputMatrix(id+(j-1)*4+k)=inputMatrix(inputPosition);
                                    indexList(id+(j-1)*4+k)=inputPosition;
                                    inputPosition=inputPosition+1;
                                    initialGuess(end+1)=initialGuessBeam(k+1);
                                end
                            end
                        elseif logical(obj.allBeams{i}.endMasterID)
                            for j=1:obj.allBeams{i}.segments
                                initialGuessBeam=obj.allBeams{i}.getInitialGuess(j);
                                %master angle
                                if j == 1
                                    outputMatrix(id+(j-1)*4)=0;
                                    indexList(id+(j-1)*4)=0;
                                else
                                    outputMatrix(id+(j-1)*4)=outputMatrix(id+(j-2)*4+3);
                                    indexList(id+(j-1)*4)=indexList(id+(j-2)*4+3);
                                end
                                %delta x,delta y
                                for k=1:2
                                    outputMatrix(id+(j-1)*4+k)=inputMatrix(inputPosition);
                                    indexList(id+(j-1)*4+k)=inputPosition;
                                    inputPosition=inputPosition+1;
                                    initialGuess(end+1)=initialGuessBeam(k+1);
                                end
                                %theta
                                if j == obj.allBeams{i}.segments
                                    outputMatrix(id+(j-1)*4+3)=outputMatrix(obj.allBeams{i}.endMasterID);
                                    indexList(id+(j-1)*4+3)=indexList(obj.allBeams{i}.endMasterID);
                                else
                                    outputMatrix(id+(j-1)*4+3)=inputMatrix(inputPosition);
                                    indexList(id+(j-1)*4+3)=inputPosition;
                                    inputPosition=inputPosition+1;
                                    initialGuess(end+1)=initialGuessBeam(end);
                                end
                            end
                        else
                            for j=1:obj.allBeams{i}.segments
                                initialGuessBeam=obj.allBeams{i}.getInitialGuess(j);
                                %master angle
                                if j == 1
                                    outputMatrix(id+(j-1)*4)=0;
                                    indexList(id+(j-1)*4)=0;
                                else
                                    outputMatrix(id+(j-1)*4)=outputMatrix(id+(j-2)*4+3);
                                    indexList(id+(j-1)*4)=indexList(id+(j-2)*4+3);
                                end
                                %delta x,delta y
                                for k=1:3
                                    outputMatrix(id+(j-1)*4+k)=inputMatrix(inputPosition);
                                    indexList(id+(j-1)*4+k)=inputPosition;
                                    inputPosition=inputPosition+1;
                                    initialGuess(end+1)=initialGuessBeam(k+1);
                                end
                            end
                        end
                end
            end
        end
        
        
        function [xValue, yValue, gradient, hessian] = solveStaticEquations(obj, outputMatrix, equations, indexList)
            if nargin < 4 || isempty(indexList)
                indexList = zeros(length(outputMatrix), 1);
            end

            calcGradient = nargout >= 3;
            calcHessian = nargout >= 4;
            maxIdx = max(indexList);

            j = 1;
            xValue = zeros(1, length(equations));
            yValue = zeros(1, length(equations));
            
            if calcGradient
                gradient = zeros(maxIdx, length(equations) * 2);
            else
                gradient = [];
            end
            
            if calcHessian
                hessian = cell(1, length(equations) * 2);
            else
                hessian = [];
            end

            for i = 1:length(equations)
                xValue(i) = 0;
                yValue(i) = 0;
                
                if calcGradient
                    gradientX = zeros(maxIdx, 1);
                    gradientY = zeros(maxIdx, 1);
                end
                if calcHessian
                    hessianX = sparse(maxIdx, maxIdx);
                    hessianY = sparse(maxIdx, maxIdx);
                end

                for k = 1:length(equations(i).beamList)
                    if logical(equations(i).beamList(k))
                        beam = obj.allBeams{k};
                        dist = equations(i).beamList(k) * 100;
                        
                        % 1. Calculate X and Y positions
                        xValue(i) = xValue(i) + beam.getX(outputMatrix, dist);
                        yValue(i) = yValue(i) + beam.getY(outputMatrix, dist);
                        
                        % 2. Calculate Gradients
                        if calcGradient
                            try
                                gX = beam.getGradientX(outputMatrix, indexList, dist);
                                gY = beam.getGradientY(outputMatrix, indexList, dist);
                            catch
                                gX = beam.getGradientX(indexList, dist);
                                gY = beam.getGradientY(indexList, dist);
                            end
                            
                            % Pad gradient vectors to prevent dimension mismatch
                            if maxIdx > 0 && size(gX, 1) < maxIdx
                                gX(maxIdx, 1) = 0;
                                gY(maxIdx, 1) = 0;
                            end
                            gradientX = gradientX + gX;
                            gradientY = gradientY + gY;
                        end
                        
                        % 3. Calculate Hessians
                        if calcHessian
                            try
                                hX = beam.getHessianX(outputMatrix, indexList, dist);
                                hY = beam.getHessianY(outputMatrix, indexList, dist);
                            catch
                                hX = beam.getHessianX(indexList, dist);
                                hY = beam.getHessianY(indexList, dist);
                            end
                            
                            % Pad hessian matrices
                            if maxIdx > 0
                                [rX, cX] = size(hX);
                                if rX < maxIdx || cX < maxIdx
                                    hX(maxIdx, maxIdx) = 0;
                                    hY(maxIdx, maxIdx) = 0;
                                end
                            end
                            hessianX = hessianX + hX;
                            hessianY = hessianY + hY;
                        end
                    end
                end
                
                % Assign to outputs
                if calcGradient
                    gradient(:, 2*i-1) = gradientX;
                    gradient(:, 2*i) = gradientY;
                end
                if calcHessian
                    hessian{1, 2*i-1} = hessianX;
                    hessian{1, 2*i} = hessianY;
                end
                
                j = j + 2;
            end
        end
        
        function [F,J] = solveEquations(obj,x,realInputs)
            %SOLVEEQUATIONS solve the kinematic equations(for fsolve)
            [outputMatrix,indexList,~] = obj.findInputMatrix(x,realInputs );
%             for i=1:length(obj.allBeams)
%                 obj.allBeams{i}=obj.allBeams{i}.updateBeam(outputMatrix,indexList);
%             end
            j=1;
            F=zeros(length(obj.equations)*2,1);
            J=zeros(length(obj.equations)*2,length(x));
            for i=1:length(obj.equations)
                xValue=0;yValue=0;
                for k=1:length(obj.equations(i).beamList)
                    if logical(obj.equations(i).beamList(k))
                        xValue=xValue+obj.allBeams{k}.getX(outputMatrix,obj.equations(i).beamList(k)*100);
                        yValue=yValue+obj.allBeams{k}.getY(outputMatrix,obj.equations(i).beamList(k)*100);
                        if nargout > 1
                            gradientX= obj.allBeams{k}.getGradientX(outputMatrix,indexList,obj.equations(i).beamList(k)*100)';
                            gradientY= obj.allBeams{k}.getGradientY(outputMatrix,indexList,obj.equations(i).beamList(k)*100)';
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
        
        function updatedNodes = updateNodes(obj,inputs )
            %UPDATENODES updates the nodes with the recent coordinates
            %update beams
            for i=1:length(obj.allBeams)
                obj.allBeams{i}=obj.allBeams{i}.updateBeam(inputs,inputs);
            end
            
            updatedNodes=obj.nodes;
            for i=1:length(obj.nodes)
                updatedNodes(i)=updatedNodes(i).setUpdated(0);
            end
            i=1;
            loopCount=1;
            while 1 && (loopCount < length(obj.allBeams)*100)
                if(i > length(obj.allBeams))
                    i=1;
                end
                %check if all nodes are updated
                allUpdated=1;
                for j=1:length(obj.uniqueConnections)
                    if updatedNodes(obj.uniqueConnections(j)).getUpdated() ~= 1
                        allUpdated=0;
                        break;
                    end
                end
                if allUpdated == 1
                    break;
                end
                
                if obj.allBeams{i}.joints(1) == Joint.GroundPin || obj.allBeams{i}.joints(1) == Joint.GroundWelded || updatedNodes(obj.allBeams{i}.nodes(1,1)).getUpdated()==1
                    updatedNodes(obj.allBeams{i}.nodes(1,1))=updatedNodes(obj.allBeams{i}.nodes(1,1)).setUpdated(1);
                    updatedNodes(obj.allBeams{i}.nodes(1,2))=updatedNodes(obj.allBeams{i}.nodes(1,2)).setUpdated(1);
                    point=updatedNodes(obj.allBeams{i}.nodes(1,1)).getNode();
                    x=point.x+obj.allBeams{i}.getX(inputs,100);
                    y=point.y+obj.allBeams{i}.getY(inputs,100);
                    updatedNodes(obj.allBeams{i}.nodes(1,2))=updatedNodes(obj.allBeams{i}.nodes(1,2)).setNode(Point(x,y));
                elseif updatedNodes(obj.allBeams{i}.nodes(1,2)).getUpdated()==1
                    updatedNodes(obj.allBeams{i}.nodes(1,1))=updatedNodes(obj.allBeams{i}.nodes(1,1)).setUpdated(1);
                    point=updatedNodes(obj.allBeams{i}.nodes(1,2)).getNode();
                    x=point.x-obj.allBeams{i}.getX(inputs,100);
                    y=point.y-obj.allBeams{i}.getY(inputs,100);
                    updatedNodes(obj.allBeams{i}.nodes(1,1))=updatedNodes(obj.allBeams{i}.nodes(1,1)).setNode(Point(x,y));
                end
                i=i+1;
                loopCount=loopCount+1;
            end
            
        end
        
        
        function [obj,exitflag] = findNewConfig(obj,realInputs)
            %FINDNEWCONFG update the nodes according to the input
            %add all initial conditions except the inputs
            [~,~,initialGuess] = obj.findInputMatrix(zeros(obj.allBeams{end}.id+obj.allBeams{end}.degreesOfFreedom+3,1),realInputs );
            options=optimoptions('fsolve','Display','none','SpecifyObjectiveGradient',false,'OptimalityTolerance',1e-10,'TolX',1e-10,'MaxIter',150,'MaxFunEvals',150,'FunctionTolerance',1e-10,...
                'CheckGradients',false,'FiniteDifferenceType','forward');   % Option to display output
            f=@(x)obj.solveEquations(x,realInputs);
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
                [newState,indexList,~] = obj.findInputMatrix(x,realInputs );
                for i=1:length(obj.allBeams)
                    obj.allBeams{i}=obj.allBeams{i}.updateBeam(newState,indexList);
                end
                obj.nodes = obj.updateNodes(newState);
                obj.nodeList(end+1,:)=obj.addAllNodes(obj.nodes);
                obj=obj.addState(newState);
            end
        end
        
%         %create the coupler curve
%         function [couplerCurve,isClosed]= getCouplerCurve(kinematics,driverIndex,nodeIndex,varargin)        
%             %
%             if isempty(varargin)%no input increment is 2 degrees
%                 increment=2*pi/180;
%             else
%                 increment=varargin{1}*pi/180;
%             end
%             angles=kinematics.allBeams{driverIndex}.theta0+linspace(0,2*pi,round(2*pi/increment)+1);
%             
%             %first go forward in direction
%             for i=1:length(angles)
%                 realInputs=struct('index',1,'dof',1,'value',angles(i));
%                 [kinematics,exitflag] = kinematics.findNewConfig(realInputs);
%             end
%             couplerCurve=1;
%             isClosed=1;
%         end
        
        
        function obj=rangeSimulation(obj,mainFig,limit,mode,parent,steps)
            %the main function for range simulation
            %get the inputs
            obj.originalNodes=obj.nodes;
            obj.workspace=parent.getWorkspace();
            dummyInputs=obj.inputs ;
            inputRange=[];
            %delete unused inputs
            willDeleted=[];
            for i=2:length(dummyInputs)
                if isempty(dummyInputs(i).linkID)
                    willDeleted(end+1)=i;
                end
            end
            dummyInputs(willDeleted)=[];
            for i=1:length(dummyInputs)
                inputRange(end+1,:)=linspace(obj.inputs(i).target(1,1),obj.inputs(i).target(1,2),steps);
            end
            %save the initial state
            [~,~,initialGuess] = obj.findInputMatrix(zeros(obj.allBeams{end}.id+obj.allBeams{end}.degreesOfFreedom+3,1),[] ); 
            [initialState,~,~] = obj.findInputMatrix(initialGuess,[] ); 
            for i=1:steps
                for j=1:length(dummyInputs)
                    dummyInputs(j).target(1,1)=inputRange(j,i);
                end
                realInputs=obj.getInputs(dummyInputs) ;
                [obj,exitflag] = obj.findNewConfig(realInputs);
                if exitflag>0
                    %draw
                    obj=obj.drawAll(mainFig,limit,mode,parent);
                    %status string
                    if strcmp(obj.inputs(1).type,'Angle')
                        parent.setStatusText(['Animating... Input Value:' num2str(dummyInputs(1).target(1,1)*180/pi)]);
                    else
                        parent.setStatusText(['Animating... Input Value:' num2str(dummyInputs(1).target(1,1))]);
                    end
                    drawnow;
                else
                    obj.allBeams{obj.inputs(1).linkID}=obj.allBeams{obj.inputs(1).linkID}.colorLine(Colors.StatusIncomplete.getColor());
                    %status string
                    if strcmp(obj.inputs(1).type,'Angle')
                        parent.setStatusText(['Animation stopped at Input Value:' num2str(dummyInputs(1).target(1,1)*180/pi)]);
                    else
                        parent.setStatusText(['Animation stopped at Input Value:' num2str(dummyInputs(1).target(1,1))]);
                    end
                    break;
                end
            end
            %return to original
            for j=1:length(obj.allBeams)
                obj.allBeams{j}=obj.allBeams{j}.updateBeam(initialState,initialState);
            end
        end
        
        
        function obj=simulationNoGUI(obj,steps)
            %the main function for range simulation
            %get the inputs
            obj.originalNodes=obj.nodes;
            dummyInputs=obj.inputs ;
            inputRange=[];
            %delete unused inputs
            willDeleted=[];
            for i=2:length(dummyInputs)
                if isempty(dummyInputs(i).linkID)
                    willDeleted(end+1)=i;
                end
            end
            dummyInputs(willDeleted)=[];
            for i=1:length(dummyInputs)
                inputRange(end+1,:)=linspace(obj.inputs(i).target(1,1),obj.inputs(i).target(1,2),steps);
            end
            %save the initial state
            [~,~,initialGuess] = obj.findInputMatrix(zeros(obj.allBeams{end}.id+obj.allBeams{end}.degreesOfFreedom+3,1),[] );
            [initialState,~,~] = obj.findInputMatrix(initialGuess,[] ); 
            for i=1:steps
                for j=1:length(dummyInputs)
                    dummyInputs(j).target(1,1)=inputRange(j,i) ;
                end
                realInputs=obj.getInputs(dummyInputs) ;
                [obj,exitflag] = obj.findNewConfig(realInputs);
                if exitflag <= 0
                    break;
                end
            end
            %return to original
            for j=1:length(obj.allBeams)
                obj.allBeams{j}=obj.allBeams{j}.updateBeam(initialState,initialState);
            end
        end
        
        
        function obj=addInput(obj,index,type,link,target,relative,links)
            %add range input
            switch type
                case 'Angle'
                    linkID=Helper.findLink(link,links,obj.allBeams,1);
                    inputType=1;
                    modifier=pi/180;
                case 'Length'
                    linkID=Helper.findLink(link,links,obj.allBeams,1);
                    inputType=2;
                    modifier=1;
                case 'Slider'
                    linkID=Helper.findLink(link,links,obj.allBeams,2);
                    inputType=1;
                    modifier=1;
            end
            if ~isempty(obj.allBeams{linkID}.calculateInitialGuess())
                initialValue=obj.allBeams{linkID}.calculateInitialGuess();
                initialValue=initialValue(inputType);
            else
                initialValue=obj.allBeams{linkID}.angle;
            end
            %
            obj.inputs(index)=struct('linkID',linkID,'inputType',inputType,'target',target*modifier,'isRelative',relative,'initialValue',initialValue,'type',[]);
        end
        
        function obj=deleteInput(obj,index)
            %delete range input
            obj.inputs(index)=struct('linkID',[],'inputType',[],'target',0,'isRelative',0,'initialValue',0,'type',0);
        end
        
        
        
        function allNodes=addAllNodes(obj,newNodes)
            %add all nodes to the list
            allNodes=zeros(1,length(newNodes)*2);
            for i=1:length(newNodes)
                node=newNodes(i).getNode();
                allNodes(1,2*i-1)=node.x;
                allNodes(1,2*i)=node.y;
            end
        end
        
        function obj=addState(obj,state)
            %add state
            obj.stateList{end+1}=state;
        end
        
        function obj=freeSimulation(obj,mainFig,limit,mode,parent)
            %the main function for free simulation
            %get the inputs
            dummyInputs=obj.inputs;
            realInputs=obj.getInputs(dummyInputs);
            obj.workspace=parent.getWorkspace();
            [obj,exitflag] = obj.findNewConfig(realInputs);
            if exitflag>0
                %draw
                obj=obj.drawAll(mainFig,limit,mode,parent);
                drawnow;
            else
                obj.allBeams{obj.inputs(1).linkID}=obj.allBeams{obj.inputs(1).linkID}.colorLine(Colors.StatusIncomplete.getColor());
                return;
            end
            
        end
        
    end
    
    
end
