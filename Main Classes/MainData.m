classdef MainData < handle
    %the main class which contains all design and analysis modules
    properties(Access = private)
        mainFig;
        popFig=[];
        popFigName=[];
        nodes;
        links;
        constraints=Constraint.empty;
        torsionSprings=TorsionSpring.empty;
        forces=Force.empty;
        moments=Moment.empty;
        currentWorkSpace;
        fileName;
        mode;
        degreesOfFreedom;
        multipleSelect=0;
        nodesDraggable=1;
        draggedNode=0;
        selectedConstraint=0;
        groupID=1;
        handle;
        stop=0;
        sessionKinematics=Kinematics.empty;
        freeKinematics=struct('mode',[],'link',0,'node',0);
        sessionLoadAnalysis=LoadAnalysis.empty;
        sessionDistanceAnalysis=DistanceAnalysis.empty;
        sessionAdvantageAnalysis=Advantage.empty;
        sessionBistable=BiStable.empty;
        sessionFlexuralStiffnessSynthesis=FlexuralStiffnessSynthesis.empty;
        sessionBiStableSynthesis=BiStableSynthesis.empty;
        sessionConstantForceSynthesis=ConstantForceSynthesis.empty;
        helpID=[2 1];
        progressBar=[];
        busy=0;
    end
    
    methods
        function obj=MainData(currentWorkSpace,fileName,axis,handles)
            obj.currentWorkSpace=currentWorkSpace;
            %nodes and links
            obj.nodes=Node.empty;
            obj.links=Link.empty;
            obj.fileName=fileName;
            obj.mainFig=axis;
            obj.handle=handles.mainGUI;
            obj.sessionKinematics=Kinematics(obj.links,obj.nodes,obj.currentWorkSpace);
        end
        
        function setHelp(obj,help)
            %set hel ids
            obj.helpID=help;
        end
        
        function openHelp(obj)
            %open help gui
            helpGUI('id',obj.helpID(1,1),obj.helpID(1,2));
        end
        
        function stop=getStop(obj)
            %get the stop value
            stop=obj.stop;
        end
        
        function setStop(obj,stop)
            %set the stop value
            obj.stop=stop;
        end
        
        function ax=getFigure(obj)
            %get the axis
            ax=obj.mainFig;
        end
        
        function setOrigin(obj,origin)
            %set the origin
            obj.currentWorkSpace.origin=origin;
        end
        
        function zoomOut(obj)
            %zoom out
            workspace=obj.currentWorkSpace;
            obj.currentWorkSpace=WorkSpace(workspace.sizeX*2,workspace.sizeY*2,workspace.unitLength,workspace.unitForce);
        end
        
        function zoomIn(obj)
            %zoom in
            workspace=obj.currentWorkSpace;
            obj.currentWorkSpace=WorkSpace(workspace.sizeX/2,workspace.sizeY/2,workspace.unitLength,workspace.unitForce);
        end
               
        function deleteAllSavedData(obj)
            %delete all saved data
            if length(obj.sessionKinematics) > 1
                obj.sessionKinematics(2:end)=[];
            end
            if ~isempty(obj.sessionLoadAnalysis)
                obj.sessionLoadAnalysis=LoadAnalysis.empty;
            end
            if ~isempty(obj.sessionDistanceAnalysis)
                obj.sessionDistanceAnalysis=DistanceAnalysis.empty;
            end
            if ~isempty(obj.sessionAdvantageAnalysis)
                obj.sessionAdvantageAnalysis=Advantage.empty;
            end
            if ~isempty(obj.sessionBistable)
                obj.sessionBistable=BiStable.empty;
            end
        end
        
        function  deleteAll(obj)
            %delete everything before a new module
            %delete rigidlink drawings
            obj.deleteLastAnimation();
            
            %delete node drawings
            for i=1:length(obj.nodes)
                obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
            end
            
            %delete link drawings
            for i=1:length(obj.links)
                obj.links(i)=obj.links(i).deleteLinkDrawing();
            end
            
            %delete all drawings
            for i=1:length(obj.torsionSprings)
                obj.torsionSprings(i)=obj.torsionSprings(i).deleteDrawing();
            end
            
            %delete all drawings
            for i=1:length(obj.forces)
                obj.forces(i)=obj.forces(i).deleteDrawing();
            end
            
            %delete all drawings
            for i=1:length(obj.moments)
                obj.moments(i)=obj.moments(i).deleteDrawing();
            end
            
            %delete prb nodes
            obj.deletePRBNodes();
        end
        
        function plotEverything(obj)
            %Plot everything
            handles=obj.getHandles();
            %axes(handles.designPlot);
            handles.mainGUI.Name=['DAS-2D v0.8 - ',[obj.getFileName(),'*']];
            obj.resizeWorkspace();
            limit=obj.currentWorkSpace.getLimit()*0.5;
            obj.deletePRBNodes();
            switch(obj.mode)
                case Module.Overview
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(obj.mainFig,obj.nodes,limit,obj.mode,obj);
                    end
                    for i=1:length(obj.nodes)
                            %plot nodes
                            obj.nodes(i)=obj.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                    end
                    %plot torsion springs
                    for i=1:length(obj.torsionSprings)
                        obj.torsionSprings(i)=obj.torsionSprings(i).draw(obj);
                    end 
                    %plot forces
                    for i=1:length(obj.forces)
                        obj.forces(i)=obj.forces(i).draw(obj);
                    end
                    %plot moments
                    for i=1:length(obj.moments)
                        obj.moments(i)=obj.moments(i).draw(obj);
                    end
                    obj.setStatus();
                    %if compliant fill the nodes
                    try
                        for i=1:length(obj.links)
                            if ~isempty(obj.links(i).geometry)
                                node1=obj.nodes(obj.links(i).getNode(1)).getNode();
                                node2=obj.nodes(obj.links(i).getNode(2)).getNode();
                                obj.links(i).geometry=obj.links(i).geometry.fillNodes(node1,node2,obj.currentWorkSpace,obj.checkReverse(i));
                            end
                        end
                        obj.plotPRBNodes(limit);
                    catch err
                        disp(err);
                        obj.deletePRBNodes();
                    end
                case {Module.Sketch,Module.Model,Module.Constrain}
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(obj.mainFig,obj.nodes,limit,obj.mode,obj);
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                    end
                    obj.setStatus();
                case Module.Linear
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(obj.mainFig,obj.nodes,limit,obj.mode,obj);
                    end
                    for i=1:length(obj.nodes)
                            %plot nodes
                            obj.nodes(i)=obj.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                    end
                    obj.setStatus();
                case Module.Compliant
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                    end
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(obj.mainFig,obj.nodes,limit,obj.mode,obj);
                    end
                    obj.setStatus();
                    %if compliant fill the nodes
                    try
                        for i=1:length(obj.links)
                            if ~isempty(obj.links(i).geometry)
                                node1=obj.nodes(obj.links(i).getNode(1)).getNode();
                                node2=obj.nodes(obj.links(i).getNode(2)).getNode();
                                obj.links(i).geometry=obj.links(i).geometry.fillNodes(node1,node2,obj.currentWorkSpace,obj.checkReverse(i));
                            end
                        end
                        obj.plotPRBNodes(limit);
                    catch err
                        disp(err);
                        obj.deletePRBNodes();
                    end
                case Module.Torsion
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                    end
                    
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(obj.mainFig,obj.nodes,limit,obj.mode,obj);
                    end
                    %plot torsion springs
                    for i=1:length(obj.torsionSprings)
                        obj.torsionSprings(i)=obj.torsionSprings(i).draw(obj);
                    end
                    obj.setStatus();
                case Module.Force
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(obj.mainFig,obj.nodes,limit,obj.mode,obj);
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                    end
                    %plot forces
                    for i=1:length(obj.forces)
                        obj.forces(i)=obj.forces(i).draw(obj);
                    end
                    obj.setStatus();
                case Module.Moment
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(obj.mainFig,obj.nodes,limit,obj.mode,obj);
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                    end
                    %plot moments
                    for i=1:length(obj.moments)
                        obj.moments(i)=obj.moments(i).draw(obj);
                    end
                    obj.setStatus();
                case {Module.Free,Module.Range}
                    %plot links
                    obj.sessionKinematics(end)=obj.sessionKinematics(end).drawAll(obj.mainFig,limit,obj.mode,obj);
                case Module.Load
                    %plot links
                    obj.sessionLoadAnalysis(end)=obj.sessionLoadAnalysis(end).drawAll(obj.mainFig,limit,obj.mode,obj,100);
                case Module.Distance
                    %plot links
                    obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).drawAll(obj.mainFig,limit,obj.mode,obj);
                    %plot nodes
                    if ~logical(obj.sessionDistanceAnalysis(end).busy)
                        for i=1:length(obj.sessionDistanceAnalysis(end).static.nodes)
                            %plot nodes
                            obj.sessionDistanceAnalysis(end).static.nodes(i)=obj.sessionDistanceAnalysis(end).static.nodes(i).drawNode(handles.designPlot,limit,obj.mode,obj);
                            obj.paintInputLoad();
                        end
                    end
                case Module.MechanicalAdvantage
                    obj.sessionAdvantageAnalysis(end)=obj.sessionAdvantageAnalysis(end).drawAll(obj.mainFig,limit,obj.mode,obj);
                    if ~logical(obj.sessionAdvantageAnalysis(end).busy)
                        obj.paintAdvantage();
                    end
                case Module.EnergyPlot
                    %plot links
                    obj.sessionBistable(end)=obj.sessionBistable(end).drawAll(obj.mainFig,limit,obj.mode,obj);
                    %plot nodes
                    if ~logical(obj.sessionBistable(end).busy)
                        for i=1:length(obj.sessionBistable(end).static.nodes)
                            %plot nodes
                            obj.sessionBistable(end).static.nodes(i)=obj.sessionBistable(end).static.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                        end
                    end
                case Module.Other
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(obj.mainFig,obj.nodes,limit,obj.mode,obj);
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                    end
                    %plot forces
                    for i=1:length(obj.forces)
                        obj.forces(i)=obj.forces(i).draw(obj);
                    end
                    %plot moments
                    for i=1:length(obj.moments)
                        obj.moments(i)=obj.moments(i).draw(obj);
                    end
                    %plot torsion springs
                    for i=1:length(obj.torsionSprings)
                        obj.torsionSprings(i)=obj.torsionSprings(i).draw(obj);
                    end
                case Module.KinematicSynthesis
                    %plot links
                    %                     obj.currentWorkSpace=obj.sessionKinematicSynthesis(end).changeWorkspace(obj);
                    %                     obj.resizeAxis();
                    %                     limit=obj.currentWorkSpace.getLimit()*0.5;
                case Module.FlexuralStiffnessSynthesis
                    %plot links
                    obj.sessionFlexuralStiffnessSynthesis(end)=obj.sessionFlexuralStiffnessSynthesis(end).drawAll(obj.mainFig,limit,obj.mode,obj);
                    %plot nodes
                    if ~logical(obj.getBusy())
                        for i=1:length(obj.sessionFlexuralStiffnessSynthesis(end).static.nodes)
                            %plot nodes
                            obj.sessionFlexuralStiffnessSynthesis(end).static.nodes(i)=obj.sessionFlexuralStiffnessSynthesis(end).static.nodes(i).drawNode(obj.mainFig,limit,obj.mode,obj);
                        end
                    end
                case Module.BiStableSynthesis
                    %plot links
                    obj.sessionBiStableSynthesis(end)=obj.sessionBiStableSynthesis(end).drawAll(handles.designPlot,limit,obj.mode,obj);
                    %plot nodes
                    if ~logical(obj.getBusy())
                        for i=1:length(obj.sessionBiStableSynthesis(end).bistable.static.nodes)
                            %plot nodes
                            obj.sessionBiStableSynthesis(end).bistable.static.nodes(i)=obj.sessionBiStableSynthesis(end).bistable.static.nodes(i).drawNode(handles.designPlot,limit,obj.mode,obj);
                        end
                    end
            end
        end
        
        function plotPRBNodes(obj,limit)
            %plot the prb nodes
            for i=1:length(obj.links)
                if ~isempty(obj.links(i).geometry)
                    obj.links(i).geometry=obj.links(i).geometry.drawNodes(obj.mainFig,limit,obj);
                end
            end
        end
        
        function deletePRBNodes(obj)
            %delete the prb nodes
            for i=1:length(obj.links)
                if ~isempty(obj.links(i).geometry)
                    obj.links(i).geometry=obj.links(i).geometry.deleteNodes();
                end
            end
        end
        
        function deleteLastAnimation(obj)
            %delete plots from last animation
            obj.sessionKinematics(end)=obj.sessionKinematics(end).deleteAll();
            
            if ~isempty(obj.sessionLoadAnalysis)
                obj.sessionLoadAnalysis(end)=obj.sessionLoadAnalysis(end).deleteAll();
            end
            if ~isempty(obj.sessionDistanceAnalysis)
                obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).deleteAll();
            end
            if ~isempty(obj.sessionAdvantageAnalysis)
                obj.sessionAdvantageAnalysis(end)=obj.sessionAdvantageAnalysis(end).deleteAll();
            end
            
            if ~isempty(obj.sessionBistable)
                obj.sessionBistable(end)=obj.sessionBistable(end).deleteAll();
            end
            
            if ~isempty(obj.sessionFlexuralStiffnessSynthesis)
                obj.sessionFlexuralStiffnessSynthesis(end)=obj.sessionFlexuralStiffnessSynthesis(end).deleteAll();
            end
            
            if ~isempty(obj.sessionBiStableSynthesis)
                obj.sessionBiStableSynthesis(end)=obj.sessionBiStableSynthesis(end).deleteAll(obj);
            end
            
        end
        
        
         
        function colorNode(obj,color,id)
            %color selected Node
            obj.nodes(id)=obj.nodes(id).colorNode(color);
        end
        
        function setJoint(obj,linkID,select,joint)
            %set joint of a link
            obj.links(linkID)=obj.links(linkID).setJoint(select,joint);
        end
        
        function setSliderAngle(obj,linkID,value)
            %set angle of a link
            obj.links(linkID)=obj.links(linkID).setAngleSlider(value);
        end
        
        function setJoints(obj,node,joint,linkID)
            %if node is not apoointed a joint appoint it
            %if joint == Joint.GroundPin || joint == Joint.Slider || joint == Joint.GroundSlider
            if  joint == Joint.Slider || joint == Joint.GroundSlider
                joint=Joint.Pin;
                %elseif joint == Joint.GroundWelded
                %    joint=Joint.Welded;
            end
            
            for i=1:length(obj.links)
                if i ~= linkID
                    pos=find(obj.links(i).getNodes()==node);
                    if ~isempty(pos) %&& obj.links(i).getJoint(pos) == Joint.NA
                        obj.links(i)=obj.links(i).setJoint(pos,joint);
                        obj.checkStatus(i);
                    end
                end
            end
            obj.plotEverything();
        end
        
        function checkStatus(obj,id)
            %check status of the selected link
            %check if it is complete
            handles=obj.getHandles();
            link=obj.links(id);
            joint1=link.getJoint(1);
            joint2=link.getJoint(2);
            sliderAngle=link.getAngleSlider();
            if  (joint1 == Joint.GroundSlider || (joint2== Joint.GroundSlider) ) && isempty(sliderAngle)
                handles.selectedLinkStatus2.String='incomplete';
                obj.links(id)=obj.links(id).setStatus(-1);
                handles.selectedLinkStatus2.ForegroundColor=Colors.StatusIncomplete.getColor();
            elseif joint1 == Joint.NA || (joint2== Joint.NA)
                if logical(link.getGroundLink())
                    handles.selectedLinkStatus2.String='valid';
                    obj.links(id)=obj.links(id).setStatus(1);
                    handles.selectedLinkStatus2.ForegroundColor=Colors.StatusComplete.getColor();
                else
                    handles.selectedLinkStatus2.String='incomplete';
                    obj.links(id)=obj.links(id).setStatus(-1);
                    handles.selectedLinkStatus2.ForegroundColor=Colors.StatusIncomplete.getColor();
                end
            elseif Helper.checkLegitLink([joint1 joint2])
                handles.selectedLinkStatus2.String='valid';
                obj.links(id)=obj.links(id).setStatus(1);
                handles.selectedLinkStatus2.ForegroundColor=Colors.StatusComplete.getColor();
            else
                handles.selectedLinkStatus2.String='invalid';
                obj.links(id)=obj.links(id).setStatus(0);
                handles.selectedLinkStatus2.ForegroundColor=Colors.StatusIncomplete.getColor();
            end
            %check if it can be still a linear spring
            if logical(obj.links(id).getLinearSpring()) && ~Helper.checkLegitLinearSpring([joint1 joint2])
                obj.links(id)=obj.links(id).deleteLinearSpring();
            end
            %check whole mechanism status
            obj.setStatus();
        end      
          
        
        function checkSingleNodes(obj)
            %check single nodes
            allNodes=zeros(length(obj.links),2);
            for i=1:length(obj.links)
                allNodes(i,:)=obj.links(i).getNodes();
            end
            nodes2Deleted=[];
            for i=1:length(obj.nodes)
                if isempty(find(allNodes==i, 1))
                    nodes2Deleted(end+1)=i;
                end
            end
            nodes2Deleted=sort(nodes2Deleted,'descend');
            for i=1:length(nodes2Deleted)
                obj.deleteNode([],[],nodes2Deleted(i));
            end
        end
        
        function fillInitialJoints(obj)
            %fill the joints after sketch
            allNodes=zeros(length(obj.links),2);
            for i=1:length(obj.links)
                allNodes(i,:)=obj.links(i).getNodes();
            end
            for i=1:length(obj.links)
                nodes=obj.links(i).getNodes();
                joints=obj.links(i).getJoints();
                node1=find(allNodes == nodes(1,1));
                node2=find(allNodes == nodes(1,2));
                if length(node1) == 1 && joints(1) == Joint.NA
                    %ground node
                    obj.links(i)=obj.links(i).setJoint(1,Joint.GroundPin);
                    obj.checkStatus(i);
                elseif joints(1) == Joint.NA
                    %intermediate node
                    obj.links(i)=obj.links(i).setJoint(1,Joint.Pin);
                    obj.checkStatus(i);
                end
                if length(node2) == 1 && joints(2) == Joint.NA
                    %ground node
                    obj.links(i)=obj.links(i).setJoint(2,Joint.GroundPin);
                    obj.checkStatus(i);
                elseif joints(2) == Joint.NA
                    %intermediate node
                    obj.links(i)=obj.links(i).setJoint(2,Joint.Pin);
                    obj.checkStatus(i);
                end
            end
        end
            
        function setStatusLink(obj,linkID,status)
            %set status of a link
            obj.links(linkID)=obj.links(linkID).setStatus(status);
        end
            
        function linearSpring=getLinearSpring(obj,linkID)
            %get linear spring status of a link
            linearSpring=obj.links(linkID).getLinearSpring();
        end
        
        function deleteLinkLinearSpring(obj,linkID)
            %delete linear spring
            obj.links(linkID)=obj.links(linkID).deleteLinearSpring();
            %tree
            obj.fillNodeTree();
            obj.fillLinkTree();
            obj.fillLinearSpringTree();
            obj.fillTorsionTree();
            obj.fillForceTree();
            obj.fillMomentTree();
            handles=obj.getHandles();
            handles.tree.collapse(handles.nodesTree);
            handles.tree.collapse(handles.linksTree);
            handles.tree.collapse(handles.linearSpringTree);
            handles.tree.collapse(handles.torsionsTree);
            handles.tree.collapse(handles.forcesTree);
            handles.tree.collapse(handles.momentsTree);
        end
        
        function addLinearSpring(obj,stiffness,node1,node2)
            %add a linear spring
            if stiffness == 0
                stiffness=1e-7;
            end
            %check if there is a existing linkbetween two nodes
            for i=1:length(obj.links)
                nodes=obj.links(i).getNodes();
                if length(union(nodes,[node1 node2])) == 2
                    id=i;
                    if logical(Helper.checkLegitLinearSpring(obj.links(id).getJoints()))
                        obj.links(id)=obj.links(id).setLinearSpring(stiffness);
                        obj.links(id).selected=1;
                        obj.setStatus();
                        obj.plotEverything();
                        return;
                    end
                end
            end
            %add a new link
            obj.links(end+1)=Link(length(obj.links)+1,[node1 node2]);
            obj.links(end)=obj.links(end).setJoints([Joint.Pin Joint.Pin]);
            obj.links(end)=obj.links(end).setLinearSpring(stiffness);
            obj.links(end).selected=1;
            obj.checkStatus(length(obj.links));
            obj.setStatus();
            obj.plotEverything();
        end
        
        function setLinearSpring(obj,stiffness)
            %set a linear spring
            if stiffness == 0
                stiffness=1e-7;
            end
            id=obj.getOnlySelectedLink();
            if logical(id) && logical(Helper.checkLegitLinearSpring(obj.links(id).getJoints()))
                obj.links(id)=obj.links(id).setLinearSpring(stiffness);
                obj.setStatus();
                obj.plotEverything();
            end
        end
        
        function deleteLinearSpring(obj)
            %convert a spring into a link
            id=obj.getOnlySelectedLink();
            if logical(id)
                obj.links(id)=obj.links(id).deleteLinearSpring();
                obj.plotEverything();
            end
        end
        
        function deleteTorsionSpringButton(obj)
            %delete torsion spring
            for i=1:length(obj.torsionSprings)
                if logical(obj.torsionSprings(i).getSelected())
                    obj.deleteTorsionSpring([],[],i);
                    break;
                end
            end
            
        end
        
        function deleteForceButton(obj)
            %delete selected force
            for i=1:length(obj.forces)
                if logical(obj.forces(i).getSelected())
                    obj.deleteForce([],[],i);
                    break;
                end
            end 
        end
        
        function deleteMomentButton(obj)
            %delete selected moment
            for i=1:length(obj.moments)
                if logical(obj.moments(i).getSelected())
                    obj.deleteMoment([],[],i);
                    break;
                end
            end
            
        end
        

        function contraints=getLinkConstraints(obj,id)
            %get link constraints
            contraints=obj.links(id).getConstraints();
        end
                
        function constrain(obj,linkID,value,type,direction)
            %optimization function for adding constraint
            %check if a constraint exists
            link=obj.links(linkID);
            constraintsLink=link.getConstraints();
            constrainNodes=0;
            for i=1:length(constraintsLink)
                currentConstraint=obj.constraints(constraintsLink(i));
                if currentConstraint.getType == type
                    %same type
                    %check if same value
                    if currentConstraint.getValue() == value
                        return;
                    else
                        %different so delete the constraint
                        if currentConstraint.getType == ConstraintType.Length
                            obj.deleteConstraintCheck(linkID,1);
                        else
                            obj.deleteConstraintCheck(linkID,2);
                        end
                    end
                    %remove constraints on nodes
                    linkNodes=obj.links(linkID).nodes;
                    obj.nodes(linkNodes(1))=obj.nodes(linkNodes(1)).setConstrained(0,1);
                    obj.nodes(linkNodes(1))=obj.nodes(linkNodes(1)).setConstrained(0,2);
                    obj.nodes(linkNodes(2))=obj.nodes(linkNodes(2)).setConstrained(0,1);
                    obj.nodes(linkNodes(2))=obj.nodes(linkNodes(2)).setConstrained(0,2);
                else
                    %fix the nodes since two constraints are given.
                    constrainNodes=1;
                end
            end
            
            handles=obj.getHandles();
            switch type
                case ConstraintType.Length
                    %length type
                    link=obj.links(linkID);
                    node1=obj.nodes(link.getNode(1)).getNode();
                    node2=obj.nodes(link.getNode(2)).getNode();
                    current=sqrt((node1.x-node2.x)^2+(node1.y-node2.y)^2 );
                case ConstraintType.AngleX
                    %angle with the x axis
                    link=obj.links(linkID);
                    node1=obj.nodes(link.getNode(1)).getNode();
                    node2=obj.nodes(link.getNode(2)).getNode();
                    vector1=[(node2.x-node1.x),node2.y-node1.y];
                    vector2=[1 0];
                    current=Helper.angleBetween2Vectors(vector2,vector1,1,1);
                case ConstraintType.AngleY
                    %angle with the y axis
                    link=obj.links(linkID);
                    node1=obj.nodes(link.getNode(1)).getNode();
                    node2=obj.nodes(link.getNode(2)).getNode();
                    vector1=[(node2.x-node1.x),node2.y-node1.y];
                    vector2=[0 1];
                    current=Helper.angleBetween2Vectors(vector2,vector1,1,1);
                case ConstraintType.AngleLink
                    %angle between two links
                    link=obj.links(linkID(1));
                    otherBeam=obj.links(linkID(2));
                    %find which node they share
                    if link.getNode(1)==otherBeam.getNode(1)
                        beamNode=1;
                        otherBeamNode=1;
                    elseif link.getNode(1)==otherBeam.getNode(2)
                        beamNode=1;
                        otherBeamNode=2;
                    elseif link.getNode(2)==otherBeam.getNode(1)
                        beamNode=2;
                        otherBeamNode=1;
                    elseif link.getNode(2)==otherBeam.getNode(2)
                        beamNode=2;
                        otherBeamNode=2;
                    end
                    %draw pointers for two links
                    node1=obj.nodes(link.getNode(1)).getNode();
                    node2=obj.nodes(link.getNode(2)).getNode();
                    if beamNode ==1
                        vector1=[node2.x-node1.x,node2.y-node1.y];
                    else
                        vector1=[node1.x-node2.x,node1.y-node2.y];
                    end
                    %link 2
                    node1=obj.nodes(otherBeam.getNode(1)).getNode();
                    node2=obj.nodes(otherBeam.getNode(2)).getNode();
                    if otherBeamNode ==1
                        vector2=[node2.x-node1.x,node2.y-node1.y];
                    else
                        vector2=[node1.x-node2.x,node1.y-node2.y];
                    end
                    
                    current=Helper.angleBetween2Vectors(vector1,vector2,1,1);
            end
            count=5;
            lengthArray=linspace(current,value,count) ;
            %for initial guess
            x=[];
            for i=1:length(obj.nodes)
                if ~logical(obj.nodes(i).getConstrained(1))
                    x(end+1)=obj.nodes(i).getNode().x;
                end
                if ~logical(obj.nodes(i).getConstrained(2))
                    x(end+1)=obj.nodes(i).getNode().y;
                end
            end
            %status bar
            handles.helpStatic.String='Running...';
           % drawnow;
            for i=1:count
                %main optimization
                if i ~= 1
                    %delete the constraint
                    obj.deleteLastConstraint();
                end
                obj.addConstraint(linkID,lengthArray(i),type,direction);
                %initial guess for the optimization
                initialGuess=x;
%                lengthArray(i)
%                initialGuess(end-1)=-15.62355520187725;
                fnc=@(x)obj.constraintMain( x );
                toDelete=[];
                %check for duplicate
                for j=1:length(obj.constraints)-1
                    if obj.constraints(j).link == obj.constraints(end).link ...
                            && obj.constraints(j).type == obj.constraints(end).type
                        toDelete(end+1)=j;
                    end
                end
                if ~isempty(toDelete)
                    obj.constraints(toDelete)=[];
                end

                opt=optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter-detailed','MaxFunEvals',1e3,...
                    'MaxIter',1e3,'TolFun',1e-8,'TolX',1e-8,'StepTolerance',1e-8,'FunctionTolerance',1e-8);
                [x,resnorm,residual,exitflag,output] =lsqnonlin(fnc,initialGuess,[],[],opt) ;
            end
            if  exitflag > 0 && resnorm < 1e-06
                %update nodes
                index=1;
                for j=1:length(obj.nodes)
                    if ~logical(obj.nodes(j).getConstrained(1))
                        xx=x(index);
                        index=index+1;
                    else
                        xx=obj.nodes(j).getNode().x;
                    end
                    if ~logical(obj.nodes(j).getConstrained(2))
                        yy=x(index);
                        index=index+1;
                    else
                        yy=obj.nodes(j).getNode().y;
                    end
                    obj.nodes(j)=obj.nodes(j).setNode(Point(xx,yy));
                end
                %add constraint
                if type == ConstraintType.AngleLink
                    obj.links(linkID(1))=obj.links(linkID(1)).addConstraint(length(obj.constraints));
                    obj.links(linkID(2))=obj.links(linkID(2)).addConstraint(length(obj.constraints));
                else
                    obj.links(linkID)=obj.links(linkID).addConstraint(length(obj.constraints));
                end
                %gui
                handles.helpStatic.String='Constraint is added';
                handles.data.setDraggable(0);
               % if logical(constrainNodes)
               %     linkNodes=obj.links(linkID).nodes;
               %     obj.nodes(linkNodes(1))=obj.nodes(linkNodes(1)).setConstrained(1,1);
               %     obj.nodes(linkNodes(1))=obj.nodes(linkNodes(1)).setConstrained(1,2);
               %     obj.nodes(linkNodes(2))=obj.nodes(linkNodes(2)).setConstrained(1,1);
               %     obj.nodes(linkNodes(2))=obj.nodes(linkNodes(2)).setConstrained(1,2);
               % end
            else
                warndlg('! Can not add the constraint. Please visit the help page. !','Warning');
                obj.deleteLastConstraint();
                handles.helpStatic.String='Can not add constraint';
            end
            
            
        end
        
        function f= constraintMain(obj,x)
            %main function for the optimization for constraint adding
            f=zeros(1,length(obj.constraints));
            points=Point.empty;
            index=1;
            for j=1:length(obj.nodes)
                if ~logical(obj.nodes(j).getConstrained(1))
                    xx=x(index);
                    index=index+1;
                else
                    xx=obj.nodes(j).getNode().x;
                end
                if ~logical(obj.nodes(j).getConstrained(2))
                    yy=x(index);
                    index=index+1;
                else
                    yy=obj.nodes(j).getNode().y;
                end
                points(j)=Point(xx,yy);
            end
            limit=obj.currentWorkSpace.getLimit();
            for i=1:length(obj.constraints)
                switch obj.constraints(i).getType()
                    case ConstraintType.Length
                        %length type
                        link=obj.links(obj.constraints(i).getLink(1));
                        value=obj.constraints(i).getValue();
                        node1=points(link.getNode(1));
                        node2=points(link.getNode(2));
                        currentLength=sqrt((node1.x-node2.x)^2+(node1.y-node2.y)^2 );
                        % f(i)=currentLength-value;
                        f(i)=(currentLength-value)/limit;
                    case ConstraintType.AngleX
                        %angle with the x axis
                        link=obj.links(obj.constraints(i).getLink(1));
                        value=obj.constraints(i).getValue();
                        node1=points(link.getNode(1));
                        node2=points(link.getNode(2));
                        vector1=[(node2.x-node1.x),node2.y-node1.y];
                        vector2=[1 0];
                        currentAngle=Helper.angleBetween2Vectors(vector2,vector1,1,1);
                        f(i)=(currentAngle-value)/360;
                    case ConstraintType.AngleY
                        %angle with the x axis
                        link=obj.links(obj.constraints(i).getLink(1));
                        value=formatAngle(obj.constraints(i).getValue());
                        node1=points(link.getNode(1));
                        node2=points(link.getNode(2));
                        vector1=[(node2.x-node1.x),node2.y-node1.y];
                        vector2=[0 1];
                        currentAngle=Helper.angleBetween2Vectors(vector2,vector1,1,1);
                        f(i)=(currentAngle-value)/360;
                    case ConstraintType.AngleLink
                        %angle between two links
                        link=obj.links(obj.constraints(i).getLink(1));
                        otherBeam=obj.links(obj.constraints(i).getLink(2));
                        value=obj.constraints(i).getValue();
                        %find which node they share
                        if link.getNode(1)==otherBeam.getNode(1)
                            beamNode=1;
                            otherBeamNode=1;
                        elseif link.getNode(1)==otherBeam.getNode(2)
                            beamNode=1;
                            otherBeamNode=2;
                        elseif link.getNode(2)==otherBeam.getNode(1)
                            beamNode=2;
                            otherBeamNode=1;
                        elseif link.getNode(2)==otherBeam.getNode(2)
                            beamNode=2;
                            otherBeamNode=2;
                        end
                        %draw pointers for two links
                        node1=points(link.getNode(1));
                        node2=points(link.getNode(2));
                        if beamNode ==1
                            vector1=[node2.x-node1.x,node2.y-node1.y];
                        else
                            vector1=[node1.x-node2.x,node1.y-node2.y];
                        end
                        %link 2
                        node1=points(otherBeam.getNode(1));
                        node2=points(otherBeam.getNode(2));
                        if otherBeamNode ==1
                            vector2=[node2.x-node1.x,node2.y-node1.y];
                        else
                            vector2=[node1.x-node2.x,node1.y-node2.y];
                        end
                        
                        currentAngle=Helper.angleBetween2Vectors(vector1,vector2,1,1);
                        f(i)=(currentAngle-value)/360;
                end
            end
        end
        
        function deleteConstraintCheck(obj,linkID,type)
            %delete a link constraint angle or length
            constraintsLink=obj.links(linkID).getConstraints();
            %delete node constraints
            linkNodes=obj.links(linkID).nodes;
            obj.nodes(linkNodes(1))=obj.nodes(linkNodes(1)).setConstrained(0,1);
            obj.nodes(linkNodes(1))=obj.nodes(linkNodes(1)).setConstrained(0,2);
            obj.nodes(linkNodes(2))=obj.nodes(linkNodes(2)).setConstrained(0,1);
            obj.nodes(linkNodes(2))=obj.nodes(linkNodes(2)).setConstrained(0,2);
            if type == 1
                %length
                for i=1:length(constraintsLink)
                    if obj.constraints(constraintsLink(i)).getType() == ConstraintType.Length
                        obj.deleteConstraintMain(constraintsLink(i));
                        break;
                    end
                end
            else
                %angle
                for i=1:length(constraintsLink)
                    if obj.constraints(constraintsLink(i)).getType() ~= ConstraintType.Length
                        obj.deleteConstraintMain(constraintsLink(i));
                        break;
                    end
                end
            end
        end
        
        function deleteConstraintMain(obj,id)
            %delete a constraint
            %adjust the other constraint ids
            for i=1:length(obj.constraints)
                if obj.constraints(i).getID > id
                    obj.constraints(i)= obj.constraints(i).setID(obj.constraints(i).getID-1);
                end
            end
            %adjust link constraints
            for i=1:length(obj.links)
                constraintsLink=obj.links(i).getConstraints();
                delete=0;
                for j=1:length(constraintsLink)
                    if constraintsLink(j) > id
                        constraintsLink(j)=constraintsLink(j)-1;
                    elseif constraintsLink(j)== id
                        delete=j;
                    end
                end
                if logical(delete)
                    constraintsLink(delete)=[];
                end
                obj.links(i)=obj.links(i).setConstraints(constraintsLink);
            end
            %delete constraint
            id=sort(id,'descend') ;
            obj.constraints(id)=[];
        end
        
        function deleteConstraint(obj,source,callbackdata,id)
            %delete a constraint
            obj.deleteConstraintMain(id);
            obj.plotEverything();
            handles.helpStatic.String='Constraint is deleted';
        end
        
        
        function clearFreeInput(obj)
            %clear the free input
            obj.freeKinematics.mode=0;
        end
        
        function setFreeInput(obj,point)
            %set free kinematics input
            link=obj.sessionKinematics(end).allBeams{obj.freeKinematics.link};
            nodes=[obj.sessionKinematics(end).nodes(link.nodes(1)).getNode(),obj.sessionKinematics(end).nodes(link.nodes(2)).getNode()];
            switch(obj.freeKinematics.mode)
                case 1
                    %one degree of freedom - angle
                    newAngle=atan2(point.y-nodes(obj.freeKinematics.node).y,point.x-nodes(obj.freeKinematics.node).x);
                    obj.sessionKinematics(end).inputs(1)=struct('linkID',obj.freeKinematics.link,'inputType',1,'target',newAngle,'isRelative',0,'initialValue',0,'type','angle');
                case 2
                    %one degree of freedom - length
                    newLength=sqrt((point.y-nodes(obj.freeKinematics.node).y)^2+(point.x-nodes(obj.freeKinematics.node).x)^2);
                    obj.sessionKinematics(end).inputs(1)=struct('linkID',obj.freeKinematics.link,'inputType',1,'target',newLength,'isRelative',0,'initialValue',0,'type','length');
                case 3
                    %two degree of freedom - angle length
                    newAngle=atan2(point.y-nodes(obj.freeKinematics.node).y,point.x-nodes(obj.freeKinematics.node).x);
                    %newLength=sqrt((point.y-nodes(obj.freeKinematics.node).y)^2+(point.x-nodes(obj.freeKinematics.node).x)^2);
                    obj.sessionKinematics(end).inputs(1)=struct('linkID',obj.freeKinematics.link,'inputType',1,'target',newAngle,'isRelative',0,'initialValue',0,'type','angle');
                    if length(obj.sessionKinematics(end).inputs) > 1
                        %   obj.sessionKinematics(end).inputs(2)=struct('linkID',obj.freeKinematics.link,'inputType',2,'target',newLength,'isRelative',0,'initialValue',0,'type',[]);
                    end
                case 4
                    %one degree of freedom - slider
                    theta=link.sliderAngle;
                    newLength=sqrt((point.y-nodes(obj.freeKinematics.node).y)^2+(point.x-nodes(obj.freeKinematics.node).x)^2);
                    angle=atan2((point.y-nodes(obj.freeKinematics.node).y),(point.x-nodes(obj.freeKinematics.node).x));
                    newLength=(newLength*cos(angle))*cos(theta)+(newLength*sin(angle))*sin(theta);
                    obj.sessionKinematics(end).inputs(1)=struct('linkID',obj.freeKinematics.link,'inputType',1,'target',newLength,'isRelative',0,'initialValue',0,'type','length');
            end
            
        end
        
                
        function deleteRangeInput(obj,index1)
            %delete d range input
            obj.sessionKinematics(end).inputs(index1)=struct('linkID',[],'inputType',[],'target',0,'isRelative',0,'initialValue',0,'type',0);
        end
        
        
        function fillForceTree(obj)
            %fill the force tree
            handles=obj.getHandles();
            handles.forcesTree.removeAllChildren();
            for i=1:length(obj.forces)
                handles.forcesTree.add(uitreenode('v0', ['Force ' num2str(i)], ['Force ' num2str(i)], [], false));
                handles.forcesTree.getLastChild().setIcon(im2java(imread('forceTreeBlue.jpg')));
                if obj.isDesignModule()
                    handles.forcesTree.getLastChild.add(uitreenode('v0', 'Edit', 'Edit', [], true));
                    handles.forcesTree.getLastChild().getLastChild().setIcon(im2java(imread('editTree.jpg')));
                    handles.forcesTree.getLastChild.add(uitreenode('v0', 'Delete', 'Delete', [], true));
                    handles.forcesTree.getLastChild().getLastChild().setIcon(im2java(imread('deleteTree.jpg')));
                end
            end
            handles.tree.reloadNode(handles.forcesTree);
            handles.tree.expand(handles.forcesTree);
        end
        
        function addForce(obj,xMagnitude,yMagnitude,distance,follower,type)
            %add a force
            link=obj.getOnlySelectedLink();
            if logical(link)
                if ~logical(type)
                    obj.forces(end+1)=Force(length(obj.forces)+1,link,xMagnitude,yMagnitude,xMagnitude,yMagnitude,distance,follower);
                    id=length(obj.forces);
                else
                    id=obj.updateSelectedForce(xMagnitude,yMagnitude,xMagnitude,yMagnitude,distance,follower);
                end
                obj.clickForce(id);
                obj.plotEverything();
                obj.fillForceTree();
            else
                warndlg('! Did not select a link !','Warning');
            end
        end
        
        function fillMomentTree(obj)
            %fill the moment tree
            handles=obj.getHandles();
            handles.momentsTree.removeAllChildren();
            for i=1:length(obj.moments)
                handles.momentsTree.add(uitreenode('v0', ['Moment ' num2str(i)], ['Moment ' num2str(i)], [], false));
                handles.momentsTree.getLastChild().setIcon(im2java(imread('momentTreeBlue.jpg')));
                if obj.isDesignModule()
                    handles.momentsTree.getLastChild.add(uitreenode('v0', 'Edit', 'Edit', [], true));
                    handles.momentsTree.getLastChild().getLastChild().setIcon(im2java(imread('editTree.jpg')));
                    handles.momentsTree.getLastChild.add(uitreenode('v0', 'Delete', 'Delete', [], true));
                    handles.momentsTree.getLastChild().getLastChild().setIcon(im2java(imread('deleteTree.jpg')));
                end
            end
            handles.tree.reloadNode(handles.momentsTree);
            handles.tree.expand(handles.momentsTree);
        end
        
        function addMoment(obj,magnitude,distance,type)
            %add a moment
            link=obj.getOnlySelectedLink();
            if logical(link)
                if ~logical(type)
                    if logical(link)
                        obj.moments(end+1)=Moment(length(obj.moments)+1,link,magnitude,distance);
                        id=length(obj.moments);
                    end
                else
                    id=obj.updateSelectedMoment(magnitude,distance);
                end
                obj.clickMoment(id);
                obj.plotEverything();
                obj.fillMomentTree();
            else
                warndlg('! Did not select a link !','Warning');
            end
        end
        
        
        function reverse=checkReverse(obj,link)
            %checks if reverse operation is logical
            link=obj.links(link);
            reverse=0;
            if ( link.getJoint(2) == Joint.GroundWelded  || (link.getJoint(1) ~= Joint.GroundWelded && link.getJoint(1) ~= Joint.Welded) && (link.getJoint(2) == Joint.GroundWelded || link.getJoint(2) == Joint.Welded || link.getJoint(2) == Joint.GroundPin ))
                reverse =1;
            end
        end
        
        function makeCompliantMultipleLinks(obj,links,thickness,width,e,type,additional)
            %make multiple links compliant or rigid
            for i=1:length(links)
                link=links(i);
                if ~isempty(obj.links(link).geometry)
                    obj.links(link).geometry=obj.links(link).geometry.deleteNodes();
                end
                %compliant
                obj.links(link)=obj.links(link).makeCompliant(thickness,width,e,type);
                node1=obj.nodes(obj.links(link).getNode(1)).getNode();
                node2=obj.nodes(obj.links(link).getNode(2)).getNode();
                if type == BeamType.PRB
                    obj.links(link).geometry.prbModel=additional{1};
                    obj.links(link).geometry.order=additional{2};
                    obj.links(link).geometry.segments=obj.links(link).geometry.prbModel(:,1);
                else
                    obj.links(link).geometry.segments=additional{1};
                end
                obj.links(link).geometry=obj.links(link).geometry.fillNodes(node1,node2,obj.currentWorkSpace,obj.checkReverse(link));
            end
            handles=obj.getHandles();
            guidata(obj.mainFig,handles);
            obj.fillLinkTree();
        end
        
        function makeCompliant(obj,thickness,width,e,type,additional)
            %make a link compliant or rigid
            link=obj.getOnlySelectedLink();
            if ~isempty(obj.links(link).geometry)
                obj.links(link).geometry=obj.links(link).geometry.deleteNodes();
            end
            %compliant
            obj.links(link)=obj.links(link).makeCompliant(thickness,width,e,type);
            node1=obj.nodes(obj.links(link).getNode(1)).getNode();
            node2=obj.nodes(obj.links(link).getNode(2)).getNode();
            if type == BeamType.PRB
                obj.links(link).geometry.prbModel=additional{1};
                obj.links(link).geometry.order=additional{2};
                obj.links(link).geometry.segments=obj.links(link).geometry.prbModel(:,1);
            else
                obj.links(link).geometry.segments=additional{1};
            end
            obj.links(link).geometry=obj.links(link).geometry.fillNodes(node1,node2,obj.currentWorkSpace,obj.checkReverse(link));
            handles=obj.getHandles();
            guidata(obj.mainFig,handles);
            obj.fillLinkTree();
        end
        
        function makeRigid(obj)
            %make a link rigid
            link=obj.getOnlySelectedLink();
            if ~isempty(obj.links(link).geometry)
                obj.links(link).geometry=obj.links(link).geometry.deleteNodes();
            end
            obj.links(link)=obj.links(link).makeRigid();
            obj.fillLinkTree();
        end
        
        function addKinematics(obj)
            %add a new kinematics
            obj.sessionKinematics(end+1)=Kinematics(obj.links,obj.nodes,obj.currentWorkSpace);
            obj.sessionKinematics(end).inputs(obj.degreesOfFreedom)=struct('linkID',[],'inputType',[],'target',[],'isRelative',0,'initialValue',[],'type',[]);
        end
        
        function addLoadAnalysis(obj)
            %add a new kinematics
            obj.sessionLoadAnalysis(end+1)=LoadAnalysis(obj.links,obj.nodes,obj.forces,obj.moments,obj.torsionSprings,obj.currentWorkSpace);
        end
               
        function updateDistanceAnalysisNewValue(obj,value)
            %update the new values
            obj.sessionDistanceAnalysis(end).newValues=value;
        end
        
        function distance=getDistanceAnalysisType(obj)
            %get distance type
            distance=obj.sessionDistanceAnalysis(end).distance;
        end
        
        function distance=getBistableType(obj)
            %get distance type
            distance=obj.sessionBistable(end).distance;
        end
        
        function paintAdvantage(obj)
            %paint the forces and moments during advantage analysis
            inputLoad=obj.sessionAdvantageAnalysis(end).inputLoad;
            outputLoad=obj.sessionAdvantageAnalysis(end).outputLoad;
            
            for i=1:length(obj.sessionAdvantageAnalysis(end).static.forces)
                if inputLoad(1,1) == 1 && inputLoad(1,2) ==i
                    if isgraphics(obj.sessionAdvantageAnalysis(end).static.forces(i).text(1))
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(1).String='Input';
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(1).Visible='on';
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(1).Color=Colors.Point2.getColor();
                    end
                    if length(obj.sessionAdvantageAnalysis(end).static.forces(i).text) == 2 && isgraphics(obj.sessionAdvantageAnalysis(end).static.forces(i).text(2))
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(2).String='Input';
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(2).Visible='on';
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(2).Color=Colors.Point2.getColor();
                    end
                elseif outputLoad(1,1) == 1 && outputLoad(1,2) ==i
                    if isgraphics(obj.sessionAdvantageAnalysis(end).static.forces(i).text(1))
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(1).String='Output';
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(1).Visible='on';
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(1).Color=Colors.Constrained.getColor();
                    end
                    if length(obj.sessionAdvantageAnalysis(end).static.forces(i).text) == 2 && isgraphics(obj.sessionAdvantageAnalysis(end).static.forces(i).text(2))
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(2).String='Output';
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(2).Visible='on';
                        obj.sessionAdvantageAnalysis(end).static.forces(i).text(2).Color=Colors.Constrained.getColor();
                    end
                else
                    for j=1:length(obj.sessionAdvantageAnalysis(end).static.forces(i).text)
                        if isgraphics(obj.sessionAdvantageAnalysis(end).static.forces(i).text(j))
                            obj.sessionAdvantageAnalysis(end).static.forces(i).text(j).Visible='off';
                        end
                    end
                end
            end
            
            for i=1:length(obj.sessionAdvantageAnalysis(end).static.moments)
                if inputLoad(1,1) == 2 && inputLoad(1,2) ==i
                    obj.sessionAdvantageAnalysis(end).static.moments(i).text.String='Input';
                    obj.sessionAdvantageAnalysis(end).static.moments(i).text.Visible='on';
                    obj.sessionAdvantageAnalysis(end).static.moments(i).text.Color=Colors.Point2.getColor();
                elseif outputLoad(1,1) == 2 && outputLoad(1,2) ==i
                    obj.sessionAdvantageAnalysis(end).static.moments(i).text.String='Output';
                    obj.sessionAdvantageAnalysis(end).static.moments(i).text.Color=Colors.Constrained.getColor();
                else
                    obj.sessionAdvantageAnalysis(end).static.moments(i).text.Visible='off';
                end
            end
            
            if logical(inputLoad(1,2))
                if inputLoad(1,1) == 1
                    %force
                    for i=1:size(obj.sessionAdvantageAnalysis(end).static.forces(inputLoad(1,2)).line,1)
                        for j=1:2
                            if ~isempty(obj.sessionAdvantageAnalysis(end).static.forces(inputLoad(1,2)).line(i,j)) && isgraphics(obj.sessionAdvantageAnalysis(end).static.forces(inputLoad(1,2)).line(i,j))
                                obj.sessionAdvantageAnalysis(end).static.forces(inputLoad(1,2)).line(i,j).Color=Colors.Point2.getColor();
                            end
                        end
                    end
                else
                    %moment
                    for i=1:size(obj.sessionAdvantageAnalysis(end).static.moments(inputLoad(1,2)).line,1)
                        for j=1:2
                            if ~isempty(obj.sessionAdvantageAnalysis(end).static.moments(inputLoad(1,2)).line(i,j)) && isgraphics(obj.sessionAdvantageAnalysis(end).static.moments(inputLoad(1,2)).line(i,j))
                                obj.sessionAdvantageAnalysis(end).static.moments(inputLoad(1,2)).line(i,j).Color=Colors.Point2.getColor();
                            end
                        end
                    end
                end
            end
            if logical(outputLoad(1,2))
                if outputLoad(1,1) == 1
                    %force
                    for i=1:size(obj.sessionAdvantageAnalysis(end).static.forces(outputLoad(1,2)).line,1)
                        for j=1:2
                            if ~isempty(obj.sessionAdvantageAnalysis(end).static.forces(outputLoad(1,2)).line(i,j)) && isgraphics(obj.sessionAdvantageAnalysis(end).static.forces(outputLoad(1,2)).line(i,j))
                                obj.sessionAdvantageAnalysis(end).static.forces(outputLoad(1,2)).line(i,j).Color=Colors.Constrained.getColor();
                            end
                        end
                    end
                else
                    %moment
                    for i=1:size(obj.sessionAdvantageAnalysis(end).static.moments(outputLoad(1,2)).line,1)
                        for j=1:2
                            if ~isempty(obj.sessionAdvantageAnalysis(end).static.moments(outputLoad(1,2)).line(i,j)) && isgraphics(obj.sessionAdvantageAnalysis(end).static.moments(outputLoad(1,2)).line(i,j))
                                obj.sessionAdvantageAnalysis(end).static.moments(outputLoad(1,2)).line(i,j).Color=Colors.Constrained.getColor();
                            end
                        end
                    end
                end
            end
        end
        
        function paintInputLoad(obj)
            %paint the input load
            inputLoad=obj.sessionDistanceAnalysis(end).inputLoad;
            if inputLoad(1,1) == 1
                for i=1:size(obj.sessionDistanceAnalysis(end).static.forces(inputLoad(1,2)).line,1)
                    for j=1:2
                        if ~isempty(obj.sessionDistanceAnalysis(end).static.forces(inputLoad(1,2)).line(i,j)) && isgraphics(obj.sessionDistanceAnalysis(end).static.forces(inputLoad(1,2)).line(i,j))
                            obj.sessionDistanceAnalysis(end).static.forces(inputLoad(1,2)).line(i,j).Color=Colors.Selected.getColor();
                        end
                    end
                end
                
                %no text for distance analysis
                for i=1:length(obj.sessionDistanceAnalysis(end).static.forces(inputLoad(1,2)).text)
                    if ~isempty(obj.sessionDistanceAnalysis(end).static.forces(inputLoad(1,2)).text(i)) && isgraphics(obj.sessionDistanceAnalysis(end).static.forces(inputLoad(1,2)).text(i))
                        obj.sessionDistanceAnalysis(end).static.forces(inputLoad(1,2)).text(i).Visible='off';
                    end
                end
                
            elseif inputLoad(1,1) == 2
                for i=1:size(obj.sessionDistanceAnalysis(end).static.moments(inputLoad(1,2)).line,1)
                    for j=1:2
                        if ~isempty(obj.sessionDistanceAnalysis(end).static.moments(inputLoad(1,2)).line(i,j)) && isgraphics(obj.sessionDistanceAnalysis(end).static.moments(inputLoad(1,2)).line(i,j))
                            obj.sessionDistanceAnalysis(end).static.moments(inputLoad(1,2)).line(i,j).Color=Colors.Selected.getColor();
                        end
                    end
                end
                
                %no text for distance analysis
                for i=1:length(obj.sessionDistanceAnalysis(end).static.moments(inputLoad(1,2)).text)
                    if ~isempty(obj.sessionDistanceAnalysis(end).static.moments(inputLoad(1,2)).text(i)) && isgraphics(obj.sessionDistanceAnalysis(end).static.moments(inputLoad(1,2)).text(i))
                        obj.sessionDistanceAnalysis(end).static.moments(inputLoad(1,2)).text(i).Visible='off';
                    end
                end
                
            end
        end
        
        function setInputLoad(obj,id,type)
            %set the input load
            if type ==1
                %force
                obj.sessionDistanceAnalysis(end).inputLoad=[1 id];
                
            elseif type  ==2
                %moment
                obj.sessionDistanceAnalysis(end).inputLoad=[2 id];
            end
            obj.plotEverything();
            
        end
        
        function activateLoad(obj,id,type)
            %activate load
            if type ==1
                %force
                obj.sessionDistanceAnalysis(end).static.forces(id).active=1;
            elseif type  ==2
                %moment
                obj.sessionDistanceAnalysis(end).static.moments(id).active=1;
            end
        end
        
        function activateAll(obj,value)
            %activate load
            for i=1:length(obj.sessionDistanceAnalysis(end).static.forces)
                obj.sessionDistanceAnalysis(end).static.forces(i).active=value;
            end
            for i=1:length(obj.sessionDistanceAnalysis(end).static.moments)
                obj.sessionDistanceAnalysis(end).static.moments(i).active=value;
            end
            obj.plotEverything();
            
        end
        
        
        
        function rightClickLoad(obj,source,callbackdata,id,pos,type)
            %right click during on a load during distance analysis
            popHandles=obj.getPopHandles();
            switch(pos)
                case 1
                    %set as input
                    obj.setInputLoad(id,type);
                    handles=obj.getHandles();
                    if logical(obj.sessionDistanceAnalysis(end).distance.member)
                        handles.helpStatic.String='Target Distance:set, Input Force:set';
                        popHandles.analyze.Enable='on';
                    else
                        handles.helpStatic.String='Target Distance:not set, Input Force:set';
                        popHandles.analyze.Enable='off';
                    end
                case 2
                    %activate
                    obj.activateLoad(id,type);
                case 3
                    %activate
                    obj.activateAll(1);
                case 4
                    %activate
                    obj.activateAll(0);
            end
            obj.plotEverything();
            figure(popHandles.distanceGUI);
        end
        
        function activateLoadLoad(obj,id,type,active)
            %activate load
            if type ==1
                %force
                obj.sessionLoadAnalysis(end).static.forces(id).active=active;
            elseif type  ==2
                %moment
                obj.sessionLoadAnalysis(end).static.moments(id).active=active;
            end
            obj.plotEverything();
            
        end
        
        function activateAllLoad(obj,value)
            %activate load
            for i=1:length(obj.sessionLoadAnalysis(end).static.forces)
                obj.sessionLoadAnalysis(end).static.forces(i).active=value;
            end
            for i=1:length(obj.sessionLoadAnalysis(end).static.moments)
                obj.sessionLoadAnalysis(end).static.moments(i).active=value;
            end
            obj.plotEverything();
            
        end
        
        function rightClickLoadLoad(obj,source,callbackdata,id,pos,type)
            %right click during on a load during distance analysis
            switch(pos)
                case 1
                    %activate
                    source.Checked
                    if strcmp(source.Checked,'on')
                        source.Checked='off';
                        obj.activateLoadLoad(id,type,0);
                    else
                        source.Checked='on';
                        obj.activateLoadLoad(id,type,1);
                    end
                case 2
                    %activate all
                    obj.activateAllLoad(1);
                case 3
                    %activate deactivate all
                    obj.activateAllLoad(0);
            end
        end
        
        function rightClickAdvantage(obj,source,callbackdata,id,pos,type)
            %right click during on a load during distance analysis
            popHandles=obj.getPopHandles();
            if ~isempty(popHandles)
                switch(pos)
                    case 1
                        %input force
                        obj.sessionAdvantageAnalysis(end).inputLoad=[type id];
                        if [type id] == obj.sessionAdvantageAnalysis(end).outputLoad
                            obj.sessionAdvantageAnalysis(end).outputLoad=[0 0];
                        end
                    case 2
                        %output force
                        obj.sessionAdvantageAnalysis(end).outputLoad=[type id];
                        if [type id] == obj.sessionAdvantageAnalysis(end).inputLoad
                            obj.sessionAdvantageAnalysis(end).inputLoad=[0 0];
                        end
                end
                handles=obj.getHandles();

                if logical(obj.sessionAdvantageAnalysis(end).inputLoad(1,2)) && logical(obj.sessionAdvantageAnalysis(end).outputLoad(1,2))
                    handles.helpStatic.String='Input Load:set, Output Load:set';
                    popHandles.analyze.Enable='on';
                elseif ~logical(obj.sessionAdvantageAnalysis(end).inputLoad(1,2)) && logical(obj.sessionAdvantageAnalysis(end).outputLoad(1,2))
                    handles.helpStatic.String='Input Load:not set, Output Load:set';
                    popHandles.analyze.Enable='off';
                elseif logical(obj.sessionAdvantageAnalysis(end).inputLoad(1,2)) && ~logical(obj.sessionAdvantageAnalysis(end).outputLoad(1,2))
                    handles.helpStatic.String='Input Load:set, Output Load:not set';
                    popHandles.analyze.Enable='off';
                elseif ~logical(obj.sessionAdvantageAnalysis(end).inputLoad(1,2)) && ~logical(obj.sessionAdvantageAnalysis(end).outputLoad(1,2))
                    handles.helpStatic.String='Input Load:not set, Output Load:not set';
                    popHandles.analyze.Enable='off';
                end
                obj.plotEverything();
                figure(obj.getPopHandles().advantageGUI);
            end
        end
        
        function updateDistance(obj)
            %restore selected member
            if obj.mode == Module.Distance
                obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).updateDistance();
            elseif obj.mode == Module.EnergyPlot
                obj.sessionBistable(end)=obj.sessionBistable(end).updateDistance();
            elseif  obj.mode == Module.FlexuralStiffnessSynthesis
                obj.sessionFlexuralStiffnessSynthesis(end)=obj.sessionFlexuralStiffnessSynthesis(end).updateDistance();
            end

        end

        function deleteDistanceDrawing(obj)
            %delete distance drawing
            delete(obj.sessionDistanceAnalysis(end).distance.line);
            obj.sessionDistanceAnalysis(end).distance=struct('type',[],'member',0,'target',[],'line',[],'selected',0);
        end
        
        function addDistanceAnalysisType(obj,type,target)
            %add distance analysis type
            member=0;
            if type ==1
                %node type
                %find the selected node
                for i=1:length(obj.sessionDistanceAnalysis(end).static.nodes)
                    if logical(obj.sessionDistanceAnalysis(end).static.nodes(i).getSelected())
                        member=i;
                        break;
                    end
                end
            else
                %link type
                %find the selected node
                for i=1:length(obj.sessionDistanceAnalysis(end).static.kinematic.allBeams)
                    if logical(obj.sessionDistanceAnalysis(end).static.kinematic.allBeams{i}.selected)
                        member=i;
                        break;
                    end
                end
            end
            if logical(member)
                delete(obj.sessionDistanceAnalysis(end).distance.line);
                obj.sessionDistanceAnalysis(end).distance=struct('type',type,'member',member,'target',target,'line',[],'selected',0);
                obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).drawDistance(obj.mainFig,obj.currentWorkSpace.getLimit()*0.5);
                handles=obj.getHandles();
                popHandles=obj.getPopHandles();
                if logical(obj.sessionDistanceAnalysis(end).inputLoad(1))
                    handles.helpStatic.String='Target Distance:set, Input Force:set';
                    popHandles.analyze.Enable='on';
                else
                    handles.helpStatic.String='Target Distance:set, Input Force:not set';
                    popHandles.analyze.Enable='off';
                end
            end
            
        end
        
        function deleteEnergyDrawing(obj)
            %delete distance drawing
            delete(obj.sessionBistable(end).distance.line);
            obj.sessionBistable(end).distance=struct('type',[],'member',0,'target',[],'line',[],'selected',0);
        end
        
        function addBistableType(obj,type,target)
            %add distance analysis type
            member=0;
            if type ==1
                %node type
                %find the selected node
                for i=1:length(obj.sessionBistable(end).static.nodes)
                    if logical(obj.sessionBistable(end).static.nodes(i).getSelected())
                        member=i;
                        break;
                    end
                end
            else
                %link type
                %find the selected node
                for i=1:length(obj.sessionBistable(end).static.kinematic.allBeams)
                    if logical(obj.sessionBistable(end).static.kinematic.allBeams{i}.selected)
                        member=i;
                        break;
                    end
                end
            end
            if logical(member)
                delete(obj.sessionBistable(end).distance.line);
                obj.sessionBistable(end).distance=struct('type',type,'member',member,'target',target,'line',[],'selected',0);
                obj.sessionBistable(end)=obj.sessionBistable(end).drawDistance(obj.mainFig,obj.currentWorkSpace.getLimit()*0.5);
            end
        end
        
        function updateLoadAfterDistanceAnalysis(obj,x,type,id)
            %after distance analysis save loads
            switch(type)
                case 1
                    obj.forces(id)=obj.forces(id).updateMagnitude(x);
                case 2
                    obj.moments(id)=obj.moments(id).updateMagnitude(x);
                case 3
                    obj.forces(id)=obj.forces(id).updateXValue(x(1));
                    obj.forces(id)=obj.forces(id).updateYValue(x(2));
                case 4
                    obj.forces(id)=obj.forces(id).updateXValue(x(1));
                case 5
                    obj.forces(id)=obj.forces(id).updateYValue(x(1));
            end
        end
        
        function updateDistanceNodeForce(obj,nodes,beams,forces,moments,state)
            %update nodes and forces during analysis
            obj.sessionDistanceAnalysis(end).static.nodes=nodes;
            obj.sessionDistanceAnalysis(end).static.forces=forces;
            obj.sessionDistanceAnalysis(end).static.moments=moments;
            obj.sessionDistanceAnalysis(end).static.kinematic.nodes=nodes;
            obj.sessionDistanceAnalysis(end).static.kinematic.allBeams=beams;
            obj.sessionDistanceAnalysis(end).state=state;
        end
        
        function saveDistanceNodeForce(obj)
            %save the data
            forces=obj.sessionDistanceAnalysis(end).static.forces;
            moments=obj.sessionDistanceAnalysis(end).static.moments;
            state=obj.sessionDistanceAnalysis(end).state;
            intialState=obj.sessionDistanceAnalysis(end).initialState;
            springs=obj.sessionDistanceAnalysis(end).static.torsionSprings;
            distance=obj.sessionDistanceAnalysis(end).distance;
            inputLoad=obj.sessionDistanceAnalysis(end).inputLoad;
            if isempty(obj.sessionDistanceAnalysis(end).run(obj.sessionDistanceAnalysis(end).runNumber).state)
                %first iteration
                obj.sessionDistanceAnalysis(end).run(obj.sessionDistanceAnalysis(end).runNumber)=struct('state',intialState...
                    ,'workspace',obj.getWorkspace(),'magnitudeList',[0 0],'forces',forces,'moments',moments,'torsionSprings',springs,'distance',distance,'inputLoad',inputLoad);
            else
                obj.sessionDistanceAnalysis(end).run(obj.sessionDistanceAnalysis(end).runNumber).state(end+1,:)=state;
                if  obj.sessionDistanceAnalysis(end).inputLoad(1,1) == 1
                    %force type
                    if logical(forces(obj.sessionDistanceAnalysis(end).inputLoad(1,2)).follower)
                        %follower
                        obj.sessionDistanceAnalysis(end).run(obj.sessionDistanceAnalysis(end).runNumber).magnitudeList(end+1,:)=[forces(obj.sessionDistanceAnalysis(end).inputLoad(1,2)).magnitude 0];
                    else
                        obj.sessionDistanceAnalysis(end).run(obj.sessionDistanceAnalysis(end).runNumber).magnitudeList(end+1,:)=[forces(obj.sessionDistanceAnalysis(end).inputLoad(1,2)).xValue forces(obj.sessionDistanceAnalysis(end).inputLoad(1,2)).yValue];
                    end
                else
                    %moment type
                    obj.sessionDistanceAnalysis(end).run(obj.sessionDistanceAnalysis(end).runNumber).magnitudeList(end+1,:)=[moments(obj.sessionDistanceAnalysis(end).inputLoad(1,2)).magnitude 0];
                end
            end
            
        end
        
        function addAdvantageAnalysis(obj)
            %add a new kinematics
            obj.sessionAdvantageAnalysis(end+1)=Advantage(obj.links,obj.nodes,obj.forces,obj.moments,obj.currentWorkSpace);
        end
        
        function addDistanceAnalysis(obj)
            %add a new kinematics
            obj.sessionDistanceAnalysis(end+1)=DistanceAnalysis(obj.links,obj.nodes,obj.forces,obj.moments,obj.torsionSprings,obj.currentWorkSpace);
        end
        
        function distanceAnalysisMain(obj)
            %main function for distance analysis
            obj.sessionDistanceAnalysis(end).busy=1;
            %delete nodes
            for i=1:length(obj.sessionDistanceAnalysis(end).static.nodes)
                obj.sessionDistanceAnalysis(end).static.nodes(i)=obj.sessionDistanceAnalysis(end).static.nodes(i).deleteNodeDrawing;
            end
            obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).analyze(obj);
        end
        
        function restoreDistanceAnalysis(obj)
            %restore the nodes
            obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).restore(obj);
            obj.plotEverything();
        end
        
        function restoreAdvantage(obj)
            %restore the nodes
            handles=obj.getHandles();
            popHandles=obj.getPopHandles();
            obj.sessionAdvantageAnalysis(end)=obj.sessionAdvantageAnalysis(end).restore(obj);
            handles.helpStatic.String='Input Load:not set, Output Load:not set';
            popHandles.analyze.Enable='off';
        end
        
        function advantageAnalysisMain(obj)
            %main function for advantage analysis
            obj.sessionAdvantageAnalysis(end).busy=1;
            obj.sessionAdvantageAnalysis(end)=obj.sessionAdvantageAnalysis(end).analyze(obj);
            obj.plotEverything();
            obj.sessionAdvantageAnalysis(end).busy=0;
        end
        
        
        function addBistable(obj)
            %add a new bi stable analysis
            obj.sessionBistable(end+1)=BiStable(obj.links,obj.nodes,obj.torsionSprings,obj.currentWorkSpace());
        end
        
        function bistableMain(obj)
            %main function for bistable analysis
            obj.sessionBistable(end).busy=1;
            %delete nodes
            for i=1:length(obj.sessionBistable(end).static.nodes)
                obj.sessionBistable(end).static.nodes(i)=obj.sessionBistable(end).static.nodes(i).deleteNodeDrawing;
            end
            limit=obj.currentWorkSpace.getLimit()*0.5;
            obj.sessionBistable(end)=obj.sessionBistable(end).energyPlot(obj.mainFig,limit,obj.mode,obj);
            obj.sessionBistable(end).busy=0;
        end
        
        
        function addTrackedNode(obj,id)
            %add tracked node
            obj.sessionKinematics(end).trackedNodes(end+1)=id;
        end
        
        function stopTrackingNode(obj,id)
            %add tracked node
            obj.sessionKinematics(end).trackedNodes(obj.sessionKinematics(end).trackedNodes == id)=[];
        end
        
        function isDesign=isDesignModule(obj)
            %check if we are at design
            if obj.mode == Module.Overview || obj.mode == Module.Sketch ||...
                obj.mode == Module.Constrain || obj.mode == Module.Model || ...
                 obj.mode == Module.Linear || obj.mode == Module.Torsion || ...
                obj.mode == Module.Force || obj.mode == Module.Moment  || obj.mode == Module.Compliant
                isDesign=true;
            else
                isDesign=false;
            end
        end
        
        function fillNodeTree(obj)
            %fill the node tree
            handles=obj.getHandles();
            handles.nodesTree.removeAllChildren();
            for i=1:length(obj.nodes)
                handles.nodesTree.add(uitreenode('v0', ['Node ' num2str(i)], ['Node ' num2str(i)], [], false));
                handles.nodesTree.getLastChild().setIcon(im2java(imread('nodeBlue.jpg')));
                if obj.isDesignModule()
                    handles.nodesTree.getLastChild.add(uitreenode('v0', 'Edit', 'Edit', [], true));
                    handles.nodesTree.getLastChild().getLastChild().setIcon(im2java(imread('editTree.jpg')));
                end
                if obj.getMode() == Module.Range || obj.getMode() == Module.Free
                    if isempty(find(obj.getKinematics().trackedNodes == i,1))
                        handles.nodesTree.getLastChild.add(uitreenode('v0', 'Track', 'Track', [], true));
                        handles.nodesTree.getLastChild().getLastChild().setIcon(im2java(imread('trackTree.jpg')));
                    else
                        handles.nodesTree.getLastChild.add(uitreenode('v0', 'Stop Tracking', 'Stop Tracking', [], true));
                        handles.nodesTree.getLastChild().getLastChild().setIcon(im2java(imread('stoptrackTree.jpg')));
                    end
                end
                if obj.isDesignModule()
                    handles.nodesTree.getLastChild.add(uitreenode('v0', 'Delete', 'Delete', [], true));
                    handles.nodesTree.getLastChild().getLastChild().setIcon(im2java(imread('deleteTree.jpg')));
                end
            end
            handles.tree.reloadNode(handles.nodesTree);
            handles.tree.expand(handles.nodesTree);
        end
        
        
        function fillLinkTree(obj)
            %fill the link tree
            handles=obj.getHandles();
            handles.linksTree.removeAllChildren();
            for i=1:length(obj.links)
                handles.linksTree.add(uitreenode('v0', obj.links(i).getName(), obj.links(i).getName(), [], false));
                handles.linksTree.getLastChild().setIcon(im2java(imread('linkTree.jpg'))); 
                if obj.isDesignModule()
                    handles.linksTree.getLastChild.add(uitreenode('v0', 'Edit Joints', 'Edit Joints', [], true));
                    handles.linksTree.getLastChild().getLastChild().setIcon(im2java(imread('editTree.jpg')));
                    handles.linksTree.getLastChild.add(uitreenode('v0', 'Edit Dimension', 'Edit Dimension', [], true));
                    handles.linksTree.getLastChild().getLastChild().setIcon(im2java(imread('editTree.jpg')));
                    if ~isempty(obj.links(i).geometry)
                        handles.linksTree.getLastChild.add(uitreenode('v0', 'Edit Compliance', 'Edit Compliance', [], true));
                        handles.linksTree.getLastChild().getLastChild().setIcon(im2java(imread('editTree.jpg')));
                    else
                        handles.linksTree.getLastChild.add(uitreenode('v0', 'Make Compliant', 'Make Compliant', [], true));
                        handles.linksTree.getLastChild().getLastChild().setIcon(im2java(imread('editTree.jpg')));
                    end
                    handles.linksTree.getLastChild.add(uitreenode('v0', 'Delete', 'Delete', [], true));
                    handles.linksTree.getLastChild().getLastChild().setIcon(im2java(imread('deleteTree.jpg')));
                end
            end
            handles.tree.reloadNode(handles.linksTree);
            handles.tree.expand(handles.linksTree);
        end
        
        function fillLinearSpringTree(obj)
            %fill the link spring tree
            handles=obj.getHandles();
            handles.linearSpringTree.removeAllChildren();
            for i=1:length(obj.links)
                 if logical(obj.links(i).linearSpring)
                    linkNodes=obj.links(i).getNodes();
                    handles.linearSpringTree.add(uitreenode('v0', ['Linear-Spring ' obj.links(i).getName()], ['Linear-Spring ' num2str(linkNodes(1)) '-' num2str(linkNodes(2))], [], false));
                    handles.linearSpringTree.getLastChild().setIcon(im2java(imread('linearSpringBlue.jpg')));
                    if obj.isDesignModule()
                        handles.linearSpringTree.getLastChild.add(uitreenode('v0', 'Edit', 'Edit', [], true));
                        handles.linearSpringTree.getLastChild().getLastChild().setIcon(im2java(imread('editTree.jpg')));
                        handles.linearSpringTree.getLastChild.add(uitreenode('v0', 'Delete', 'Delete', [], true));
                        handles.linearSpringTree.getLastChild().getLastChild().setIcon(im2java(imread('deleteTree.jpg')));
                    end
                 end
            end
            handles.tree.reloadNode(handles.linearSpringTree);
            handles.tree.expand(handles.linearSpringTree);
        end
        
        function addNode(obj,point)
            %add a new node
            obj.nodes(end+1)=Node(length(obj.nodes)+1,point.x,point.y);
            obj.fillNodeTree();
        end
        
        function addLink(obj,nodes)
            %add a new link
            obj.links(end+1)=Link(length(obj.links)+1,nodes);
            obj.fillLinkTree();
        end
        
        function addConstraint(obj,link,value,type,direction)
            %add a new constraint
            obj.constraints(end+1)=Constraint(length(obj.constraints)+1,obj,link,value,type,direction);
        end
        
        function fillTorsionTree(obj)
            %fill the torsion tree
            handles=obj.getHandles();
            handles.torsionsTree.removeAllChildren();
            for i=1:length(obj.torsionSprings)
                handles.torsionsTree.add(uitreenode('v0', ['Spring ' num2str(i)], ['Torsion-Spring ' num2str(i)], [], false));
                handles.torsionsTree.getLastChild().setIcon(im2java(imread('torsionSpringTreeBlue.jpg')));
                if obj.isDesignModule()
                    handles.torsionsTree.getLastChild.add(uitreenode('v0', 'Edit', 'Edit', [], true));
                    handles.torsionsTree.getLastChild().getLastChild().setIcon(im2java(imread('editTree.jpg')));
                    handles.torsionsTree.getLastChild.add(uitreenode('v0', 'Delete', 'Delete', [], true));
                    handles.torsionsTree.getLastChild().getLastChild().setIcon(im2java(imread('deleteTree.jpg')));
                end
            end
            handles.tree.reloadNode(handles.torsionsTree);
            handles.tree.expand(handles.torsionsTree);
        end
        
        function addTorsionSpring(obj,link1,link2,node,stiffness)
            %add a new torsion spring
            obj.torsionSprings(end+1)=TorsionSpring(length(obj.torsionSprings)+1,link1,link2,node,stiffness);
            obj.clickTorsionSpring(length(obj.torsionSprings));
            obj.fillTorsionTree();
        end
        
        function spring=getTorsionSpring(obj,id)
            %get the torsion spring
            spring=obj.torsionSprings(id);
        end
        
        function lastIndex=noOfTorsionSprings(obj)
            %last id of the torsion springs
            lastIndex=length(obj.torsionSprings);
        end
        
        
        function deleteTorsionSpringDrawings(obj)
            %delete all drawings
            for i=1:length(obj.torsionSprings)
                obj.torsionSprings(i)=obj.torsionSprings(i).deleteDrawing();
            end
        end
        
        
        function force=getForce(obj,id)
            %get the force
            force=obj.forces(id);
        end
        
        function moment=getMoment(obj,id)
            %get the moment
            moment=obj.moments(id);
        end
        
        function lastIndex=noOfMoments(obj)
            %last id of the moments
            lastIndex=length(obj.moments);
        end
        
        function lastIndex=noOfForces(obj)
            %last id of the forces
            lastIndex=length(obj.forces);
        end
        
        function deleteForceDrawings(obj)
            %delete all drawings
            for i=1:length(obj.forces)
                obj.forces(i)=obj.forces(i).deleteDrawing();
            end
        end
        
        function deleteMomentDrawings(obj)
            %delete all drawings
            for i=1:length(obj.moments)
                obj.moments(i)=obj.moments(i).deleteDrawing();
            end
        end
        
        
        function deleteLastConstraint(obj)
            %delete the last constraint
            obj.constraints(end)=[];
        end
        
        function deleteTorsionSpringMain(obj,id)
            %delete a torsion spring
            obj.torsionSprings(id)=obj.torsionSprings(id).deleteDrawing();
            obj.torsionSprings(id)=[];
            %adjust tosion spring ids
            for i=1:length(obj.torsionSprings)
                if obj.torsionSprings(i).getID > id
                    obj.torsionSprings(i)=obj.torsionSprings(i).setID(obj.torsionSprings(i).getID-1);
                end
            end
            %tree
            obj.fillNodeTree();
            obj.fillLinkTree();
            obj.fillLinearSpringTree();
            obj.fillTorsionTree();
            obj.fillForceTree();
            obj.fillMomentTree();
            handles=obj.getHandles();
            handles.tree.collapse(handles.nodesTree);
            handles.tree.collapse(handles.linksTree);
            handles.tree.collapse(handles.linearSpringTree);
            handles.tree.collapse(handles.torsionsTree);
            handles.tree.collapse(handles.forcesTree);
            handles.tree.collapse(handles.momentsTree);
        end
        
        function deleteTorsionSpring(obj,source,callbackdata,id)
            %delete a torsion spring
            obj.deleteTorsionSpringMain(id);
            obj.plotEverything();
        end
        
        function deleteForce(obj,source,callbackdata,id)
            %delete a force
            obj.forces(id)=obj.forces(id).deleteDrawing();
            obj.forces(id)=[];
            %adjust force spring ids
            for i=1:length(obj.forces)
                if obj.forces(i).getID > id
                    obj.forces(i)=obj.forces(i).setID(obj.forces(i).getID-1);
                end
            end
            obj.plotEverything();
            %tree
            obj.fillNodeTree();
            obj.fillLinkTree();
            obj.fillLinearSpringTree();
            obj.fillTorsionTree();
            obj.fillForceTree();
            obj.fillMomentTree();
            handles=obj.getHandles();
            handles.tree.collapse(handles.nodesTree);
            handles.tree.collapse(handles.linksTree);
            handles.tree.collapse(handles.linearSpringTree);
            handles.tree.collapse(handles.torsionsTree);
            handles.tree.collapse(handles.forcesTree);
            handles.tree.collapse(handles.momentsTree);
        end
        
        function deleteMoment(obj,source,callbackdata,id)
            %delete a moment
            obj.moments(id)=obj.moments(id).deleteDrawing();
            obj.moments(id)=[];
            %adjust force spring ids
            for i=1:length(obj.moments)
                if obj.moments(i).getID > id
                    obj.moments(i)=obj.moments(i).setID(obj.moments(i).getID-1);
                end
            end
            obj.plotEverything();
            %tree
            obj.fillNodeTree();
            obj.fillLinkTree();
            obj.fillLinearSpringTree();
            obj.fillTorsionTree();
            obj.fillForceTree();
            obj.fillMomentTree();
            handles=obj.getHandles();
            handles.tree.collapse(handles.nodesTree);
            handles.tree.collapse(handles.linksTree);
            handles.tree.collapse(handles.linearSpringTree);
            handles.tree.collapse(handles.torsionsTree);
            handles.tree.collapse(handles.forcesTree);
            handles.tree.collapse(handles.momentsTree);
        end
        
        function lockNode(obj,id,index,value)
            %lock a node
            if logical(value)
                value=1;
            else
                value=0;
            end
            obj.nodes(id)=obj.nodes(id).setConstrained(value,index);
        end
              
        function deleteNode(obj,source,callbackdata,id)
            %delete a node
            if id > length(obj.nodes)
                %dont delete a previously deleted node
                return;
            end
            handles=obj.getHandles();
            %cleaning
            handles.firstNode=[];
            delete(handles.dummyLine);
            handles.dummyLine=gobjects(0);
            guidata(obj.handle,handles);
            %delete the node
            obj.nodes(id).deleteNodeDrawing();
            obj.nodes(id)=[];
            %adjust the nodes
            for i=1:length(obj.nodes)
                if  obj.nodes(i).getID > id
                    obj.nodes(i)=obj.nodes(i).setID(obj.nodes(i).getID-1);
                end
            end
            %adjust the torsion springs
            for i=1:length(obj.torsionSprings)
                if obj.torsionSprings(i).getNode > id
                    obj.torsionSprings(i)=obj.torsionSprings(i).setNode(obj.torsionSprings(i).getNode-1);
                end
            end
            %adjust the Links
            Links2Delete=[];
            for i=1:length(obj.links)
                if obj.links(i).getNode(1) == id || obj.links(i).getNode(2) == id
                    Links2Delete(end+1)=i;
                    continue;
                end
                if obj.links(i).getNode(1) > id
                    obj.links(i)=obj.links(i).setNode(1,obj.links(i).getNode(1)-1);
                end
                if obj.links(i).getNode(2) > id
                    obj.links(i)=obj.links(i).setNode(2,obj.links(i).getNode(2)-1);
                end
            end
            %delete links
            Links2Delete=sort(Links2Delete,'descend');
            for i=1:length(Links2Delete)
                obj.deleteLink([],[],Links2Delete(i));
            end
            %plot
            %plot
            try
                obj.plotEverything();
                %tree
                obj.fillNodeTree();
                obj.fillLinkTree();
                obj.fillLinearSpringTree();
                obj.fillTorsionTree();
                obj.fillForceTree();
                obj.fillMomentTree();
                handles.tree.collapse(handles.nodesTree);
                handles.tree.collapse(handles.linksTree);
                handles.tree.collapse(handles.linearSpringTree);
                handles.tree.collapse(handles.torsionsTree);
                handles.tree.collapse(handles.forcesTree);
                handles.tree.collapse(handles.momentsTree);
            catch
                
            end
        end
        
        function deleteLink(obj,source,callbackdata,id)
            %delete a link
            if id > length(obj.links)
                %dont delete a previously deleted node
                return;
            end
            %delete constraints
            constraintsLink=obj.links(id).getConstraints();
            constraintsLink=sort(constraintsLink,'descend');
            for i=1:length(constraintsLink)
                obj.deleteConstraintMain(constraintsLink(i));
            end
            %delete the link
            obj.links(id).deleteLinkDrawing();
            if ~isempty(obj.links(id).geometry)
                obj.links(id).geometry=obj.links(id).geometry.deleteNodes();
            end
            obj.links(id)=[];
            %delete the forces
            j=1;
            i=1;
            while(j<=length(obj.forces))
                if obj.forces(i).getLink ==id
                    obj.forces(i).deleteDrawing();
                    obj.forces(i)=[];
                    for k=1:length(obj.forces)
                        if obj.forces(k).getID > i
                            obj.forces(k)=obj.forces(k).setID(obj.forces(k).getID-1);
                        end
                    end
                else
                    i=i+1;
                end
                j=j+1;
            end
            %delete the moments
            j=1;
            i=1;
            while(j<=length(obj.moments))
                if obj.moments(i).getLink ==id
                    obj.moments(i).deleteDrawing();
                    obj.moments(i)=[];
                    for k=1:length(obj.moments)
                        if obj.moments(k).getID > i
                            obj.moments(k)=obj.moments(k).setID(obj.moments(k).getID-1);
                        end
                    end
                else
                    i=i+1;
                end
                j=j+1;
            end
            %delete torsion springs
            j=1;
            i=1;
            while(j<=length(obj.torsionSprings))
                links=obj.torsionSprings(i).getLinks();
                if links(1) ==id || links(2) ==id
                    obj.torsionSprings(i).deleteDrawing();
                    obj.torsionSprings(i)=[];
                else
                    i=i+1;
                end
                j=j+1;
            end
            %adjust the links
            for i=1:length(obj.links)
                if  obj.links(i).getID > id
                    obj.links(i)=obj.links(i).setID(obj.links(i).getID-1);
                end
            end
            %adjust the torsion springs
            for i=1:length(obj.torsionSprings)
                links=obj.torsionSprings(i).getLinks();
                if  links(1) > id
                    links(1)=links(1)-1;
                end
                if  links(2) > id
                    links(2)=links(2)-1;
                end
                obj.torsionSprings(i)=obj.torsionSprings(i).setLinks(links);
            end
            %adjust forces
            for i=1:length(obj.forces)
                if  obj.forces(i).getLink > id
                    obj.forces(i)=obj.forces(i).setLink(obj.forces(i).getLink-1);
                end
            end
            
            %adjust moments
            for i=1:length(obj.moments)
                if  obj.moments(i).getLink > id
                    obj.moments(i)=obj.moments(i).setLink(obj.moments(i).getLink-1);
                end
            end
            %adjust constraints
            for i=1:length(obj.constraints)
                if  obj.constraints(i).getType() == ConstraintType.AngleLink
                    constraintLinks=[obj.constraints(i).getLink(1) obj.constraints(i).getLink(2)];
                else
                    constraintLinks=obj.constraints(i).getLink(1);
                end
                for j=1:length(constraintLinks)
                    if  constraintLinks(j) > id
                        obj.constraints(i).link(j)=obj.constraints(i).link(j) -1;
                    end
                end
            end
            
            %delete the points  with no beams
            noLinkNodes=[];
            allNodes=[obj.links.getNodes()];
            for i=1:length(obj.nodes)
                if isempty(find(allNodes==i, 1))
                    noLinkNodes(end+1)=i;
                end
            end
            for i=1:length(noLinkNodes)
                obj.deleteNode([],[],noLinkNodes(length(noLinkNodes)-i+1));
            end
            
            
            
            %plot
            try
                obj.plotEverything();
                %tree
                obj.fillNodeTree();
                obj.fillLinkTree();
                obj.fillLinearSpringTree();
                obj.fillTorsionTree();
                obj.fillForceTree();
                obj.fillMomentTree();
                handles.tree.collapse(handles.nodesTree);
                handles.tree.collapse(handles.linksTree);
                handles.tree.collapse(handles.linearSpringTree);
                handles.tree.collapse(handles.torsionsTree);
                handles.tree.collapse(handles.forcesTree);
                handles.tree.collapse(handles.momentsTree);
            catch
                ;
            end
        end
        
        function number=getCountGroup(obj,groupID)
            %get a count of group
            number=0;
            for i=1:length(obj.links)
                if obj.links(i).getGroup() == groupID
                    number=number+1;
                end
            end
        end
        
        function removeOneLinkGroups(obj)
            %remove one link left groups
            for i=1:length(obj.links)
                if logical(obj.links(i).getGroup())
                    number=obj.getCountGroup(obj.links(i).getGroup());
                    if number == 1
                        obj.links(i)=obj.links(i).setGroup(0);
                    end
                end
            end
            
        end
        
        function incrementGroupID(obj)
            %increment the group id
            obj.groupID=obj.groupID+1;
        end
        
        function setGroup(obj,linkID,groupID)
            %set a group id for a link
            obj.links(linkID)= obj.links(linkID).setGroup(groupID);
        end
        
        
        
        function drawJoint=getIntermediateJoint(obj,groupID,nodeID)
            %check if a node should be drawn
            if groupID == 0
                drawJoint=1;
            else
                allNodes=[];
                for i=1:length(obj.links)
                    if obj.links(i).getGroup() == groupID
                        allNodes=[allNodes obj.links(i).getNodes()];
                    end
                end
                if length(find(allNodes==nodeID)) > 1
                    drawJoint=0;
                else
                    drawJoint=1;
                end
            end
        end
        
        
        function groupID=getGroupID(obj,linkID)
            %get a group id for a link
            groupID=obj.groupID;
        end
        
        function group=checkGroup(obj,ids)
            %check if a group of links can be tied
            group=1;
            for i=1:length(ids)
                id=ids(i);
                if logical(obj.links(id).getGroup()) || ~isempty(obj.links(id).getCrossSection())
                    %if one of them has a group return negative
                    group=0;
                    break;
                end
            end
        end
        
        function deselectEverything(obj)
            %deselect everything
            obj.deselectAllLinks();
            obj.deselectAllNodes();
            obj.setSelectedForce(0);
            obj.setSelectedMoment(0);
            obj.setSelectedTorsionSpring(0);
            obj.plotEverything();
        end
        
        function deselectAllLinks(obj)
            %deselect all links
            for i=1:length(obj.links)
                obj.links(i)=obj.links(i).setSelected(0);
            end
            for i=1:length(obj.sessionKinematics(end).allBeams)
                obj.sessionKinematics(end)=obj.sessionKinematics(end).setSelectedLink(i,0);
            end
            if ~isempty(obj.sessionDistanceAnalysis)
                for i=1:length(obj.sessionDistanceAnalysis(end).static.kinematic.allBeams)
                    obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).setSelectedLink(i,0);
                end
            end
            
            if ~isempty(obj.sessionBistable)
                for i=1:length(obj.sessionBistable(end).static.kinematic.allBeams)
                    obj.sessionBistable(end)=obj.sessionBistable(end).setSelectedLink(i,0);
                end
            end
            
        end
        
        function deselectAllNodes(obj)
            %deselect all links
            for i=1:length(obj.nodes)
                obj.nodes(i)=obj.nodes(i).setSelected(0);
            end
            if ~isempty(obj.sessionDistanceAnalysis)
                for i=1:length(obj.sessionDistanceAnalysis(end).static.nodes)
                    obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).setSelectedNode(i,0);
                end
            end
            if ~isempty(obj.sessionBistable)
                for i=1:length(obj.sessionBistable(end).static.nodes)
                    obj.sessionBistable(end)=obj.sessionBistable(end).setSelectedNode(i,0);
                end
            end
            
        end
        
        function setStatus(obj)
            %get mechanism status and degrees of freedom
            %check if there is an incomplete or invalid beam
            status=[];
            for i=1:length(obj.links)
                if obj.links(i).getStatus() == -1
                    status='Incomplete';
                    dof='-';
                elseif obj.links(i).getStatus()  == 0
                    status='Invalid';
                    dof='-';
                end
            end
            %try to get rigidlinks
            if isempty(status)
                try
                    %get the beams
                    obj.sessionKinematics(1)=Kinematics(obj.links,obj.nodes,obj.currentWorkSpace);
                    status='Valid';
                    obj.degreesOfFreedom=obj.sessionKinematics(1).getDegreesOfFreedom;
                    if logical(obj.degreesOfFreedom) && obj.degreesOfFreedom > 0
                        obj.sessionKinematics(1).inputs(obj.degreesOfFreedom)=struct('linkID',[],'inputType',[],'target',[],'isRelative',0,'initialValue',[],'type',[]);
                    end
                    dof=num2str(obj.degreesOfFreedom);
                    
                catch err
%                    status='Invalid';
                     dof='-';
                     disp(err)
                     err.stack.line
                     err.stack.name
                end
                if isempty(obj.sessionKinematics(1).loops)
                    status='Invalid';
                    dof='-';
                end
            end
            handles=obj.getHandles();
            handles.mechanismStatus.String=['Mechanism: ',status,' D.O.F.: ', dof];
            if strcmp(status,'Valid')
                handles.mechanismStatus.ForegroundColor=Colors.StatusComplete.getColor();
            else
                handles.mechanismStatus.ForegroundColor=Colors.StatusIncomplete.getColor();
            end
        end
        
        function dof=getDegreesOfFreedom(obj)
            %get the dof
            dof=obj.degreesOfFreedom;
        end
        
        function input=getRangeInput(obj,index)
            %get range input
            input=obj.sessionKinematics(end).inputs(index);
        end
        
        function inputs=getRangeInputs(obj)
            %get range inputs
            inputs=obj.sessionKinematics(end).inputs;
        end
        
        function name=getFileName(obj)
            %get the file name
            name=obj.fileName;
        end
        
        function setFileName(obj,name)
            %set the file name
            obj.fileName=name;
        end
        
        function mode=getMode(obj)
            %get the current mode
            mode=obj.mode;
        end
        
        function handle=getHandles(obj)
            %return the handle
            handle=guidata(obj.handle);
        end
        
        function updateMainHandles(obj,handles)
            %update handles
            guidata(obj.handle,handles);
        end
        
        function deletePopFigure(obj)
            %delete pop figure
            try
                obj.popFig.delete();
            catch err
                disp(err);
                delete(obj.popFig);
                disp('cannot delete pop figure');
                obj.popFig=[];
            end
        end
        
        function openApp(obj,func,x,y,additional)
            %open app
            if ~logical(obj.getBusy())
                try
                    obj.deletePopFigure();
                    fnc=str2func(func);
                    %open app
                    obj.popFig=fnc();
                    obj.popFig.setParent(obj);
                    obj.popFigName=func;
                    %set app position
                    handles=obj.getHandles();
                    appLocation=[x,y]+handles.mainGUI.Position(1:2);
                    if isvalid(obj.popFig)
                        obj.popFig.setPosition(appLocation(1),appLocation(2));
                    end
                    %units
                    lengthString=LengthUnit.getString(obj.currentWorkSpace.unitLength);
                    loadString=ForceUnit.getString(obj.currentWorkSpace.unitForce);
                    eString=obj.currentWorkSpace.getEString();
                    if isvalid(obj.popFig)
                        obj.popFig.setUnits({lengthString,loadString,eString});
                    end
                    %additional considerations
                    if isvalid(obj.popFig)
                        obj.popFig.additionalAdjustments(additional);
                    end
                catch
                    obj.deletePopFigure();
                    obj.setBusy(0);
                end
            end
        end
        
        function app=getApp(obj)
            %get app
            try
                app=obj.popFig;
            catch
                obj.deletePopFigure();
            end
        end
        
        function setPopFigure(obj,string,additional)
            %set the pop figure
            try
                if ~logical(obj.getBusy())
                    obj.setBusy(1);
                    obj.deletePopFigure();
                    fnc=str2func(string);
                    lengthString=LengthUnit.getString(obj.currentWorkSpace.unitLength);
                    loadString=ForceUnit.getString(obj.currentWorkSpace.unitForce);
                    handles=obj.getHandles();
                    obj.popFig=fnc('mainGUI',obj.mainFig,'position',handles.mainGUI.Position,'length',lengthString,'load',loadString,'e',obj.currentWorkSpace.getEString(),additional);
                    obj.popFigName=string;
                    obj.setBusy(0);
                end
            catch err
                disp(err);
                obj.setBusy(0);
            end
        end
        
        function setPopFigurePosition(obj,x,y)
            %set the pop figure position
            handles=obj.getHandles();
            if isvalid(obj.popFig)
                obj.popFig.Position(1:2)=[x,y]+handles.mainGUI.Position(1:2);
            end
        end
        
        function setDefaultPopFigurePosition(obj)
            %set the pop figure position
            handles=obj.getHandles();
            obj.popFig.Position(1:2)=[handles.mainGUI.Position(1)*handles.mainGUI.Position(3)/2,handles.mainGUI.Position(2)*handles.mainGUI.Position(4)/2];
        end
        
        
        function handle=getPopHandles(obj)
            %return the pop window handle
            try
                handle=guidata(obj.popFig);
            catch
                handle=[];
            end
        end
        
        function setProgressBar(obj,bar)
            %set the progress bar GUI handle
            obj.progressBar=bar;
        end
        
        function handle=getProgressBar(obj)
            %return the progress bar handle
            try
                handle=guidata(obj.progressBar);
            catch err
                display(err);
                handle=[];
            end
        end
        
        function name=getPopName(obj)
            %return the pop window name
            name=obj.popFigName;
        end
        
        
        function deleteHandles(obj)
            %delete handles before deleting
            obj.handle=[];
            obj.mainFig=[];
            obj.popFig=[];
            obj.progressBar=[];
        end
        
        function updateHandles(obj,handles,axis)
            %delete handles before deleting
            obj.handle=handles.mainGUI;
            obj.mainFig=axis;
        end
        
        function node=getNode(obj,nodeID)
            %return the node
            node=obj.nodes(nodeID).getNode();
        end
        
        function link=getLink(obj,id)
            %get the link
            link=obj.links(id);
        end
        
        function nodes=getLinkNodes(obj,id)
            %get the link nodes
            nodes=obj.links(id).getNodes();
        end
        
        function nodes=getAllLinkNodes(obj)
            %get all the link nodes
            nodes=[];
            for i=1:length(obj.links)
                nodes=horzcat( nodes ,obj.links(i).getNodes());
            end
        end
        
        function constraint=getConstraint(obj,id)
            %get the constraint
            constraint=obj.constraints(id);
        end
        
        function kin=getKinematics(obj)
            %get the latest kinematics
            kin=obj.sessionKinematics(end);
        end
        
        function distance=getDistanceAnalysis(obj)
            %get the latest distance analysis
            distance=obj.sessionDistanceAnalysis(end);
        end
        
        function bistable=getBistable(obj)
            %get the latest bistable analysis
            bistable=obj.sessionBistable(end);
        end
        
        function setFreeKinematics(obj,mode)
            %set the free kinematics mode
            obj.freeKinematics=mode;
        end
        
        function pos=getPosFromID(obj,id)
            %get pos from id
            switch(obj.mode)
                case {Module.Free,Module.Range}
                    beams=obj.sessionKinematics(end).allBeams;
                case Module.Distance
                    beams=obj.sessionDistanceAnalysis(end).static.kinematic.allBeams;
                case Module.EnergyPlot
                    beams=obj.sessionBistable(end).static.kinematic.allBeams;
                case Module.FlexuralStiffnessSynthesis
                    beams=obj.sessionFlexuralStiffnessSynthesis(end).static.kinematic.allBeams;
            end
            for i=1:length(beams)
                if beams{i}.id == id
                    pos=i;
                    break;
                end
            end
        end
        
        function id=getIDFromName(obj,name)
            %get link id from the link name
            id=0;
            for i=1:length(obj.links)
                if strcmp(name,obj.links(i).getName())
                    id=i;
                end
            end
        end
        
        function constraint=getSelectedConstraint(obj)
            %get selected constraint
            constraint=obj.selectedConstraint;
        end
        
        function names=getNames(obj,ids)
            %get names from the ids
            for i=1:length(ids)
                names{i}=obj.links(ids(i)).getName();
            end
            if length(ids) == 1
                names=char(names);
            end
        end
        
        function draggable=getDraggable(obj)
            %check if nodes can be draggable
            draggable=obj.nodesDraggable;
        end
        
        function draggableNode=getDraggableNode(obj)
            %check which nodes can be draggable
            draggableNode=obj.draggedNode;
        end
        
        function select=getMultipleSelect(obj)
            %return multiple select
            select=obj.multipleSelect;
        end
        
        function nodeID=getClickedNode(obj,point)
            nodeID=0;
            %check if a node is selected
            for i=1:length(obj.nodes)
                node=obj.nodes(i).getNode();
                distance=sqrt((node.x - point.x)^2 + (node.y - point.y)^2);
                if distance <= obj.currentWorkSpace.getLimit()*0.02
                    nodeID=i;
                    break;
                end
            end
        end
        
        function [linkID,newPoint,percentage]=getClickedLink(obj,point)
            %check all links
            linkID=0;
            newPoint=[];
            percentage=0;
            for i=1:length(obj.links)
                node1=obj.nodes(obj.links(i).getNode(1)).getNode();
                node2=obj.nodes(obj.links(i).getNode(2)).getNode();
                distance=abs((node2.x-node1.x)*(node1.y-point.y)-(node1.x-point.x)*(node2.y-node1.y))/sqrt((node2.x-node1.x)^2+(node2.y-node1.y)^2);
                if distance <= obj.currentWorkSpace.getLimit()*0.02 && point.x > min(node1.x,node2.x) &&  point.x < max(node1.x,node2.x) && point.y > min(node1.y,node2.y) &&  point.y < max(node1.y,node2.y)
                    linkID=i;
                    linkLength=sqrt((node2.x-node1.x)^2+(node2.y-node1.y)^2);
                    nodeLength=sqrt((point.x-node1.x)^2+(point.y-node1.y)^2);
                    percentage=nodeLength/linkLength;
                    newPoint=Point(node1.x+(node2.x-node1.x)*percentage,node1.y+(node2.y-node1.y)*percentage);
                    break;
                end
            end
        end
        
        
        function divideALinkCompliant(obj,link)
            %divide a link
            percentage=50;
            node1=obj.getNode(obj.links(link).nodes(1,1));
            node2=obj.getNode(obj.links(link).nodes(1,2));
            obj.links(end+1)=obj.links(link);
            obj.links(link).nodes(1,2)=obj.getNoOfNodes+1;
            obj.links(link).joints(1,2)=Joint.Welded;
            obj.links(end).joints(1,1)=Joint.Welded;
            obj.links(end).nodes(1,1)=obj.getNoOfNodes+1;
            obj.links(end).line=gobjects(0);
            obj.links(end).id=length(obj.links);
            %add node
            obj.addNode(Point(0.5*(node1.x+node2.x),0.5*(node1.y+node2.y)));
            %assign torsion springs
            for i=1:length(obj.torsionSprings)
                if obj.torsionSprings(i).node == obj.links(link).nodes(1,2)
                    %it should be other new link
                    if obj.torsionSprings(i).link1 == link
                        obj.torsionSprings(i).link1=length(obj.links);
                    elseif obj.torsionSprings(i).link2 == link
                        obj.torsionSprings(i).link2=length(obj.links);
                    end
                end
            end
            %asssign forces
            for i=1:length(obj.forces)
                if obj.forces(i).distance > percentage
                    obj.forces(i).link=length(obj.links);
                    obj.forces(i).distance=(obj.forces(i).distance-percentage)/(100-percentage)*100;
                end
            end
            %asssign moments
            for i=1:length(obj.moments)
                if obj.moments(i).distance > percentage
                    obj.moments(i).link=length(obj.links);
                    obj.moments(i).distance=(obj.moments(i).distance-percentage)/(100-percentage)*100;
                end
            end
            %assign constraints
            constraints=[];
            for i=1:length(obj.constraints)
                if ~isempty(find(obj.constraints(i).link == link, 1))
                    constraints(end+1)=i;
                end
            end
            constraints=sort(constraints,'descend');
            for i=1:length(constraints)
                obj.deleteConstraintMain(constraints(i));
            end
        end

        
        function divideALink(obj,link,percentage)
            %divide a link
            obj.links(end+1)=obj.links(link);
            obj.links(link).nodes(1,2)=obj.getNoOfNodes+1;
            obj.links(end).nodes(1,1)=obj.getNoOfNodes+1;
            obj.links(end).line=gobjects(0);
            obj.links(end).id=length(obj.links);
            %assign torsion springs
            for i=1:length(obj.torsionSprings)
                if obj.torsionSprings(i).node == obj.links(link).nodes(1,2)
                    %it should be other new link
                    if obj.torsionSprings(i).link1 == link
                        obj.torsionSprings(i).link1=length(obj.links);
                    elseif obj.torsionSprings(i).link2 == link
                        obj.torsionSprings(i).link2=length(obj.links);
                    end
                end
            end
            %asssign forces
            for i=1:length(obj.forces)
                if obj.forces(i).distance > percentage
                    obj.forces(i).link=length(obj.links);
                    obj.forces(i).distance=(obj.forces(i).distance-percentage)/(100-percentage)*100;
                end
            end
            %asssign moments
            for i=1:length(obj.moments)
                if obj.moments(i).distance > percentage
                    obj.moments(i).link=length(obj.links);
                    obj.moments(i).distance=(obj.moments(i).distance-percentage)/(100-percentage)*100;
                end
            end
            %assign constraints
            constraints=[];
            for i=1:length(obj.constraints)
                if ~isempty(find(obj.constraints(i).link == link, 1))
                    constraints(end+1)=i;
                end
            end
            constraints=sort(constraints,'descend');
            for i=1:length(constraints)
                obj.deleteConstraintMain(constraints(i));
            end
        end
        
        function id=getNoOfNodes(obj)
            %get number of nodes
            id=length(obj.nodes);
        end
        
        function id=getNoOfLinks(obj)
            %get number of links
            id=length(obj.links);
        end
        
        function xSize=getXSize(obj)
            %get the x size
            xSize=obj.currentWorkSpace.getX();
        end
        
        function ySize=getYSize(obj)
            %get the x size
            ySize=obj.currentWorkSpace.getY();
        end
        
        function workspace=getWorkspace(obj)
            %get the workspace
            workspace=obj.currentWorkSpace;
        end
        
        function setWorkspace(obj,workspace)
            %set the workspace
            obj.currentWorkSpace=workspace;
            obj.resizeAxis();
        end
        
        function select=getSelectNode(obj,id)
            %get selected for a node
            select=obj.nodes(id).getSelected();
        end
        
        function select=getSelectLink(obj,id)
            %get selected for a node
            select=obj.links(id).getSelected();
        end
        
        function id=getOnlySelectedLink(obj)
            %get the only selected link
            id=0;
            for i=1:length(obj.links)
                if logical(obj.getSelectLink(i))
                    id=i;
                    break;
                end
            end
        end
        
        function id=getOnlySelectedLink2(obj)
            %get the only selected rigid link
            id=0;
            for i=1:length(obj.sessionKinematics(end).allBeams)
                if logical(obj.sessionKinematics(end).allBeams{i}.selected)
                    id=i;
                    break;
                end
            end
        end
        
        function ids=getSelectedLinks(obj)
            %get the all selected links
            ids=[];
            for i=1:length(obj.links)
                if logical(obj.getSelectLink(i))
                    ids(end+1)=i;
                end
            end
        end
        
        function updateSelectedTorsionSpring(obj,stifness)
            %get selected torsion spring
            for i=1:length(obj.torsionSprings)
                if logical(obj.torsionSprings(i).getSelected())
                    obj.torsionSprings(i)=obj.torsionSprings(i).setStiffness(stifness);
                    break;
                end
            end
            obj.plotEverything();
        end
        
        function id=updateSelectedForce(obj,xValue,yValue,magnitude,angle,distance,follower)
            %get selected force
            for i=1:length(obj.forces)
                if logical(obj.forces(i).getSelected())
                    id=i;
                    obj.forces(i)=obj.forces(i).changeForce(xValue,yValue,magnitude,angle,distance,follower);
                    break;
                end
            end
            obj.plotEverything();
        end
        
        function id=updateSelectedMoment(obj,magnitude,distance)
            %get selected force
            for i=1:length(obj.moments)
                if logical(obj.moments(i).getSelected())
                    id=i;
                    obj.moments(i)=obj.moments(i).changeMoment(magnitude,distance);
                    break;
                end
            end
            obj.plotEverything();
        end
        
               
        function constraint=getNodeConstraints(obj,id)
            %get node constraints
            constraint=[obj.nodes(id).getConstrained(1),obj.nodes(id).getConstrained(2)];
        end
        
        function constrained=getNodeConstrained(obj,id)
            %get node constrained
            constrained=obj.nodes(id).getConstrained(1) || obj.nodes(id).getConstrained(2) ;
            if ~logical(constrained)
                %search if the node is constrained because of link
                %constraints
                for i=1:length(obj.constraints)
                    if obj.constraints(i).getType()== ConstraintType.AngleLink
                        constrainedLinks=[obj.constraints(i).getLink(1) obj.constraints(i).getLink(2)] ;
                    else
                        constrainedLinks=obj.constraints(i).getLink(1) ;
                    end
                    constrainedNodes=[];
                    for j=1:length(constrainedLinks)
                        constrainedNodes=[constrainedNodes obj.links(constrainedLinks(j)).getNodes()];
                    end
                    if ~isempty(find(constrainedNodes == id,1))
                        constrained=1;
                        break;
                    end
                end
            end
        end
        
        function moveNodes(obj,deltaX,deltaY)
            %moves the selected nodes
            for i=1:length(obj.nodes)
                if logical(obj.nodes(i).getSelected)
                    node=obj.nodes(i).getNode();
                    if ~logical(obj.getNodeConstrained(i))
                        obj.nodes(i)=obj.nodes(i).setNode(Point(node.x+deltaX,node.y+deltaY));
                    end
                end
            end
        end
        
        function resizeAxis(obj)
            %resize the axis area
            axis(obj.mainFig,[-obj.currentWorkSpace.getX()+obj.currentWorkSpace.origin(1),obj.currentWorkSpace.getX()+obj.currentWorkSpace.origin(1)...
                ,-obj.currentWorkSpace.getY()+obj.currentWorkSpace.origin(2),obj.currentWorkSpace.getY()+obj.currentWorkSpace.origin(1)]);
        end
        
        function resizeWorkspace(obj)
            %resize the axis area
            xRange=xlim(obj.mainFig);
            yRange=ylim(obj.mainFig);
            obj.currentWorkSpace.origin=[sum(xRange)/2, sum(yRange)/2];
            obj.currentWorkSpace.sizeX=(xRange(2)-xRange(1))/2;
            obj.currentWorkSpace.sizeY=(yRange(2)-yRange(1))/2;
        end
        
        function setDraggable(obj,draggable)
            %assign if nodes can be draggable
            obj.nodesDraggable=draggable;
        end
        
        function setDraggableNode(obj,draggableNode)
            %set which nodes can be draggable
            obj.draggedNode=draggableNode;
        end
        
        function setMode(obj,mode)
            %set the current mode
            obj.mode=mode;
        end
        
        function setNode(obj,id,x,y)
            %set the node to the position
            obj.nodes(id)=obj.nodes(id).setNode(Point(x,y));
        end
        
        function setMultipleSelect(obj,select)
            %set multiple select
            obj.multipleSelect=select;
        end
        
        function setSelectNode(obj,id,select)
            %set selected for a node
            obj.nodes(id)=obj.nodes(id).setSelected(select);
        end
        
        function setSelectedLinkDistance(obj,id)
            %set selected link during distance analysis
            obj.sessionDistanceAnalysis(end).selectedLink=id;
            obj.sessionDistanceAnalysis(end).selectedNode=0;
        end
        
        function setSelectedNodeDistance(obj,id)
            %set selected link during distance analysis
            obj.sessionDistanceAnalysis(end).selectedNode=id;
            obj.sessionDistanceAnalysis(end).selectedLink=0;
        end
        
        function setSelectedLinkBistable(obj,id)
            %set selected link during bistable analysis
            obj.sessionBistable(end).selectedLink=id;
            obj.sessionBistable(end).selectedNode=0;
        end
        
        function setSelectedNodeBistable(obj,id)
            %set selected link during  bistable analysis
            obj.sessionBistable(end).selectedNode=id;
            obj.sessionBistable(end).selectedLink=0;
        end
        
        function setSelectedTorsionSpring(obj,id)
            %select one torsion spring and deselect remaining
            for i=1:length(obj.torsionSprings)
                if obj.torsionSprings(i).getID() == id
                    obj.torsionSprings(i)=obj.torsionSprings(i).setSelected(1);
                else
                    obj.torsionSprings(i)=obj.torsionSprings(i).setSelected(0);
                end
            end
        end
        
        function setModeStringRange(obj)
            %set the mode string
            handles=obj.getHandles();
            inputs=obj.getRangeInputs();
            count=0;
            for i=1:length(inputs)
                if ~isempty(inputs(i).linkID)
                    count=count+1;
                end
            end
            handles.helpStatic.String=[num2str(count) '/' num2str(obj.degreesOfFreedom) ' inputs are assigned.' ];
        end
        
        function setSelectedForce(obj,id)
            %select one torsion spring and deselect remaining
            for i=1:length(obj.forces)
                if obj.forces(i).getID() == id
                    obj.forces(i)=obj.forces(i).setSelected(1);
                else
                    obj.forces(i)=obj.forces(i).setSelected(0);
                end
            end
        end
        
        function setSelectedMoment(obj,id)
            %select one torsion spring and deselect remaining
            for i=1:length(obj.moments)
                if obj.moments(i).getID() == id
                    obj.moments(i)=obj.moments(i).setSelected(1);
                else
                    obj.moments(i)=obj.moments(i).setSelected(0);
                end
            end
        end
        
        function setSelectedConstraint(obj,constraint)
            %set selected constraint
            obj.selectedConstraint=constraint;
        end
        
        function setSelectLink(obj,id,select)
            %set selected for a link
            obj.links(id)=obj.links(id).setSelected(select);
        end
        
        function setSelectedLink2(obj,id)
            %set selected for a link
            obj.sessionKinematics(end).allBeams{id}.selected=1;
            obj.sessionKinematics(end).allBeams{id}.clickLinkRange([],[],obj,id);
        end
        
        function setSelectLink2(obj,id,select)
            %set selected for a rigidLink
            obj.sessionKinematics(end)=obj.sessionKinematics(end).setSelectedLink(id,select);
        end
        
        function setSelectLink3(obj,id,select)
            %set selected for a rigidLink
            obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).setSelectedLink(id,select);
        end
        
        function setSelectNode3(obj,id,select)
            %set selected for a node
            obj.sessionDistanceAnalysis(end)=obj.sessionDistanceAnalysis(end).setSelectedNode(id,select);
        end
        
        function setSelectLink4(obj,id,select)
            %set selected for a rigidLink
            obj.sessionBistable(end)=obj.sessionBistable(end).setSelectedLink(id,select);
        end
        
        function setSelectNode4(obj,id,select)
            %set selected for a rigidLink
            obj.sessionBistable(end)=obj.sessionBistable(end).setSelectedNode(id,select);
        end
                
        function loadAnalysis(obj,lowerBound,upperBound,increment)
            %the main load analysis
            limit=obj.currentWorkSpace.getLimit()*0.5;
            obj.sessionLoadAnalysis(end)=obj.sessionLoadAnalysis(end).analysis(lowerBound,upperBound,increment,obj.mainFig,limit,obj.mode,obj);            
        end
        
        function restoreloadAnalysis(obj)
            %the restore load analysis
            obj.sessionLoadAnalysis(end).static.nodes=obj.sessionLoadAnalysis(end).originalNodes;
            obj.sessionLoadAnalysis(end).static.kinematic.nodes=obj.sessionLoadAnalysis(end).originalNodes;
            obj.plotEverything();
        end
        
        function rangeSimulation(obj,inputs,steps)
            %range kinematic simulation
            limit=obj.currentWorkSpace.getLimit()*0.5;
            obj.sessionKinematics(end).inputs=inputs;
            obj.sessionKinematics(end)=obj.sessionKinematics(end).rangeSimulation(obj.mainFig,limit,obj.mode,obj,steps);
        end
        
        
        function freeSimulation(obj,currentPoint)
            %free kinematic simulation
            if ~isempty(obj.freeKinematics.mode) && logical(obj.freeKinematics.mode) && ~logical(obj.sessionKinematics(end).busy)
                limit=obj.currentWorkSpace.getLimit()*0.5;
                obj.setFreeInput(currentPoint);
                obj.sessionKinematics(end).busy=1;
                obj.sessionKinematics(end)=obj.sessionKinematics(end).freeSimulation(obj.mainFig,limit,obj.mode,obj);
                obj.sessionKinematics(end).busy=0;
            end
        end
        
        function setStatusText(obj,string)
            %set the status string
            handles=obj.getHandles();
            handles.helpStatic.String=string;
        end
        
        function kinematicSessions=getAllKinematicSessions(obj)
            %get all kinematic sessions
            if length(obj.sessionKinematics) >= 2
                kinematicSessions=obj.sessionKinematics(2:end);
            else
                kinematicSessions=Kinematics.empty;
            end
        end
        
        function loadAnalysisSessions=getAllLoadAnalysisSessions(obj)
            %get all load analysis sessions
            if ~isempty(obj.sessionLoadAnalysis)
                loadAnalysisSessions=obj.sessionLoadAnalysis;
            else
                loadAnalysisSessions=LoadAnalysis.empty;
            end
        end
        
        function distanceAnalysisSessions=getAllDistanceAnalysisSessions(obj)
            %get all distance analysis sessions
            if ~isempty(obj.sessionDistanceAnalysis)
                distanceAnalysisSessions=obj.sessionDistanceAnalysis;
            else
                distanceAnalysisSessions=DistanceAnalysis.empty;
            end
        end
        
        function advantageSessions=getAllAdvantageSessions(obj)
            %get all advantage analysis sessions
            if ~isempty(obj.sessionAdvantageAnalysis)
                advantageSessions=obj.sessionAdvantageAnalysis;
            else
                advantageSessions=Advantage.empty;
            end
        end
        
        function bistableSessions=getAllBistableSessions(obj)
            %get all bistable sessions
            if ~isempty(obj.sessionBistable)
                bistableSessions=obj.sessionBistable;
            else
                bistableSessions=BiStable.empty;
            end
        end
        
                
        function busy=getBusy(obj)
            %get busy
            busy=obj.busy;
        end
        
        function setBusy(obj,busy)
            %set busy
            obj.busy=busy;
        end
        
        function deleteFlexureDrawing(obj)
            %delete flexure drawing
            delete(obj.sessionFlexuralStiffnessSynthesis(end).distance.line);
            obj.sessionFlexuralStiffnessSynthesis(end).distance=struct('type',[],'member',0,'target',[],'line',[],'selected',0);
        end
        
        function addFlexuralStiffnessSynthesis(obj)
            %add flexural stiffness synthesis
            obj.sessionFlexuralStiffnessSynthesis(1)=FlexuralStiffnessSynthesis(obj.links,obj.nodes,obj.forces,obj.moments,obj.torsionSprings,obj);
            obj.updateFlexuralText();
        end
        
        function flexural=getFlexuralStiffnessSynthesis(obj)
            %get the flexural synthesis
            flexural=obj.sessionFlexuralStiffnessSynthesis(end);
        end
        
        function setSelectLink6(obj,id,select)
            %set selected for a rigidLink
            obj.sessionFlexuralStiffnessSynthesis(end)=obj.sessionFlexuralStiffnessSynthesis(end).setSelectedLink(id,select);
        end
        
        function setSelectNode6(obj,id,select)
            %set selected for a node
            obj.sessionFlexuralStiffnessSynthesis(end)=obj.sessionFlexuralStiffnessSynthesis(end).setSelectedNode(id,select);
        end
        
        function setSelectedLinkFlexural(obj,id)
            %set selected link during flexural synthesis
            obj.sessionFlexuralStiffnessSynthesis(end).selectedLink=id;
            obj.sessionFlexuralStiffnessSynthesis(end).selectedNode=0;
        end
        
        function setSelectedNodeFlexural(obj,id)
            %set selected link during flexural synthesis
            obj.sessionFlexuralStiffnessSynthesis(end).selectedNode=id;
            obj.sessionFlexuralStiffnessSynthesis(end).selectedLink=0;
        end
        
        function addFlexuralType(obj,type,target)
            %add distance analysis type
            member=0;
            if type ==1
                %node type
                %find the selected node
                for i=1:length(obj.sessionFlexuralStiffnessSynthesis(end).static.nodes)
                    if logical(obj.sessionFlexuralStiffnessSynthesis(end).static.nodes(i).getSelected())
                        member=i;
                        break;
                    end
                end
            else
                %link type
                %find the selected node
                for i=1:length(obj.sessionFlexuralStiffnessSynthesis(end).static.kinematic.allBeams)
                    if logical(obj.sessionFlexuralStiffnessSynthesis(end).static.kinematic.allBeams{i}.selected)
                        member=i;
                        break;
                    end
                end
            end
            if logical(member)
                delete(obj.sessionFlexuralStiffnessSynthesis(end).distance.line);
                obj.sessionFlexuralStiffnessSynthesis(end).distance=struct('type',type,'member',member,'target',target,'line',[],'selected',0);
                obj.sessionFlexuralStiffnessSynthesis(end)=obj.sessionFlexuralStiffnessSynthesis(end).drawDistance(obj.mainFig,obj.currentWorkSpace.getLimit()*0.5);
            end
        end
        
        function distance=getDistanceAnalysisTypeFlexural(obj)
            %get distance type
            distance=obj.sessionFlexuralStiffnessSynthesis(end).distance;
        end
        
        function pos=findFlexuralUnknownPos(obj,id)
            %find position of a link in unknowns
            pos=find(obj.sessionFlexuralStiffnessSynthesis(end).unknowns == id);
        end
        
        function flexuralSynthesisRight(obj,source,callbackdata,id)
            %right click on a link during flexural stiffness synthesis
            pos=obj.findFlexuralUnknownPos(id);
            if isempty(pos)
                %add to unknowns
                source.Checked='on';
                obj.sessionFlexuralStiffnessSynthesis(end).unknowns(end+1)=id;
            else
                %remove from unknowns
                source.Checked='off';
                obj.sessionFlexuralStiffnessSynthesis(end).unknowns(pos)=[];
            end
            obj.updateFlexuralText();
            obj.plotEverything();
        end
        
        function count=getFlexuralTotalCompliant(obj)
            %get number of total compliant beams
            count=0;
            for i=1:length(obj.sessionFlexuralStiffnessSynthesis(end).static.kinematic.allBeams)
                if ~isempty(obj.sessionFlexuralStiffnessSynthesis(end).static.kinematic.allBeams{i}.crossSection)
                    count=count+1;
                end
            end
        end
        
        function updateFlexuralText(obj)
            %update the flexural text
            handles=obj.getHandles();
            if isempty(obj.sessionFlexuralStiffnessSynthesis(end).unknowns) || length(obj.sessionFlexuralStiffnessSynthesis(end).unknowns) == 1
                handles.helpStatic.String=['Optimization Variable: ' num2str(length(obj.sessionFlexuralStiffnessSynthesis(end).unknowns)) '/'  num2str(obj.getFlexuralTotalCompliant)];
            else
                handles.helpStatic.String=['Optimization Variables: ' num2str(length(obj.sessionFlexuralStiffnessSynthesis(end).unknowns)) '/'  num2str(obj.getFlexuralTotalCompliant)];
            end
            popHandles=obj.getPopHandles();
            if isempty(obj.sessionFlexuralStiffnessSynthesis(end).unknowns)
                popHandles.analyze.Enable='off';
            else
                popHandles.analyze.Enable='on';
            end
        end
        
        function flexuralSynthesisMain(obj)
            %main function for flexural synthesis
            obj.setBusy(1);
            %delete nodes
            for i=1:length(obj.sessionFlexuralStiffnessSynthesis(end).static.nodes)
                obj.sessionFlexuralStiffnessSynthesis(end).static.nodes(i)=obj.sessionFlexuralStiffnessSynthesis(end).static.nodes(i).deleteNodeDrawing;
            end
            obj.sessionFlexuralStiffnessSynthesis(end)=obj.sessionFlexuralStiffnessSynthesis(end).synthesize(obj);
            obj.setBusy(0);
            obj.plotEverything();
        end
        
        function updateFlexuralNodeForce(obj,nodes,beams)
            %update nodes and forces during analysis
            obj.sessionFlexuralStiffnessSynthesis(end).static.nodes=nodes;
            obj.sessionFlexuralStiffnessSynthesis(end).static.kinematic.nodes=nodes;
            obj.sessionFlexuralStiffnessSynthesis(end).static.kinematic.allBeams=beams;
        end
        
        function restoreFlexuralSynthesis(obj)
            %restore the nodes
            obj.sessionFlexuralStiffnessSynthesis(end)=obj.sessionFlexuralStiffnessSynthesis(end).restore(obj);
            handles=obj.getHandles();
            handles.data.plotEverything();
        end
        
        function FlexuralSynthesis(obj,unknowns,newValues)
            %save values found at flexural synthesis
            for i=1:length(unknowns)
                xSection=obj.links(unknowns(i)).getCrossSection();
                xSection.width=12;
                xSection.thickness=1;
                xSection.E=abs(newValues(i));
                obj.links(unknowns(i))=obj.links(unknowns(i)).setCrossSection(xSection);
            end
            distance=obj.sessionFlexuralStiffnessSynthesis(end).distance;
            obj.sessionFlexuralStiffnessSynthesis(end)=FlexuralStiffnessSynthesis(obj.links,obj.nodes,obj.forces,obj.moments,obj.torsionSprings,obj);
            obj.sessionFlexuralStiffnessSynthesis(end).distance=distance;
        end
        
        function addBiStableSynthesis(obj)
            %add flexural stiffness synthesis
            obj.sessionBiStableSynthesis(1)=BiStableSynthesis(obj.links,obj.nodes,obj.torsionSprings,obj);
        end
        
        function pos=findBistableUnknownPos(obj,id)
            %find position of a link in unknowns
            pos=find(obj.sessionBiStableSynthesis(end).unknowns == id);
        end
        
        function bistableSynthesisRight(obj,source,callbackdata,id)
            %right click on a link during bistable mechanism synthesis
            pos=obj.findBistableUnknownPos(id);
            if isempty(pos)
                %add to unknowns
                source.Checked='on';
                obj.sessionBiStableSynthesis(end).unknowns(end+1)=id;
            else
                %remove from unknowns
                source.Checked='off';
                obj.sessionBiStableSynthesis(end).unknowns(pos)=[];
            end
            obj.plotEverything();
            popHandles=obj.getPopHandles();
            if isempty(obj.sessionBiStableSynthesis(end).unknowns)
                popHandles.synthesize.Enable='off';
            else
                popHandles.synthesize.Enable='on';
            end
        end
        
        function changeBistableSynthesisMode(obj)
            %change mode function
            obj.sessionBiStableSynthesis(end)=obj.sessionBiStableSynthesis(end).setSelectedMode(obj);
            obj.plotEverything();
        end
        
        function changeBistableSynthesisData(obj)
            %change mode function
            obj.sessionBiStableSynthesis(end)=obj.sessionBiStableSynthesis(end).setSelectedData(obj);
            obj.plotEverything();
        end
        
        function bistableSynthesisMain(obj)
            %main function for bistable synthesis
            obj.setBusy(1);
            %delete nodes
            for i=1:length(obj.sessionBiStableSynthesis(end).bistable.static.nodes)
                obj.sessionBiStableSynthesis(end).bistable.static.nodes(i)=obj.sessionBiStableSynthesis(end).bistable.static.nodes(i).deleteNodeDrawing;
            end
            obj.plotEverything();
            obj.sessionBiStableSynthesis(end)=obj.sessionBiStableSynthesis(end).synthesize(obj);
            obj.setBusy(0);
            obj.sessionBiStableSynthesis(end)=obj.sessionBiStableSynthesis(end).restore(obj);
            obj.plotEverything();
        end
        
        function updateBistableSynthesisBistable(obj,bistable,data)
            %update the bistable
            obj.sessionBiStableSynthesis(end)=obj.sessionBiStableSynthesis(end).deleteAll(obj);
            obj.sessionBiStableSynthesis(end).bistable=bistable;
            obj.plotEverything();
            if obj.sessionBiStableSynthesis(end).selectedDriver(1) == 1
                %crank
                obj.sessionBiStableSynthesis(end).crankList{obj.sessionBiStableSynthesis(end).selectedDriver(2),3}=data;
            else
                %slider
                obj.sessionBiStableSynthesis(end).sliderList{obj.sessionBiStableSynthesis(end).selectedDriver(2),3}=data;
            end
        end
        
        function bistable=getBistableSynthesis(obj)
            %get the bistable synthesis
            bistable=obj.sessionBiStableSynthesis(end);
        end
        
        function saveBistableSynthesis(obj,unknowns,newValues)
            %save values found at bistable synthesis
            for i=1:length(unknowns)
                xSection=obj.links(unknowns(i)).getCrossSection();
                xSection.width=12;
                xSection.thickness=1;
                xSection.E=abs(newValues(i));
                obj.links(unknowns(i))=obj.links(unknowns(i)).setCrossSection(xSection);
            end
            obj.sessionBiStableSynthesis(end)=obj.sessionBiStableSynthesis(end).deleteAll(obj);
            obj.sessionBiStableSynthesis(end)=BiStableSynthesis(obj.links,obj.nodes,obj.torsionSprings,obj);
        end
        
        function saveBistableSynthesis2(obj,newNodes)
            %save values found at bistable synthesis
            j=1;
            for i=1:length(obj.nodes)
                obj.nodes(i)=obj.nodes(i).setNode(Point(newNodes(j),newNodes(j+1)));
                j=j+2;
            end
            obj.sessionBiStableSynthesis(end)=obj.sessionBiStableSynthesis(end).deleteAll(obj);
            obj.sessionBiStableSynthesis(end)=BiStableSynthesis(obj.links,obj.nodes,obj.torsionSprings,obj);
        end       
                             
        
        function loadWithoutGUI(obj,nodes,links,torsionSprings,forces,moments)
            %load the input file data
            obj.nodes=nodes;
            obj.links=links;
            obj.torsionSprings=torsionSprings;
            obj.forces=forces;
            obj.moments=moments;
        end
        
        
        function [status,degreesOFFreedom]=checkAllLinksWithoutGUI(obj)
            %check status of all links
            status=1;
            for i=1:length(obj.links)
                link=obj.links(i);
                joint1=link.getJoint(1);
                joint2=link.getJoint(2);
                sliderAngle=link.getAngleSlider();
                if  (joint1 == Joint.GroundSlider || (joint2== Joint.GroundSlider) ) && isempty(sliderAngle)
                    obj.links(i)=obj.links(i).setStatus(-1);
                    status=-1;
                elseif joint1 == Joint.NA || (joint2== Joint.NA)
                    if logical(link.getGroundLink())
                        obj.links(i)=obj.links(i).setStatus(1);
                    else
                        obj.links(i)=obj.links(i).setStatus(-1);
                        status=-1;
                    end
                elseif Helper.checkLegitLink([joint1 joint2])
                    obj.links(i)=obj.links(i).setStatus(1);
                else
                    obj.links(i)=obj.links(i).setStatus(0);
                    status=-1;
                end
                %check if it can be still a linear spring
                if logical(obj.links(i).getLinearSpring()) && ~Helper.checkLegitLinearSpring([joint1 joint2])
                    obj.links(i)=obj.links(i).deleteLinearSpring();
                end
            end
            obj.sessionKinematics(1)=Kinematics(obj.links,obj.nodes,obj.currentWorkSpace);
            obj.degreesOfFreedom=obj.sessionKinematics(1).getDegreesOfFreedom;
            degreesOFFreedom=obj.degreesOfFreedom;
        end
        
        
        function clickForce(obj,id)
            %click on a force during design
            obj.deselectAllLinks();
            obj.deselectAllNodes();
            %select force
            obj.setSelectedForce(id);
            %plot
            obj.plotEverything();
        end
        
        function clickMoment(obj,id)
            %click on a moment during design
            obj.deselectAllLinks();
            obj.deselectAllNodes();
            %select force
            obj.setSelectedMoment(id);
            %fill
            moment=obj.moments(id);
            %plot
            obj.plotEverything();
            %color nodes
            nodes=obj.getLinkNodes(moment.link);
            obj.colorNode(Colors.Point1.getColor(),nodes(1,1));
            obj.colorNode(Colors.Point2.getColor(),nodes(1,2));
        end
        
        function clickTorsionSpring(obj,id)
            %click on a torsion spring
            %select the torsion spring
            obj.setSelectedTorsionSpring(id);
            %string
            obj.deselectAllLinks();
            obj.plotEverything();
        end
           
    end
    
end

