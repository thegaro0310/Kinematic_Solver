classdef ConstantForceSynthesis
    %constant force synthesis class
    properties
        links=Link.empty;
        nodes=Node.empty;
        torsionSprings=TorsionSpring.empty;
        constantBiStable=BiStable.empty;
        mode=1;
        variables={};
        selectedLink=0;
        selectedNode=0;
        nodeRanges=[];
        linkRanges=[];
        distance=struct('type',[],'member',0,'target',[],'line',[],'selected',0);
        torsionRanges=[];
        selectedMember=[0 0];
        plotData=[];
        start=0;
        ending=0;
    end
    
    methods
        function obj=ConstantForceSynthesis(links,nodes,torsionSprings)
            %constructor
            obj.links=links;
            obj.nodes=nodes;
            obj.torsionSprings=torsionSprings;
            obj.mode=1;
            obj.variables{4,3}=[];
            %initial variables
            %first links
            for i=1:length(links)
                if isempty(obj.variables{1,1})
                    %make it reference
                    obj.variables{1,1}=i;
                else
                    %add to variables
                    obj.variables{1,2}=[obj.variables{1,2} i] ;
                end
                %add to list
                obj.variables{1,3}=[obj.variables{1,3} i] ;
            end
            %torsion springs
            for i=1:length(torsionSprings)
                if isempty(obj.variables{2,1})
                    %make it reference
                    obj.variables{2,1}=i;
                else
                    %add to variables
                    obj.variables{2,2}=[obj.variables{2,2} i] ;
                end
                %add to list
                obj.variables{2,3}=[obj.variables{2,3} i] ;
            end
            %linear springs
            for i=1:length(links)
                if logical(links(i).getLinearSpring())
                    if isempty(obj.variables{3,1})
                        %make it reference
                        obj.variables{3,1}=i;
                    else
                        %add to variables
                        obj.variables{3,2}=[obj.variables{3,2} i] ;
                    end
                    %add to list
                    obj.variables{3,3}=[obj.variables{3,3} i] ;
                end
            end
            %compliant stiffness
            for i=1:length(links)
                if ~isempty(links(i).getCrossSection())
                    if isempty(obj.variables{4,1})
                        %make it reference
                        obj.variables{4,1}=i;
                    else
                        %add to variables
                        obj.variables{4,2}=[obj.variables{4,2} i] ;
                    end
                    %add to list
                    obj.variables{4,3}=[obj.variables{4,3} i] ;
                end
            end
            obj=obj.resize();
            obj.nodeRanges=zeros(length(obj.nodes),8);
            obj.linkRanges=zeros(length(obj.links),12);
            obj.torsionRanges=zeros(length(obj.torsionSprings),4);
        end
        
        function obj=drawDistance(obj,mainFig,limit)
            %draw the distance
            target=obj.distance.target(1,2);
            if obj.distance.type == 1
                %node type
                link=obj.getSlider(obj.distance.member);
                theta=obj.links(link).getAngleSlider*pi/180;
                node=obj.nodes(obj.distance.member).getNode();
                %x motion
                obj.distance.line(1)=plot(mainFig,target*cos(theta)+[node.x node.x],target*sin(theta)+[node.y node.y]+[limit*0.2 -limit*0.2],'LineStyle','--','Color',Colors.NormalLink.getColor());
                
                %y motion
                obj.distance.line(2)=plot(mainFig,target*cos(theta)+[node.x node.x]+[limit*0.2 -limit*0.2],target*sin(theta)+[node.y node.y],'LineStyle','--','Color',Colors.NormalLink.getColor());
                
            else
                %link type
                link=obj.links(obj.distance.member);
                node1=obj.nodes(link.getNode(1)).getNode;
                node2=obj.nodes(link.getNode(2)).getNode;
                linkLength=sqrt((node1.x-node2.x)^2+(node1.y-node2.y)^2);
                linkAngle=atan2((node2.y-node1.y),(node2.x-node1.x));
                if obj.distance.target(1,1) == 1
                    %CW motion
                    newAngle=linkAngle-obj.distance.target(1,2)*pi/180;
                else
                    %CCW motion
                    newAngle=linkAngle+obj.distance.target(1,2)*pi/180;
                end
                obj.distance.line=plot(mainFig,[node1.x node1.x+linkLength*cos(newAngle) ],[node1.y node1.y+linkLength*sin(newAngle) ],'LineStyle','--','Color',Colors.NormalLink.getColor());
            end
        end
        
        
        function obj=updateDistance(obj)
            %update the initial nodes and links
            if obj.distance.type == 1
                %node type
                if obj.distance.target(1) == 1
                    for i=1:length(obj.nodes)
                        obj.nodes(i)=obj.nodes(i).setSelected(0);
                    end
                    obj.nodes(obj.distance.member)=obj.nodes(obj.distance.member).setSelected(1);
                end
            else
                %link type
                for i=1:length(obj.links)
                    obj.links(i)=obj.links(i).setSelected(0);
                end
                obj.links(obj.distance.member)=obj.links(obj.distance.member).setSelected(1);
            end
        end
        
        function link=getSlider(obj,nodeID)
            %get link from node id
            link=0;
            for i=1:length(obj.links)
                if (obj.links(i).getNode(2) == nodeID && obj.links(i).getJoint(2) == Joint.GroundSlider) || (obj.links(i).getNode(1) == nodeID && obj.links(i).getJoint(1) == Joint.GroundSlider)
                    link=i;
                end
            end
        end
        
        function obj=setSelectedLink(obj,id,select)
            %set the selected link
            obj.links(id)=obj.links(id).setSelected(select);
        end
        
        function obj=setSelectedNode(obj,id,select)
            %set the selected link
            obj.nodes(id)=obj.nodes(id).setSelected(select);
        end
        
        
        function obj=setSelectedTorsionSpring(obj,id,select)
            %set the selected torsion spring
            obj.torsionSprings(id)=obj.torsionSprings(id).setSelected(select);
        end
        
        function changeUnknownString(obj,parent)
            %change the unknown string
            handles=parent.getHandles();
            switch obj.mode
                case 1 %link lengths
                    if length(obj.variables{1,2}) == 1
                        handles.constantForceTopStatic.String=[num2str(length(obj.variables{1,2})) ' link length is chosen as optimization variable.'];
                    else
                        handles.constantForceTopStatic.String=[num2str(length(obj.variables{1,2})) ' link lengths are chosen as optimization variables.'];
                    end
                case 2%torsion spring stiffness
                    %check if there is a torsional spring
                    if isempty(obj.variables{2,3})
                        handles.constantForceTopStatic.String='There is no torsional spring in the design';
                    else
                        if length(obj.variables{2,2}) == 1
                            handles.constantForceTopStatic.String=[num2str(length(obj.variables{2,2})) ' torsion spring stiffness is chosen as optimization variable.'];
                        else
                            handles.constantForceTopStatic.String=[num2str(length(obj.variables{2,2})) ' torsion spring stiffnesses are chosen as optimization variables.'];
                        end
                    end
                case 3%linear spring stiffness
                    %check if there is a linear spring
                    if isempty(obj.variables{3,3})
                        handles.constantForceTopStatic.String='There is no linear spring in the design';
                    else
                        if length(obj.variables{3,2}) == 1
                            handles.constantForceTopStatic.String=[num2str(length(obj.variables{3,2})) ' linear spring stiffness is chosen as optimization variable.'];
                        else
                            handles.constantForceTopStatic.String=[num2str(length(obj.variables{3,2})) ' linear spring stiffnesses are chosen as optimization variables.'];
                        end
                    end
                case 4%compliant stiffness
                    %check if there is a linear spring
                    if isempty(obj.variables{4,3})
                        handles.constantForceTopStatic.String='There is no compliant member in the design';
                    else
                        if length(obj.variables{4,2}) == 1
                            handles.constantForceTopStatic=[num2str(length(obj.variables{4,2})) ' compliant member flexural stiffness is chosen as optimization variable.'];
                        else
                            handles.constantForceTopStatic=[num2str(length(obj.variables{4,2})) ' compliant member flexural stiffnesses as optimization variables.'];
                        end
                    end
                case 5 %select constants
                    handles.constantForceTopStatic.String='Define the optimization variable ranges by clicking members and nodes.';
                    
            end
        end
        
        function obj=changePanel(obj,parent)
            %change panel
            handles=parent.getHandles();
            parent.setBusy(0);
            handles.mainGUI.Position(3)=155;
            handles.mainGUI.Position(4)=48;
            handles.distancePanel.Visible='off';
            handles.unknownForcePanel.Visible='off';
            handles.previousNextPanel.Visible='off';
            handles.constantForceSynthesisMainPanel.Visible='off';
            handles.nextButton1.Visible='on';
            handles.nextButton2.Visible='on';
            handles.previousButton1.Visible='on';
            handles.previousButton2.Visible='on';
            switch obj.mode
                case 1 %link lengths
                    handles.unknownForcePanel.Visible='on';
                    handles.previousButton1.Visible='off';
                    handles.unknownForcePanel.Title='Optimization Variables-Link Lengths';
                    handles.helpStatic.String='Select reference link by clicking at links.';
                case 2%torsion spring stiffness
                    handles.unknownForcePanel.Visible='on';
                    handles.unknownForcePanel.Title='Optimization Variables-Torsion Springs';
                    handles.helpStatic.String='Select reference stiffness by clicking at torsion springs.';
                case 3%linear spring stiffness
                    handles.unknownForcePanel.Visible='on';
                    handles.unknownForcePanel.Title='Optimization Variables-Linear Springs';
                    handles.helpStatic.String='Select reference stiffness by clicking at linear springs.';
                case 4%compliant member
                    handles.unknownForcePanel.Visible='on';
                    handles.unknownForcePanel.Title='Optimization Variables-Compliant Beams';
                    handles.helpStatic.String='Select reference stiffness by clicking at compliant members.';
                case 5 %select constants
                    handles.unknownForcePanel.Visible='on';
                    handles.unknownForcePanel.Title='Variable Ranges';
                    handles.helpStatic.String='Select optimization variable ranges by clicking at members.';
                case 6%select the driver
                    handles.distancePanel.Visible='on';
                    handles.previousNextPanel.Visible='on';
                    handles.previousNextPanel.Position(1:2)=[100 23];
                    handles.helpStatic.String='Select the driver by clicking at a crank link or a slider.';
                    if logical(obj.distance.member)
                        handles.nextButton2.Visible='on';
                    else
                        handles.nextButton2.Visible='off';
                    end
                case 7%main panel
                    handles.constantForceSynthesisMainPanel.Visible='on';
                    handles.constantForceSynthesisMainPanel.Position(1:2)=[98 9];
                    handles.previousNextPanel.Visible='on';
                    handles.previousNextPanel.Position(1:2)=[110 3];
                    handles.nextButton2.Visible='off';
                    width=170;height=48;
                    handles.mainGUI.Position(3)=width;
                    handles.mainGUI.Position(4)=height;
                    handles.tabbedPanel.Position=[0 39 width height-39];
                    handles.statusBar.Position=[0 0 width 2];
                    handles.helpStatic.Position=[0 0.2 width/2 handles.helpStatic.Position(4)];
                    handles.constantRunStatic.Visible='off';
                    handles.constantRunStatic2.Visible='off';
                    handles.stopConstantAnalysisButton.Visible='off';
                    %set up the bistable
                    obj.constantBiStable=BiStable(obj.links,obj.nodes,obj.torsionSprings,parent.getWorkspace());
                    obj.constantBiStable.distance=obj.distance;
                    obj.constantBiStable=obj.constantBiStable.findCorrectLinkConstantForce(obj.links);
                    [energyList,loadList,energyDistances]=obj.constantBiStable.energyConstant(parent);
                    obj.plotData(1,:)=energyDistances(1:end-1);
                    obj.plotData(2,:)=loadList(1:end);
                    obj.plotData(3,:)=energyList(1:end-1);
                    [ratio,limit1,limit2]=obj.calculateRatio(str2double(handles.percentDistanceText.String));
                    obj.start=limit1;
                    obj.ending=limit2;
                    handles.currentRatioText.String=num2str(ratio);
            end
            handles.panelListStatic.String=[num2str(obj.mode) '/7'];
            handles.panelListStatic2.String=[num2str(obj.mode) '/7'];
            %save the data
            j=1;
            %nodes
            %keep the reference nodes constant
            referenceNodes=obj.links(obj.variables{1,1}).getNodes() ;
            for k=1:length(obj.nodes)
                if isempty(find(referenceNodes==k,1))
                    point=obj.nodes(k).getNode();
                    finalVariables(j)=point.x;
                    finalVariables(j+1)=point.y;
                    j=j+2;
                end
            end
            %torsion springs
            unknownSprings=obj.variables{2,2};
            for k=1:length(unknownSprings)
                finalVariables(j)=obj.torsionSprings(unknownSprings(k)).stiffness;
                j=j+1;
            end
            %linear springs
            unknownSprings=obj.variables{3,2};
            for k=1:length(unknownSprings)
                finalVariables(j)=obj.links(unknownSprings(k)).getLinearSpring();
                j=j+1;
            end
            %compliant links
            unknownSprings=obj.variables{4,2};
            for k=1:length(unknownSprings)
                finalVariables(j)=obj.links(unknownSprings(k)).getCrossSection().E;
                j=j+1;
            end
            parent.updateAfterConstantSynthesis(obj.variables,finalVariables,obj.nodes);
        end
        
        function obj=drawAll(obj,mainFig,limit,mode,parent)
            %draw everything
            obj=obj.deleteAll();
            switch obj.mode
                case 1 %link lengths
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(mainFig,obj.nodes,limit,mode,parent);
                    end
                    %plot torsion springs
                    for i=1:length(obj.torsionSprings)
                        obj.torsionSprings(i)=obj.torsionSprings(i).drawTorsionSpring(obj.nodes,parent);
                    end
                    %color the links
                    %reference link
                    obj.links(obj.variables{1,1})=obj.links(obj.variables{1,1}).colorLine(Colors.Selected.getColor());
                    %unknown links
                    unknowns=obj.variables{1,2};
                    for i=1:length(unknowns)
                        obj.links(unknowns(i))=obj.links(unknowns(i)).colorLine(Colors.Constrained.getColor());
                    end
                case 2 %torsion spring stiffness
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(mainFig,obj.nodes,limit,mode,parent);
                    end
                    %plot torsion springs
                    for i=1:length(obj.torsionSprings)
                        obj.torsionSprings(i)=obj.torsionSprings(i).drawTorsionSpring(obj.nodes,parent);
                    end
                    %color the links
                    %reference torsion spring
                    if ~isempty(obj.variables{2,1})
                        obj.torsionSprings(obj.variables{2,1})=obj.torsionSprings(obj.variables{2,1}).colorSpring(Colors.Selected.getColor());
                        %unknown links
                        unknowns=obj.variables{2,2};
                        for i=1:length(unknowns)
                            obj.torsionSprings(unknowns(i))=obj.torsionSprings(unknowns(i)).colorSpring(Colors.Constrained.getColor());
                        end
                    end
                case 3 %linear spring stiffness
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(mainFig,obj.nodes,limit,mode,parent);
                    end
                    %plot torsion springs
                    for i=1:length(obj.torsionSprings)
                        obj.torsionSprings(i)=obj.torsionSprings(i).drawTorsionSpring(obj.nodes,parent);
                    end
                    %color the links
                    %reference link
                    if ~isempty(obj.variables{3,1})
                        obj.links(obj.variables{3,1})=obj.links(obj.variables{3,1}).colorLine(Colors.Selected.getColor());
                        %unknown links
                        unknowns=obj.variables{3,2};
                        for i=1:length(unknowns)
                            obj.links(unknowns(i))=obj.links(unknowns(i)).colorLine(Colors.Constrained.getColor());
                        end
                    end
                case 4 %compliant member
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(mainFig,obj.nodes,limit,mode,parent);
                    end
                    %plot torsion springs
                    for i=1:length(obj.torsionSprings)
                        obj.torsionSprings(i)=obj.torsionSprings(i).drawTorsionSpring(obj.nodes,parent);
                    end
                    %color the links
                    %reference link
                    if ~isempty(obj.variables{4,1})
                        obj.links(obj.variables{4,1})=obj.links(obj.variables{4,1}).colorLine(Colors.Selected.getColor());
                        %unknown links
                        unknowns=obj.variables{4,2};
                        for i=1:length(unknowns)
                            obj.links(unknowns(i))=obj.links(unknowns(i)).colorLine(Colors.Constrained.getColor());
                        end
                    end
                case 5 %select constants
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(mainFig,obj.nodes,limit,mode,parent);
                    end
                    %plot torsion springs
                    for i=1:length(obj.torsionSprings)
                        obj.torsionSprings(i)=obj.torsionSprings(i).drawTorsionSpring(obj.nodes,parent);
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).drawNode(mainFig,limit,mode,parent);
                    end
                case 6 %select driver
                    for i=1:length(obj.links)
                        %plot links
                        obj.links(i)=obj.links(i).drawLink(mainFig,obj.nodes,limit,mode,parent);
                    end
                    %plot torsion springs
                    for i=1:length(obj.torsionSprings)
                        obj.torsionSprings(i)=obj.torsionSprings(i).drawTorsionSpring(obj.nodes,parent);
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).drawNode(mainFig,limit,mode,parent);
                    end
                    if logical(obj.distance.member)
                        obj=obj.drawDistance(mainFig,limit);
                    end
                case 7 %synthesis part
                    obj.constantBiStable=obj.constantBiStable.drawAll(mainFig,limit,mode,parent);
                    obj.drawPlot(parent);
                    if logical(obj.distance.member)
                        obj=obj.drawDistance(mainFig,limit);
                    end
            end
        end
        
        function obj=deleteAll(obj)
            %delete all drawing
            for i=1:length(obj.links)
                %delete links
                obj.links(i)=obj.links(i).deleteLinkDrawing();
            end
            for i=1:length(obj.nodes)
                %delete nodes
                obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
            end
            %plot torsion springs
            for i=1:length(obj.torsionSprings)
                obj.torsionSprings(i)=obj.torsionSprings(i).deleteDrawing();
            end
            if isgraphics(obj.distance.line)
                delete(obj.distance.line);
            end
            
            if ~isempty(obj.constantBiStable)
                obj.constantBiStable=obj.constantBiStable.deleteAll();
            end
        end
        
        function obj=changeReferance(obj,member,mode)
            %change a referance of a element
            unknowns=obj.variables{mode,2};
            referance=obj.variables{mode,1};
            unknowns(unknowns == member)=referance;
            obj.variables{mode,2}=unknowns;
            obj.variables{mode,1}=member;
        end
        
        function obj=resize(obj)
            %resize the members
            %resize the nodes
            node1=obj.nodes(obj.links(obj.variables{1,1}).getNode(1)).getNode;
            node2=obj.nodes(obj.links(obj.variables{1,1}).getNode(2)).getNode;
            referenceLength=sqrt((node1.x-node2.x)^2+(node1.y-node2.y)^2);
            for i=1:length(obj.nodes)
                currentPoint=obj.nodes(i).getNode();
                obj.nodes(i)=obj.nodes(i).setNode(Point(currentPoint.x/referenceLength,currentPoint.y/referenceLength));
            end
            %resize torsion springs
            if ~isempty(obj.variables{2,3})
                referenceStiffness=obj.torsionSprings(obj.variables{2,1}).stiffness;
                for i=1:length(obj.torsionSprings)
                    obj.torsionSprings(i).stiffness=obj.torsionSprings(i).stiffness/referenceStiffness;
                end
            end
            %linear springs
            if ~isempty(obj.variables{3,3})
                referenceStiffness=obj.links(obj.variables{3,1}).getLinearSpringStiffness();
                springs=obj.variables{3,3};
                for i=1:length(springs)
                    obj.links(springs(i))=obj.links(springs(i)).setLinearSpring(obj.links(springs(i)).getLinearSpringStiffness()/referenceStiffness);
                end
            end
            %compliance
            if ~isempty(obj.variables{4,3})
                referenceStiffness=obj.links(obj.variables{4,1}).getCrossSection().getI()*obj.links(obj.variables{4,1}).getCrossSection().E;
                compliants=obj.variables{4,3};
                for i=1:length(compliants)
                    crossSection=obj.links(compliants(i)).getCrossSection();
                    crossSection.width=12;crossSection.depth=1;
                    crossSection.E=obj.links(compliants(i)).getCrossSection().getI()*obj.links(compliants(i)).getCrossSection().E/referenceStiffness;
                    obj.links(compliants(i))=obj.links(compliants(i)).setCrossSection(crossSection);
                end
            end
        end
        
        function fillData(obj,parent)
            %fill data during range limits
            handles=parent.getHandles();
            member=obj.selectedMember(2);
            handles.lowerConstantText.Enable='on';
            handles.upperConstantText.Enable='on';
            handles.addConstantButton.String='Add';
            switch obj.selectedMember(1)
                case 1%node
                    node=obj.nodes(member).getNode();
                    %reference nodes
                    referenceNodes=obj.links(obj.variables{1,1}).getNodes();
                    if handles.typeConstantPopup.Value == 1
                        %x
                        handles.currentConstantText.String=num2str(node.x);
                        %check if reference
                        if obj.nodes(member).getID() == referenceNodes(1,1) || obj.nodes(member).getID() == referenceNodes(1,2)
                            %this is the referance
                            handles.lowerConstantText.Enable='off';
                            handles.upperConstantText.Enable='off';
                            handles.lowerConstantText.String=num2str(node.x);
                            handles.upperConstantText.String=num2str(node.x);
                            return;
                        end
                        
                        if logical(obj.nodeRanges(member,1))
                            handles.lowerConstantText.String=num2str(obj.nodeRanges(member,5));
                        else
                            handles.lowerConstantText.String=[];
                        end
                        if logical(obj.nodeRanges(member,2))
                            handles.upperConstantText.String=num2str(obj.nodeRanges(member,6));
                        else
                            handles.upperConstantText.String=[];
                        end
                    else
                        %y
                        handles.currentConstantText.String=num2str(node.y);
                        %check if reference
                        if obj.nodes(member).getID() == referenceNodes(1,1) || obj.nodes(member).getID() == referenceNodes(1,2)
                            %this is the referance
                            handles.lowerConstantText.Enable='off';
                            handles.upperConstantText.Enable='off';
                            handles.lowerConstantText.String=num2str(node.y);
                            handles.upperConstantText.String=num2str(node.y);
                            return;
                        end
                        
                        if logical(obj.nodeRanges(member,3))
                            handles.lowerConstantText.String=num2str(obj.nodeRanges(member,7));
                        else
                            handles.lowerConstantText.String=[];
                        end
                        if logical(obj.nodeRanges(member,4))
                            handles.upperConstantText.String=num2str(obj.nodeRanges(member,8));
                        else
                            handles.upperConstantText.String=[];
                        end
                    end
                case 2%link
                    contents = cellstr(handles.typeConstantPopup.String);
                    contents = contents{handles.typeConstantPopup.Value};
                    if handles.typeConstantPopup.Value == 1
                        %length
                        node1=obj.nodes(obj.links(member).getNode(1)).getNode;
                        node2=obj.nodes(obj.links(member).getNode(2)).getNode;
                        linkLength=sqrt((node1.x-node2.x)^2+(node1.y-node2.y)^2);
                        handles.currentConstantText.String=num2str(linkLength);
                        if obj.variables{1,1} == member
                            %this is the referance
                            handles.lowerConstantText.Enable='off';
                            handles.upperConstantText.Enable='off';
                            handles.lowerConstantText.String='1';
                            handles.upperConstantText.String='1';
                            return;
                        end
                        if logical(obj.linkRanges(member,1))
                            handles.lowerConstantText.String=num2str(obj.linkRanges(member,7));
                        else
                            handles.lowerConstantText.String=[];
                        end
                        if logical(obj.linkRanges(member,2))
                            handles.upperConstantText.String=num2str(obj.linkRanges(member,8));
                        else
                            handles.upperConstantText.String=[];
                        end
                    elseif ~isempty(strfind(contents, 'Linear'))
                        handles.currentConstantText.String=num2str(obj.links(member).getLinearSpringStiffness());
                        if obj.variables{3,1} == member
                            %this is the referance
                            handles.lowerConstantText.Enable='off';
                            handles.upperConstantText.Enable='off';
                            handles.lowerConstantText.String='1';
                            handles.upperConstantText.String='1';
                            return;
                        end
                        if logical(obj.linkRanges(member,3))
                            handles.lowerConstantText.String=num2str(obj.linkRanges(member,9));
                        else
                            handles.lowerConstantText.String=[];
                        end
                        if logical(obj.linkRanges(member,4))
                            handles.upperConstantText.String=num2str(obj.linkRanges(member,10));
                        else
                            handles.upperConstantText.String=[];
                        end
                    elseif ~isempty(strfind(contents, 'Compliant'))
                        handles.currentConstantText.String=num2str(obj.links(member).getCrossSection().E);
                        if obj.variables{4,1} == member
                            %this is the referance
                            handles.lowerConstantText.Enable='off';
                            handles.upperConstantText.Enable='off';
                            handles.lowerConstantText.String='1';
                            handles.upperConstantText.String='1';
                            return;
                        end
                        if logical(obj.linkRanges(member,5))
                            handles.lowerConstantText.String=num2str(obj.linkRanges(member,11));
                        else
                            handles.lowerConstantText.String=[];
                        end
                        if logical(obj.linkRanges(member,6))
                            handles.upperConstantText.String=num2str(obj.linkRanges(member,12));
                        else
                            handles.upperConstantText.String=[];
                        end
                    end
                case 3%torsion spring
                    handles.currentConstantText.String=num2str(obj.torsionSprings(member).stiffness);
                    if obj.variables{2,1} == member
                        %this is the referance
                        handles.lowerConstantText.Enable='off';
                        handles.upperConstantText.Enable='off';
                        handles.lowerConstantText.String='1';
                        handles.upperConstantText.String='1';
                        return;
                    end
                    if logical(obj.torsionRanges(member,1))
                        handles.lowerConstantText.String=num2str(obj.torsionRanges(member,3));
                    else
                        handles.lowerConstantText.String=[];
                    end
                    if logical(obj.torsionRanges(member,2))
                        handles.upperConstantText.String=num2str(obj.torsionRanges(member,4));
                    else
                        handles.upperConstantText.String=[];
                    end
            end
        end
        
        function obj=saveData(obj,parent)
            %save data during range limits
            handles=parent.getHandles();
            member=obj.selectedMember(2);
            handles.lowerConstantText.Enable='on';
            handles.upperConstantText.Enable='on';
            switch obj.selectedMember(1)
                case 1%node
                    if handles.typeConstantPopup.Value == 1
                        %x
                        if ~isnan(str2double(handles.lowerConstantText.String))
                            obj.nodeRanges(member,1)=1;
                            obj.nodeRanges(member,5)=str2double(handles.lowerConstantText.String);
                        end
                        if ~isnan(str2double(handles.upperConstantText.String))
                            obj.nodeRanges(member,2)=1;
                            obj.nodeRanges(member,6)=str2double(handles.upperConstantText.String);
                        end
                    else
                        %y
                        if ~isnan(str2double(handles.lowerConstantText.String))
                            obj.nodeRanges(member,3)=1;
                            obj.nodeRanges(member,7)=str2double(handles.lowerConstantText.String);
                        end
                        if ~isnan(str2double(handles.upperConstantText.String))
                            obj.nodeRanges(member,4)=1;
                            obj.nodeRanges(member,8)=str2double(handles.upperConstantText.String);
                        end
                    end
                    handles.addConstantButton.String='Update';
                case 2%link
                    contents = cellstr(handles.typeConstantPopup.String);
                    contents = contents{handles.typeConstantPopup.Value};
                    if handles.typeConstantPopup.Value == 1
                        %length
                        if obj.variables{1,1} == member
                            %this is the referance
                            return;
                        end
                        if ~isnan(str2double(handles.lowerConstantText.String))
                            obj.linkRanges(member,1)=1;
                            obj.linkRanges(member,7)=str2double(handles.lowerConstantText.String);
                        end
                        if ~isnan(str2double(handles.upperConstantText.String))
                            obj.linkRanges(member,2)=1;
                            obj.linkRanges(member,8)=str2double(handles.upperConstantText.String);
                        end
                    elseif ~isempty(strfind(contents, 'Linear'))
                        if obj.variables{3,1} == member
                            %this is the referance
                            return;
                        end
                        if ~isnan(str2double(handles.lowerConstantText.String))
                            obj.linkRanges(member,3)=1;
                            obj.linkRanges(member,9)=str2double(handles.lowerConstantText.String);
                        end
                        if ~isnan(str2double(handles.upperConstantText.String))
                            obj.linkRanges(member,4)=1;
                            obj.linkRanges(member,10)=str2double(handles.upperConstantText.String);
                        end
                    elseif ~isempty(strfind(contents, 'Compliant'))
                        handles.currentConstantText.String=num2str(obj.links(member).getCrossSection().E);
                        if obj.variables{4,1} == member
                            %this is the referance
                            return;
                        end
                        if ~isnan(str2double(handles.lowerConstantText.String))
                            obj.linkRanges(member,5)=1;
                            obj.linkRanges(member,11)=str2double(handles.lowerConstantText.String);
                        end
                        if ~isnan(str2double(handles.upperConstantText.String))
                            obj.linkRanges(member,6)=1;
                            obj.linkRanges(member,12)=str2double(handles.upperConstantText.String);
                        end
                    end
                    handles.addConstantButton.String='Update';
                case 3%torsion spring
                    if obj.variables{2,1} == member
                        %this is the referance
                        return;
                    end
                    if ~isnan(str2double(handles.lowerConstantText.String))
                        obj.torsionRanges(member,1)=1;
                        obj.torsionRanges(member,3)=str2double(handles.lowerConstantText.String);
                    end
                    if ~isnan(str2double(handles.upperConstantText.String))
                        obj.torsionRanges(member,2)=1;
                        obj.torsionRanges(member,4)=str2double(handles.upperConstantText.String);
                    end
                    handles.addConstantButton.String='Update';
            end
        end
        
        function drawPlot(obj,parent)
            %draw the plot for energy and force
            handles=parent.getHandles();
            cla(handles.constantForceAxis);
            hold(handles.constantForceAxis,'on');
            plot(handles.constantForceAxis,obj.plotData(1,:),obj.plotData(2,:),'LineWidth',1.5,'Color',Colors.Constrained.getColor(),'Marker','+','MarkerSize',3);
            grid(handles.constantForceAxis,'on');
            grid(handles.constantForceAxis,'minor');
            
            if obj.distance.type == 2
                %link type
                xlabel(handles.constantForceAxis,'input link angle(deg)','FontSize',9);
                xlim(handles.constantForceAxis,[min(obj.plotData(1,:)) max(obj.plotData(1,:))]);
                ylabel(handles.constantForceAxis,handles.momentUnit.String,'FontSize',9);
                title(handles.constantForceAxis,'Torque Magnitude','FontSize',9);
            else
                %node type
                xlim(handles.constantForceAxis,[min(obj.plotData(1,:)) max(obj.plotData(1,:))]);
                xlabel(handles.constantForceAxis,['slider motion','(',handles.unit1.String,')'],'FontSize',9);
                ylabel(handles.constantForceAxis,handles.forceUnit1.String,'FontSize',9);
                title(handles.constantForceAxis,'Force Magnitude','FontSize',9);
            end
            %plot range
            if logical(obj.start) && logical(obj.ending)
                plot(handles.constantForceAxis,[obj.plotData(1,obj.start) obj.plotData(1,obj.start)],ylim(handles.constantForceAxis),'LineWidth',1.5,'Color',Colors.Blue.getColor(),'LineStyle','--');
                plot(handles.constantForceAxis,[obj.plotData(1,obj.ending) obj.plotData(1,obj.ending)],ylim(handles.constantForceAxis),'LineWidth',1.5,'Color',Colors.Blue.getColor(),'LineStyle','--');
            end
            hold(handles.constantForceAxis,'off');
        end
        
        function [ratio,start,ending]=calculateRatio(obj,percentDistance)
            %calculate the ratio Fmax/Fmin
            if size(obj.plotData,2) < 3
                ratio=NaN;
                start=0;
                ending=0;
                return;
            end
            deltaX=abs(obj.plotData(1,2)-obj.plotData(1,1));
            targetX=abs(obj.distance.target(1,2)*percentDistance/100);
            numberOfNodes=max([round(targetX/deltaX) 1]);
            ratio=Inf;
            start=0;
            ending=0;
            if numberOfNodes == size(obj.plotData,2)
                ratio=max(abs(obj.plotData(2,:)))/min(abs(obj.plotData(2,:)));
                start=1;
                ending=size(obj.plotData,2);
            elseif numberOfNodes > size(obj.plotData,2)-1
                ratio=NaN;
                start=0;
                ending=0;
            else
                for i=2:size(obj.plotData,2)-(numberOfNodes+1)
                    dummy=max(abs(obj.plotData(2,i:i+numberOfNodes)))/min(abs(obj.plotData(2,i:i+numberOfNodes)));
                    if dummy < ratio
                        ratio=dummy;
                        start=i;
                        ending=i+numberOfNodes;
                    end
                end
            end
        end
        
        function obj=analyze(obj,mainFig,limit,parent)
            %main
            handles=parent.getHandles();
            handles.constantRunStatic.Visible='on';
            handles.stopConstantAnalysisButton.Visible='on';
            handles.constantRunStatic.String='Started';
            drawnow;
            parent.setBusy(1);
            runs=str2double(handles.synthesisTimeConstantText.String);
            randomness=str2double(handles.synthesisRandomConstantText.String);
            target=1e9;
            %find the workspace length
            limit=0;
            for i=1:length(obj.nodes)
                point=obj.nodes(i).getNode();
                if limit < abs(point.x)
                    limit=abs(point.x);
                end
                if limit < abs(point.y)
                    limit=abs(point.y);
                end
            end
            limit=limit*2;
            %final variables
            finalVariables=[];
            for i=1:runs
                if ~logical(parent.getBusy())
                    break;
                end
                j=1;
                %nodes
                %keep the reference nodes constant
                referenceNodes=obj.links(obj.variables{1,1}).getNodes() ;
                for k=1:length(obj.nodes)
                    if isempty(find(referenceNodes==k,1))
                        point=obj.nodes(k).getNode();
                        initial(j)=point.x+limit*randomness/100*(rand*2-1);
                        %lower bound
                        if logical(obj.nodeRanges(k,1))
                            lower(j)=max([obj.nodeRanges(k,5) -limit]);
                        else
                            lower(j)=-limit;
                        end
                        %upper bound
                        if logical(obj.nodeRanges(k,2))
                            upper(j)=min([obj.nodeRanges(k,6) limit]);
                        else
                            upper(j)=limit;
                        end
                        initial(j+1)=point.y+limit*randomness/100*(rand*2-1);
                        %lower bound
                        if logical(obj.nodeRanges(k,3))
                            lower(j+1)=max([obj.nodeRanges(k,7) -limit]);
                        else
                            lower(j+1)=-limit;
                        end
                        %upper bound
                        if logical(obj.nodeRanges(k,4))
                            upper(j+1)=min([obj.nodeRanges(k,8) limit]);
                        else
                            upper(j+1)=limit;
                        end
                        j=j+2;
                    end
                end
                %torsion springs
                unknownSprings=obj.variables{2,2};
                for k=1:length(unknownSprings)
                    initial(j)=obj.torsionSprings(unknownSprings(k)).stiffness*(1+randomness/100*(rand*2-1));
                    %lower bound
                    if logical(obj.torsionRanges(unknownSprings(k),1))
                        lower(j)=max([obj.torsionRanges(unknownSprings(k),3) 0.0]);
                    else
                        lower(j)=0.0;
                    end
                    %upper bound
                    if logical(obj.torsionRanges(unknownSprings(k),2))
                        upper(j)=obj.torsionRanges(unknownSprings(k),4);
                    else
                        upper(j)=obj.torsionSprings(unknownSprings(k)).stiffness*100;
                    end
                    j=j+1;
                end
                %linear springs
                unknownSprings=obj.variables{3,2};
                for k=1:length(unknownSprings)
                    initial(j)=obj.links(unknownSprings(k)).getLinearSpring()*(1+randomness/100*(rand*2-1));
                    %lower bound
                    if logical(obj.linkRanges(unknownSprings(k),3))
                        lower(j)=max([obj.linkRanges(unknownSprings(k),9) 0.0]);
                    else
                        lower(j)=0.0;
                    end
                    %upper bound
                    if logical(obj.linkRanges(unknownSprings(k),4))
                        upper(j)=obj.linkRanges(unknownSprings(k),10);
                    else
                        upper(j)=obj.links(unknownSprings(k)).getLinearSpring()*100;
                    end
                    j=j+1;
                end
                %compliant links
                unknownSprings=obj.variables{4,2};
                for k=1:length(unknownSprings)
                    initial(j)=obj.links(unknownSprings(k)).getCrossSection().E*(1+randomness/100*(rand*2-1));
                    %lower bound
                    if logical(obj.linkRanges(unknownSprings(k),5))
                        lower(j)=max([obj.linkRanges(unknownSprings(k),11) 0.0]);
                    else
                        lower(j)=0.0;
                    end
                    %upper bound
                    if logical(obj.linkRanges(unknownSprings(k),6))
                        upper(j)=obj.linkRanges(unknownSprings(k),12);
                    else
                        upper(j)=obj.links(unknownSprings(k)).getCrossSection().E*100;
                    end
                    j=j+1;
                end
                %if final variables are empty save the initial points
                if isempty(finalVariables)
                    finalVariables=initial;
                end
                outputFunc=@(x,optimValues,state)obj.outFunc(x,optimValues,state,parent,i);
                %parallel processing
                poolobj = gcp('nocreate');
                if isempty(poolobj)
                    parallel='never' ;
                else
                    parallel='always' ;
                end
                try
                    opt=optimset('Display','iter-detailed','OutputFcn', outputFunc,'UseParallel',parallel,'MaxFunEvals',200);
                    [x,fval,exitflag,output] =fmincon(@(x)obj.mainSynthesis(x,parent),initial,[],[],[],[],lower,upper,@(x)obj.nonLinear(x),opt);
                    %fminsearch(@(x)obj.mainSynthesis(x,parent),initial,opt);
                    %check if this is the best yet
                    if fval < target
                        target=fval;
                        %update final parameters
                        finalVariables=x;
                        %update nodes
                        j=1;
                        referenceNodes=obj.links(obj.variables{1,1}).getNodes() ;
                        for k=1:length(obj.nodes)
                            if isempty(find(referenceNodes==k,1))
                                obj.nodes(k)=obj.nodes(k).setNode(Point(x(j),x(j+1)));
                                j=j+2;
                            end
                        end
                        %torsion springs
                        unknownSprings=obj.variables{2,2};
                        for k=1:length(unknownSprings)
                            obj.torsionSprings(unknownSprings(k)).stiffness=x(j);
                            j=j+1;
                        end
                        %linear  springs
                        unknownSprings=obj.variables{3,2};
                        for k=1:length(unknownSprings)
                            obj.links(unknownSprings(k))=obj.links(unknownSprings(k)).setLinearSpring(x(j));
                            j=j+1;
                        end
                        %compliant links
                        unknownSprings=obj.variables{4,2};
                        for k=1:length(unknownSprings)
                            crossSection=obj.links(unknownSprings(k)).getCrossSection();
                            crossSection.width=12;crossSection.depth=1;
                            crossSection.E=x(j);
                            obj.links(unknownSprings(k))=obj.links(unknownSprings(k)).setCrossSection(crossSection);
                            j=j+1;
                        end
                        obj=obj.deleteAll();
                        obj.constantBiStable=BiStable(obj.links,obj.nodes,obj.torsionSprings,parent.getWorkspace());
                        obj.constantBiStable.distance=obj.distance;
                        obj.constantBiStable=obj.constantBiStable.findCorrectLinkConstantForce(obj.links);
                        [energyList,loadList,energyDistances]=obj.constantBiStable.energyConstant(parent);
                        obj.plotData=[];
                        obj.plotData(1,:)=energyDistances(1:end-1);
                        obj.plotData(2,:)=loadList(1:end);
                        obj.plotData(3,:)=energyList(1:end-1);
                        [~,limit1,limit2]=obj.calculateRatio(str2double(handles.percentDistanceText.String));
                        obj.start=limit1;
                        obj.ending=limit2;
                        obj=obj.drawAll(mainFig,limit,Module.ConstantForceSynthesis,parent);
                    end
                catch
                end
            end
            parent.setBusy(0);
            handles.stopConstantAnalysisButton.Visible='off';
            handles.constantRunStatic.Visible='off';
            parent.updateAfterConstantSynthesis(obj.variables,finalVariables,obj.nodes);
        end
        
        
        function stop=outFunc(obj,x,optimValues,state,parent,run)
            handles=parent.getHandles();
            obj=obj.updateObject(x);
            %calculate the ratio
            obj.constantBiStable=BiStable(obj.links,obj.nodes,obj.torsionSprings,parent.getWorkspace());
            obj.constantBiStable.distance=obj.distance;
            obj.constantBiStable=obj.constantBiStable.findCorrectLinkConstantForce(obj.links);
            [energyList,loadList,energyDistances]=obj.constantBiStable.energyConstant(parent);
            if length(energyDistances) > 2
                obj.plotData=[];
                obj.plotData(1,:)=energyDistances(1:end-1);
                obj.plotData(2,:)=loadList(1:end);
                obj.plotData(3,:)=energyList(1:end-1);
            end
            [ratio,limit1,limit2]=obj.calculateRatio(str2double(handles.percentDistanceText.String));
            obj.start=limit1;
            obj.ending=limit2;
            
            if  str2double(handles.currentRatioText.String) > ratio
                handles.currentRatioText.String=num2str(ratio);
                obj.drawPlot(parent);
                obj.deleteAll();
                drawnow;
            end
            %
            if ~logical(parent.getBusy())
                stop=true;
                handles.flexuralTargetStatic.String='Target: aborted';
                drawnow;
                return;
            else
                stop=false;
            end
            handles.constantRunStatic.String=['Run:' num2str(run) ' Iter:' num2str(optimValues.iteration)];
            drawnow;
        end
        
        
        function obj=updateObject(obj,x)
            %update the variables of the object
            j=1;
            %update nodes
            referenceNodes=obj.links(obj.variables{1,1}).getNodes() ;
            for i=1:length(obj.nodes)
                if isempty(find(referenceNodes==i,1))
                    obj.nodes(i)=obj.nodes(i).setNode(Point(x(j),x(j+1)));
                    j=j+2;
                end
            end
            %torsion springs
            unknownSprings=obj.variables{2,2};
            for k=1:length(unknownSprings)
                obj.torsionSprings(unknownSprings(k)).stiffness=x(j);
                j=j+1;
            end
            %linear  springs
            unknownSprings=obj.variables{3,2};
            for k=1:length(unknownSprings)
                obj.links(unknownSprings(k))=obj.links(unknownSprings(k)).setLinearSpring(x(j));
                j=j+1;
            end
            %compliant links
            unknownSprings=obj.variables{4,2};
            for k=1:length(unknownSprings)
                crossSection=obj.links(unknownSprings(k)).getCrossSection();
                crossSection.width=12;crossSection.depth=1;
                crossSection.E=x(j);
                obj.links(unknownSprings(k))=obj.links(unknownSprings(k)).setCrossSection(crossSection);
                j=j+1;
            end
        end
        
        function f=mainSynthesis(obj,x,parent)
            %main function for constant force synthesis
            handles=parent.getHandles();
            obj=obj.updateObject(x);
            %calculate the ratio
            obj.constantBiStable=BiStable(obj.links,obj.nodes,obj.torsionSprings,parent.getWorkspace());
            obj.constantBiStable.distance=obj.distance;
            obj.constantBiStable=obj.constantBiStable.findCorrectLinkConstantForce(obj.links);
            [energyList,loadList,energyDistances]=obj.constantBiStable.energyConstant(parent);
            if length(energyDistances) > 2
                obj.plotData=[];
                obj.plotData(1,:)=energyDistances(1:end-1);
                obj.plotData(2,:)=loadList(1:end);
                obj.plotData(3,:)=energyList(1:end-1);
            end
            [ratio,~,~]=obj.calculateRatio(str2double(handles.percentDistanceText.String));
            f=ratio;
        end
        
        function [c,ceq]=nonLinear(obj,x)
            %main function for constant force synthesis
            j=1;
            %update nodes
            referenceNodes=obj.links(obj.variables{1,1}).getNodes() ;
            for i=1:length(obj.nodes)
                if isempty(find(referenceNodes==i,1))
                    obj.nodes(i)=obj.nodes(i).setNode(Point(x(j),x(j+1)));
                    j=j+2;
                end
            end
            ceq=[];
            %lower and upper bounds for the links
            j=1;
            c=[];
            linkLengths=obj.variables{1,2} ;
            for i=1:length(linkLengths)
                referenceNodes=obj.links(linkLengths(i)).getNodes();
                node1=obj.nodes(referenceNodes(1)).getNode;
                node2=obj.nodes(referenceNodes(2)).getNode;
                linkLength=sqrt((node1.x-node2.x)^2+(node1.y-node2.y)^2);
                %lower bound
                if logical(obj.linkRanges(linkLengths(i),1))
                    c(j)=obj.linkRanges(linkLengths(i),7)-linkLength;
                    j=j+1;
                end
                %upper bound
                if logical(obj.linkRanges(linkLengths(i),2))
                    c(j)=linkLength-obj.linkRanges(linkLengths(i),8);
                    j=j+1;
                end
                
            end
        end
        
        
    end
end