classdef Link
    %Design links
    properties 
        id;
        line=gobjects(0);
        nodes;
        joints;
        geometry=CrossSection.empty;
        vector;
        prescribedLength;
        selected=0;
        name;
        status=-1;
        angle;
        angleSlider=[];
        group=0;
        constraints=[];
        linearSpring=0;
        linearSpringStiffness=0;
        groundLink=0;
        prbModel=[];
    end
    
    methods
        function obj=Link(id,nodes)
            %constructor
            obj.id=id;
            obj.nodes=nodes;
            obj.joints=[Joint.NA,Joint.NA];
            obj.name=['Link ' num2str(nodes(1,1)) '-' num2str(nodes(1,2))];
        end
        
        function id=getID(obj)
            %get id
            id=obj.id;
        end
        
        function obj=setID(obj,id)
            %set id
            obj.id=id;
        end
        
        function name=getName(obj)
            %get name
            name=obj.name;
        end
        
        
        function node=getNodes(obj)
            %get the two nodes
            node=[obj.nodes];
        end
        
        function obj=setNodes(obj,newNodes)
            %set the two nodes to new nodes
            obj.nodes=newNodes;
        end
        
        function joint=getJoints(obj)
            %get joints
            joint=[obj.joints];
        end
        

        function node=getNode(obj,pos)
            %get a node
            node=obj.nodes(1,pos);
        end
        
        function joint=getJoint(obj,pos)
            %get joint
            joint=obj.joints(pos);
        end
        
        function obj=setJoint(obj,pos,joint)
            %set joint
            obj.joints(pos)=joint;
        end
        
        function constraints=getConstraints(obj)
            %get constraints
            constraints=obj.constraints;
        end
        
        function obj=setConstraints(obj,constraints)
            %set constraints
            obj.constraints=constraints;
        end
        
        function obj=addConstraint(obj,constraint)
            %add a constraint
            obj.constraints(end+1)=constraint;
        end
        
        function obj=setJoints(obj,joints)
            %set joint
            obj.joints=joints;
        end
        
        function angle=getAngleSlider(obj)
            %get angle
            angle=obj.angleSlider;
        end
        
        function obj=setAngleSlider(obj,angle)
            %set angle
            obj.angleSlider=angle;
        end
        
        function obj=setNode(obj,pos,node)
            %get a node
            obj.nodes(1,pos)=node;
            obj.name=['Link ' num2str(obj.nodes(1,1)) '-' num2str(obj.nodes(1,2))];
        end
        
        function status=getStatus(obj)
            %get status
            status=obj.status;
        end
        
        function obj=setStatus(obj,status)
            %set status
            obj.status=status;
        end
        
        function status=getGroundLink(obj)
            %get ground link
            status=obj.groundLink;
        end
        
        function obj=setGroundLink(obj,status)
            %set ground link
            obj.groundLink=status;
        end
        
        function spring=getLinearSpring(obj)
            %get linear spring
            spring=obj.linearSpring;    
        end
        
        function stiffness=getLinearSpringStiffness(obj)
            %get stiffness
            stiffness=obj.linearSpringStiffness;
        end
        
        function obj=setLinearSpring(obj,stiffness)
            %convert link to linear spring
            obj.linearSpring=1 ;
            obj.linearSpringStiffness=stiffness;        
        end
        
        function obj=deleteLinearSpring(obj)
            %delete linear spring
            obj.linearSpring=0;
            obj.linearSpringStiffness=0;       
        end
        
        function drawNoGUI(obj,currentWorkspace,fig)
            %draw without GUI mode
            figure(fig);
            limit=currentWorkspace.getLimit()*0.5;
            node1=nodes(obj.nodes(1,1)).getNode;
            node2=nodes(obj.nodes(1,2)).getNode;
            [x,y]=DrawLink.drawLine([node1 node2],obj.joints,limit,obj.linearSpring);
            [xJoint1,yJoint1,xJoint2,yJoint2]=DrawLink.drawJoints([node1 node2],obj.joints,limit,obj.angleSlider);
            [xGround1,yGround1,xGround2,yGround2]=DrawLink.drawGrounds([node1 node2],obj.joints,limit,obj.angleSlider);
            if ~isempty(obj.geometry)
                lineWidthBeam=0.5;
            elseif logical(obj.group)
                lineWidthBeam=3.5;
            else
                lineWidthBeam=2;
            end
            obj.line=gobjects(5,7);
            %line
            if ~isempty(x)
                obj.line(1,1)=plot(x,y,'LineWidth',lineWidthBeam);
            end
            %joint 1
            if ~isempty(xJoint1)
                obj.line(2,1)=patch(xJoint1,yJoint1,zeros(length(xJoint1),1),zeros(length(xJoint1),1),'FaceColor','none');
            end
            %joint 2
            if ~isempty(xJoint2)
                obj.line(3,1)=patch(xJoint2,yJoint2,zeros(length(xJoint2),1),zeros(length(xJoint2),1),'FaceColor','none');
            end
            %ground 1
            if obj.joints(1,1)== Joint.GroundPin
                obj.line(4,1)=patch([xGround1(1,1);xGround1(1,2);xGround1(1,3)],[yGround1(1,1);yGround1(1,2);yGround1(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                index=2;
            else
                index=1;
            end
            for i=index:length(xGround1)
                obj.line(4,i)=plot([xGround1(i,1) xGround1(i,2)],[yGround1(i,1) yGround1(i,2)]);
            end
            %ground 2
            if obj.joints(1,2)==Joint.GroundPin
                obj.line(5,1)=patch([xGround2(1,1);xGround2(1,2);xGround2(1,3)],[yGround2(1,1);yGround2(1,2);yGround2(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                index=2;
            else
                index=1;
            end
            for i=index:length(xGround2)
                obj.line(5,i)=plot([xGround2(i,1) xGround2(i,2)],[yGround2(i,1) yGround2(i,2)]);
            end
            %color the beam
            color=Colors.NormalLink.getColor();
            obj.colorLine(color);
        end
        
        
        function obj=drawLink(obj,currentAxis,nodes,limit,mode,parent)
            %draw the link
            node1=nodes(obj.nodes(1,1)).getNode;
            node2=nodes(obj.nodes(1,2)).getNode;
            [x,y]=DrawLink.drawLine([node1 node2],obj.joints,limit,obj.linearSpring);
            [xJoint1,yJoint1,xJoint2,yJoint2]=DrawLink.drawJoints([node1 node2],obj.joints,limit,obj.angleSlider);
            [xGround1,yGround1,xGround2,yGround2]=DrawLink.drawGrounds([node1 node2],obj.joints,limit,obj.angleSlider);
            if ~isempty(obj.geometry)
                lineWidthBeam=0.5;
            elseif logical(obj.group)
                lineWidthBeam=3.5;
            else
                lineWidthBeam=2;
            end
            if isempty(obj.line)
                %new
                obj.line=gobjects(5,7);
            end
            %line
            if ~isempty(x)
                if ~isgraphics(obj.line(1,1))
                    obj.line(1,1)=plot(currentAxis,x,y,'LineWidth',lineWidthBeam);
                else
                    obj.line(1,1).XData=x;
                    obj.line(1,1).YData=y;
                    obj.line(1,1).LineWidth=lineWidthBeam;
                end
            else
                delete(obj.line(1,1));
            end
            
            %joint 1
            if ~isempty(xJoint1) && (logical(parent.getIntermediateJoint(obj.group,obj.nodes(1))) || obj.joints(1) == Joint.GroundPin)
                if ~isgraphics(obj.line(2,1))
                    obj.line(2,1)=patch(currentAxis,xJoint1,yJoint1,zeros(length(xJoint1),1),zeros(length(xJoint1),1),'FaceColor','none');
                else
                    obj.line(2,1).XData=xJoint1;
                    obj.line(2,1).YData=yJoint1;
                end
            else
                delete(obj.line(2,1));
            end
            
            %joint 2
            if ~isempty(xJoint2)&& logical(parent.getIntermediateJoint(obj.group,obj.nodes(2))|| obj.joints(2) == Joint.GroundPin)
                if ~isgraphics(obj.line(3,1))
                    obj.line(3,1)=patch(currentAxis,xJoint2,yJoint2,zeros(length(xJoint2),1),zeros(length(xJoint2),1),'FaceColor','none');
                else
                    obj.line(3,1).XData=xJoint2;
                    obj.line(3,1).YData=yJoint2;
                end
            else
                delete(obj.line(3,1));
            end
            
            %ground 1
            if ~isempty(xGround1)&& logical(parent.getIntermediateJoint(obj.group,obj.nodes(1))|| obj.joints(1) == Joint.GroundPin)
                delete(obj.line(4,:));
                if obj.joints(1,1)== Joint.GroundPin
                    obj.line(4,1)=patch(currentAxis,[xGround1(1,1);xGround1(1,2);xGround1(1,3)],[yGround1(1,1);yGround1(1,2);yGround1(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                    index=2;
                else
                    index=1;
                end
                for i=index:length(xGround1)
                    obj.line(4,i)=plot(currentAxis,[xGround1(i,1) xGround1(i,2)],[yGround1(i,1) yGround1(i,2)]);
                end
            else
                delete(obj.line(4,:));
            end
            
            %ground 2
            if ~isempty(xGround2)&& logical(parent.getIntermediateJoint(obj.group,obj.nodes(2))|| obj.joints(2) == Joint.GroundPin)
                delete(obj.line(5,:));
                if obj.joints(1,2)==Joint.GroundPin
                    obj.line(5,1)=patch(currentAxis,[xGround2(1,1);xGround2(1,2);xGround2(1,3)],[yGround2(1,1);yGround2(1,2);yGround2(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                    index=2;
                else
                    index=1;
                end
                for i=index:length(xGround2)
                    obj.line(5,i)=plot(currentAxis,[xGround2(i,1) xGround2(i,2)],[yGround2(i,1) yGround2(i,2)]);
                end
            else
                delete(obj.line(5,:));
            end
            
            %color the beam
            if logical(obj.selected)
                color=Colors.Selected.getColor();
            elseif obj.status<1 && parent.getMode ~= Module.Sketch
                color=Colors.Incomplete.getColor();
            else
                color=Colors.NormalLink.getColor();
            end
            obj=obj.colorLine(color);
            %right click menu
            obj=obj.rightClickMenu(mode,parent);
            %click event
            obj=obj.clickEvent(mode,parent);
            %name
            obj.name=['Link ' num2str(obj.nodes(1,1)) '-' num2str(obj.nodes(1,2))];
        end
        
        function obj=colorLine(obj,colorBeam)
            %color the line
            if  isgraphics(obj.line(1,1))
                obj.line(1,1).Color=colorBeam;
            end
            %color the joints
            for i=2:3
                if isgraphics(obj.line(i,1))
                    obj.line(i,1).EdgeColor=colorBeam;
                end
            end
            %color the ground
            if isgraphics(obj.line(4,1))&& obj.joints(1,1)== Joint.GroundPin
                obj.line(4,1).FaceColor=colorBeam;
                for i=2:7
                    obj.line(4,i).Color=colorBeam;
                end
            elseif isgraphics(obj.line(4,1))
                for i=1:6
                    obj.line(4,i).Color=colorBeam;
                end
            end
            
            if isgraphics(obj.line(5,1)) && obj.joints(1,2)==Joint.GroundPin
                obj.line(5,1).FaceColor=colorBeam;
                for i=2:7
                    obj.line(5,i).Color=colorBeam;
                end
            elseif isgraphics(obj.line(5,1))
                for i=1:6
                    obj.line(5,i).Color=colorBeam;
                end
            end
        end
        
        function obj=deleteLinkDrawing(obj)
            %delete link drawing
            delete(obj.line);
        end
        
        function editLink(obj,src,evnt,parent,id)
            %edit link 
            handles=parent.getHandles();
            parent.setSelectLink(id,1);
            %select the draggable node
            parent.plotEverything();
            %open the panel
            additional={parent.getNames(id),id,[],1};
            pos=[0.8*handles.mainGUI.Position(3),0.4*handles.mainGUI.Position(4)]; 
            parent.openApp('dimension',pos(1),pos(2),additional);
        end
        
        function editJoints(obj,src,evnt,parent,id)
            %edit link
            handles=parent.getHandles();
            parent.setSelectLink(id,1);
            %select the draggable node
            parent.plotEverything();
            %open popup
            additional={'id',parent.getNames(id),id,1};
            parent.setPopFigure('modelPop',additional);
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            parent.setPopFigurePosition(pos(1),pos(2));
        end
        
        function editLinearSpring(obj,src,evnt,parent,id)
            %edit linear spring 
            handles=parent.getHandles();
            %open the spring popup
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            parent.openApp('linearSpringClick',pos(1),pos(2),{});
            app=parent.getApp();
            app.updateSpring(id);
            app.magnitudeTab.ForegroundColor='black';
            app.typeTab.ForegroundColor=Colors.Inactive.getColor();
            app.gainFocus();
            parent.setSelectLink(id,1);
            %select the draggable node
            parent.plotEverything();
         end
        
        function deleteLinearSpringEvent(obj,src,evnt,parent,id)
            %delete linear spring
            parent.deleteLinearSpring();
            parent.setSelectLink(id,0);
            parent.plotEverything();
        end
        
        function addLinearSpring(obj,src,evnt,parent,id)
            handles=parent.getHandles();
            parent.setSelectLink(id,1);
            %popHandles=parent.getPopHandles();       
            if logical(obj.linearSpring)
                %already linear spring
                uiwait(msgbox('Already converted to a Linear Spring.','Error','error','modal'));
                parent.setSelectLink(id,0);
            elseif logical(Helper.checkLegitLinearSpring(obj.joints)) && ~logical(obj.group) && isempty(obj.geometry)
                %get the pop gui
                pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
                parent.openApp('linearSpringClick',pos(1),pos(2),{});
                app=parent.getApp();
                app.select1.FontColor=Colors.StatusComplete.getColor();
                app.select1.Text=obj.name;
                %change tab
                app.TabGroup.SelectedTab=app.magnitudeTab;
                app.add2AvailableTabs(app.magnitudeTab);
                app.magnitudeTab.ForegroundColor='black';
                handles.link1= id;
                parent.setSelectLink(id,1);
            else
                %invalid link
                uiwait(msgbox('Invalid Link Selected.','Error','error','modal'));
                parent.setSelectLink(id,0);
            end
        end
        
        function editCompliance(obj,src,evnt,parent,id)
            %edit compliance
            handles=parent.getHandles();
            %open popup
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            x=pos(1);y=pos(2);
            if obj.geometry.type == BeamType.PRB
                parent.openApp('prbApp',x*1.05,y*1.05,{});
                app=parent.getApp();
                app.update(obj.geometry.thickness,obj.geometry.width,obj.geometry.E,obj.geometry.prbModel,obj.geometry.order);
            elseif  obj.geometry.type == BeamType.CBCM
                parent.openApp('compliantModelApp',x*1.05,y*1.05,{{'Beam Constraint Model';'Linear Model'},'Beam Constraint Model'});
                app=parent.getApp();
                app.update(obj.geometry.thickness,obj.geometry.width,obj.geometry.E,length(obj.geometry.segments),1);
            elseif obj.geometry.type == BeamType.Mlinear
                parent.openApp('compliantModelApp',x*1.05,y*1.05,{{'Beam Constraint Model';'Linear Model'},'Linear Model'});
                app=parent.getApp();
                app.update(obj.geometry.thickness,obj.geometry.width,obj.geometry.E,length(obj.geometry.segments),2);
            end
            app.gainFocus();
            parent.setSelectLink(id,1);
            %select the draggable node
            parent.plotEverything();
        end
        
        function deleteComplianceEvent(obj,src,evnt,parent,id)
            %delete linear spring
            parent.makeRigid();
            parent.setSelectLink(id,0);
            parent.plotEverything();
        end
        
        function addCompliance(obj,src,evnt,parent,id)
            %edit compliance
            handles=parent.getHandles();
            % main2D('changeMode',handles,Module.Compliant);
            %parent.updateMainHandles(handles);
            parent.setSelectLink(id,1);
            if logical(obj.linearSpring) || logical(obj.group) 
                uiwait(msgbox('Invalid Link Selected.','Error','error','modal'));
                if ~isempty(app)
                    app.gainFocus();
                end
                return;
            end
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            parent.openApp('compliantStep1',pos(1),pos(2),{});
            app=parent.getApp();
            app.select.FontColor=Colors.StatusComplete.getColor();
            app.select.Text=obj.name;
            handles.link1= id;
            parent.setSelectLink(id,1);
            parent.plotEverything();
            app.gainFocus();
            if ~isempty(handles.previousGeometry)
                app.models.Value=handles.previousGeometry{1};
                app.changeModelPublic();
            end
            %change tab
            app.TabGroup.SelectedTab=app.modelType;
            app.add2AvailableTabs(app.modelType);
            app.modelType.ForegroundColor='black';
            %initialized
            guidata(handles.mainGUI,handles);
        end
        
        function addForce(obj,src,evnt,parent,id)
            %add force
            handles=parent.getHandles();
            %open the force popup
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            parent.openApp('forceApp',pos(1),pos(2),{});
            app=parent.getApp();
            app.select.FontColor=Colors.StatusComplete.getColor();
            app.select.Text=obj.name;
            handles.link1= id;
            parent.setSelectLink(id,1);
            parent.plotEverything();
            app.gainFocus();
            %change tab
            app.TabGroup.SelectedTab=app.magnitudeTab;
            app.add2AvailableTabs(app.magnitudeTab);
            app.magnitudeTab.ForegroundColor='black';
            %initialized
            app.initialize(id);
            guidata(handles.mainGUI,handles);
        end
        
        function addMoment(obj,src,evnt,parent,id)
            %add moment
            handles=parent.getHandles();
            %open the force popup
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            parent.openApp('momentApp',pos(1),pos(2),{});
            app=parent.getApp();
            app.select.FontColor=Colors.StatusComplete.getColor();
            app.select.Text=obj.name;
            handles.link1= id;
            parent.setSelectLink(id,1);
            parent.plotEverything();
            app.gainFocus();
            %change tab
            app.TabGroup.SelectedTab=app.magnitudeTab;
            app.add2AvailableTabs(app.magnitudeTab);
            app.magnitudeTab.ForegroundColor='black';
            %initialized
            app.initialize(id);
            guidata(handles.mainGUI,handles);
        end
        
        function obj=rightClickMenu(obj,mode,parent)
            %right click menu
            switch(mode)
                case Module.Overview
                    handles=parent.getHandles();
                    menu= uicontextmenu(handles.mainGUI);
                    uimenu(menu,'Label','Edit Length & Angle','Callback',@(src,evnt)obj.editLink(src,evnt,parent,obj.id));
                    uimenu(menu,'Label','Edit Joints','Callback',@(src,evnt)obj.editJoints(src,evnt,parent,obj.id));
                    if logical(obj.linearSpring)
                        linear=uimenu(menu,'Label','Linear Spring');
                        uimenu(linear,'Label','Edit','Callback',@(src,evnt)obj.editLinearSpring(src,evnt,parent,obj.id));
                        uimenu(linear,'Label','Delete','Callback',@(src,evnt)obj.deleteLinearSpringEvent(src,evnt,parent,obj.id));
                    else
                        uimenu(menu,'Label','Convert to Linear Spring','Callback',@(src,evnt)obj.addLinearSpring(src,evnt,parent,obj.id));
                    end
                    if ~isempty(obj.geometry)
                        linear=uimenu(menu,'Label','Compliance');
                        uimenu(linear,'Label','Edit','Callback',@(src,evnt)obj.editCompliance(src,evnt,parent,obj.id));
                        uimenu(linear,'Label','Make Rigid','Callback',@(src,evnt)obj.deleteComplianceEvent(src,evnt,parent,obj.id));
                    else
                        uimenu(menu,'Label','Make Compliant','Callback',@(src,evnt)obj.addCompliance(src,evnt,parent,obj.id));
                    end
                    uimenu(menu,'Label','Add Force','Callback',@(src,evnt)obj.addForce(src,evnt,parent,obj.id));
                    uimenu(menu,'Label','Add Moment','Callback',@(src,evnt)obj.addMoment(src,evnt,parent,obj.id));
                    uimenu(menu,'Label','Delete','Callback',@(src,evnt)parent.deleteLink(src,evnt,obj.id));
                    for i=1:5
                        for j=1:7
                            if isgraphics(obj.line(i,j))
                                obj.line(i,j).UIContextMenu=menu;
                            end
                        end
                    end
                case Module.Sketch                   
                case Module.Model
                case Module.Constrain
                case Module.Linear
                case Module.Torsion
                case Module.Force
                case Module.Moment
                case Module.Compliant
                case Module.Free
                case Module.Range
                case Module.Load
                case Module.Distance
                case Module.MechanicalAdvantage
                case Module.EnergyPlot
                case Module.PostProcessing
                    handles=parent.getHandles();
                    menu= uicontextmenu(handles.mainGUI);
                    m1 = uimenu(menu,'Label','Set Link-Angle Coordinates as X Axis','Callback',@(source,callbackdata)parent.setLinkXAxis(source,callbackdata,obj.id,1));
                    m2 = uimenu(menu,'Label','Set Link-Length Coordinates as X Axis','Callback',@(source,callbackdata)parent.setLinkXAxis(source,callbackdata,obj.id,2));
                    m3 = uimenu(menu,'Label','Add Link-Angle Coordinates to Y Axis','Callback',@(source,callbackdata)parent.setLinkYAxis(source,callbackdata,obj.id,1));
                    m4 = uimenu(menu,'Label','Add Link-Length Coordinates to Y Axis','Callback',@(source,callbackdata)parent.setLinkYAxis(source,callbackdata,obj.id,2));
                    for i=1:5
                        for j=1:7
                            if isgraphics(obj.line(i,j))
                                obj.line(i,j).UIContextMenu=menu;
                            end
                        end
                    end
            end
        end
        
        function clickLinkModel(obj,src,evnt,parent,id)
            %click event on a link for modelling
            handles=parent.getHandles();
            currentID=id;
            %idList for the selected nodes
            idList=[];
            for i=1:parent.getNoOfLinks()
                %if multiple selection is permitted previously selected points will be
                %retained else only current point be made selected
                if  ~logical(parent.getMultipleSelect) && i ~= currentID
                    parent.setSelectLink(i,0);
                elseif i == currentID
                    parent.setSelectLink(i,1);
                    idList(end+1)=currentID;
                    currentPointIDListPointer=length(idList);
                elseif logical(parent.getSelectLink(i))
                    idList(end+1)=i;
                end
            end
            %deselect all links
            parent.plotEverything();
            %open the panel
            additional={'id',parent.getNames(idList),idList,currentPointIDListPointer};
            handles.data.setPopFigure('modelPop',additional);
            handles.data.setPopFigurePosition(0.8*handles.mainGUI.Position(3),0.4*handles.mainGUI.Position(4));
        end
        
        function clickLinkConstraint(obj,src,evnt,parent,id)
            %click event on a link for modelling
            if ~isempty(parent.getApp())&& isvalid(parent.getApp()) && strcmp(parent.getPopName(),'dimension')
                %check if the app is open
                parent.getApp().selectedLinks.Value=obj.getName();
                parent.getApp().updateLink();
                currentID=id;
                for i=1:parent.getNoOfLinks()
                    %if multiple selection is permitted previously selected points will be
                    %retained else only current point be made selected
                    if  i ~= currentID
                        parent.setSelectLink(i,0);
                    elseif i == currentID
                        parent.setSelectLink(i,1);
                    end
                end
                parent.getApp().gainFocus();
                return;
            end
            %first clear
            handles=parent.getHandles();
            currentID=id;
            %idList for the selected nodes
            idList=[];
            idList2=[];
            for i=1:parent.getNoOfLinks()
                %if multiple selection is permitted previously selected points will be
                %retained else only current point be made selected
                if  i ~= currentID
                    parent.setSelectLink(i,0);
                elseif i == currentID
                    parent.setSelectLink(i,1);
                    idList(end+1)=currentID;
                    currentPointIDListPointer=length(idList);
                elseif logical(parent.getSelectLink(i))
                    idList(end+1)=i;
                    idList2=i;
                end
            end
            %deselect all nodes
            handles.data.deselectAllNodes();
            parent.plotEverything();
            %open the panel
            additional={parent.getNames(idList),idList,idList2,currentPointIDListPointer};
            parent.openApp('dimension',0.8*handles.mainGUI.Position(3),0.4*handles.mainGUI.Position(4),additional);
        end
        
        function clickLinkLinearSpring(obj,src,evnt,parent,id)
            %click event on a link for modelling
            handles=parent.getHandles();
            %check if link stiffness is shown
            app=parent.getApp();
            if ~isempty(app) && ~strcmp(app.TabGroup.SelectedTab.Title,'Type') && logical(app.betweenButton.Value)
                app.gainFocus();
                return;
            end
            for i=1:parent.getNoOfLinks()
                %deselect everything
                parent.setSelectLink(i,0);
            end
            %if (isempty(popHandles) || strcmp(parent.getPopName,'linearSpringPop')) && logical(obj.linearSpring)
             if logical(obj.linearSpring)  
                %open the spring popup
                parent.setSelectLink(id,1);
                parent.plotEverything();
                app.updateSpring(id);
                app.magnitudeTab.ForegroundColor='black';
                app.typeTab.ForegroundColor=Colors.Inactive.getColor();
                handles.helpStatic.String=Module.getString(handles.data.getMode());
                app.gainFocus();
                return;
             elseif isempty(app)
                 return;
             end
            if logical(app.select1.Value)
                if logical(obj.linearSpring)
                    %already linear spring
                    uiwait(msgbox('Already converted to a Linear Spring.','Error','error','modal'));
                    parent.setSelectLink(id,0);
                elseif logical(Helper.checkLegitLinearSpring(obj.joints)) && ~logical(obj.group) && isempty(obj.geometry)
                    %get the pop gui
                    app.select1.FontColor=Colors.StatusComplete.getColor();
                    app.select1.Text=obj.name;
                    %change tab
                    app.TabGroup.SelectedTab=app.magnitudeTab;
                    app.add2AvailableTabs(app.magnitudeTab);
                    app.magnitudeTab.ForegroundColor='black';
                    handles.link1= id;
                    parent.setSelectLink(id,1);
                else
                    %invalid link
                    uiwait(msgbox('Invalid Link Selected.','Error','error','modal'));
                    parent.setSelectLink(id,0);
                end
            end
            parent.plotEverything();
            app.gainFocus();
        end
        
        function clickLinkTorsionSpring(obj,src,evnt,parent,id)
            %click event on a link for torsion spring adding
            handles=parent.getHandles();
            %check if link stiffness is shown
            app=parent.getApp();
            if isempty(app) || ~strcmp(app.TabGroup.SelectedTab.Title,'Type')
                app.gainFocus();
                return;
            end
            for i=1:parent.getNoOfLinks()
                %deselect everything
                parent.setSelectLink(i,0);
            end
            if logical(app.select1.Value)
                %get the pop gui
                app.select1.FontColor=Colors.StatusComplete.getColor();
                app.select1.Text=obj.name;
                app.select1.Value=0;
                app.select2.Value=1;
                handles.link1= id;
                parent.setSelectLink(id,1);
                if handles.link2 > 0
                    parent.setSelectLink(handles.link2,1);
                end
                parent.plotEverything();
                app.gainFocus();
            else
                app.select2.FontColor=Colors.StatusComplete.getColor();
                app.select2.Text=obj.name;
                app.select1.Value=1;
                app.select2.Value=0;
                handles.link2= id;
                parent.setSelectLink(id,1);
                if handles.link1 > 0
                    parent.setSelectLink(handles.link1,1);
                end
                parent.plotEverything();
                app.gainFocus();
            end
            %check if possible to continue
            if (handles.link1 > 0 && handles.link2 > 0 && handles.link1 ~= handles.link2)
                nodes1=parent.getLinkNodes(handles.link1);
                nodes2=parent.getLinkNodes(handles.link2) ;
                if ~isempty(intersect(nodes1,nodes2))
                   %change tab
                    app.TabGroup.SelectedTab=app.magnitudeTab;
                    app.add2AvailableTabs(app.magnitudeTab);
                    app.magnitudeTab.ForegroundColor='black';
                end
            elseif handles.link1== 0 && handles.link2>0
                %link1 is ground
                allNodes=parent.getAllLinkNodes();
                nodes=handles.data.getLinkNodes(handles.link2) ;
                node1=find(allNodes==nodes(1));
                node2=find(allNodes==nodes(2));
                if length(node1) == 1 || length(node2) == 1
                    %change tab
                    app.TabGroup.SelectedTab=app.magnitudeTab;
                    app.add2AvailableTabs(app.magnitudeTab);
                    app.magnitudeTab.ForegroundColor='black';
                else
                    app.magnitudeTab.ForegroundColor=Colors.Inactive.getColor();
                end
            elseif handles.link1> 0 && handles.link2 ==0
                allNodes=parent.getAllLinkNodes();
                nodes=handles.data.getLinkNodes(handles.link1) ;
                node1=find(allNodes==nodes(1));
                node2=find(allNodes==nodes(2));
                if length(node1) == 1 || length(node2) == 1
                   %change tab
                    app.TabGroup.SelectedTab=app.magnitudeTab;
                    app.add2AvailableTabs(app.magnitudeTab);
                    app.magnitudeTab.ForegroundColor='black';
                else
                    app.magnitudeTab.ForegroundColor=Colors.Inactive.getColor();
                end
            else
                app.magnitudeTab.ForegroundColor=Colors.Inactive.getColor();
            end
            guidata(handles.mainGUI,handles);
        end
        
        function clickLinkForce(obj,src,evnt,parent,id)
            %click event on a link for force adding
            handles=parent.getHandles();
            %check if link stiffness is shown
            app=parent.getApp();
            if isempty(app) || ~strcmp(app.TabGroup.SelectedTab.Title,'Link')
                app.gainFocus();
                return;
            end
            for i=1:parent.getNoOfLinks()
                %deselect everything
                parent.setSelectLink(i,0);
            end
            if logical(app.select.Value)
                app.select.FontColor=Colors.StatusComplete.getColor();
                app.select.Text=obj.name;
                handles.link1= id;
                parent.setSelectLink(id,1);
                parent.plotEverything();
                app.gainFocus();
                %change tab
                app.TabGroup.SelectedTab=app.magnitudeTab;
                app.add2AvailableTabs(app.magnitudeTab);
                app.magnitudeTab.ForegroundColor='black';
                %initialized
                app.initialize(id);
            end
            guidata(handles.mainGUI,handles);
        end
        
        function clickLinkMoment(obj,src,evnt,parent,id)
            %click event on a link for moment adding
            handles=parent.getHandles();
            %check if link stiffness is shown
            app=parent.getApp();
            if isempty(app) || ~strcmp(app.TabGroup.SelectedTab.Title,'Link')
                app.gainFocus();
                return;
            end
            for i=1:parent.getNoOfLinks()
                %deselect everything
                parent.setSelectLink(i,0);
            end
            if logical(app.select.Value)
                app.select.FontColor=Colors.StatusComplete.getColor();
                app.select.Text=obj.name;
                handles.link1= id;
                parent.setSelectLink(id,1);
                parent.plotEverything();
                %change tab
                app.TabGroup.SelectedTab=app.magnitudeTab;
                app.add2AvailableTabs(app.magnitudeTab);
                app.magnitudeTab.ForegroundColor='black';
                %initialized
                app.initialize(id);
                app.gainFocus();
            end
            guidata(handles.mainGUI,handles);
        end
        
        function clickLinkCompliant(obj,src,evnt,parent,id)
            %click event on a link for moment adding
            handles=parent.getHandles();
            %check if link stiffness is shown
            app=parent.getApp();
            for i=1:parent.getNoOfLinks()
                %deselect everything
                parent.setSelectLink(i,0);
            end
            
            if logical(obj.linearSpring) || logical(obj.group) 
                uiwait(msgbox('Invalid Link Selected.','Error','error','modal'));
                if ~isempty(app)
                    app.gainFocus();
                end
                return;
            end
            
            if ~isempty(app) && strcmp(app.TabGroup.SelectedTab.Title,'Link') && logical(app.select.Value)
                app.select.FontColor=Colors.StatusComplete.getColor();
                app.select.Text=obj.name;
                handles.link1= id;
                parent.setSelectLink(id,1);
                parent.plotEverything();
                app.gainFocus();
                %change tab
                app.TabGroup.SelectedTab=app.modelType;
                app.add2AvailableTabs(app.modelType);
                app.modelType.ForegroundColor='black';
            elseif ~isempty(obj.geometry)
                 parent.setSelectLink(id,1);
                %open the spring popup
                additional={'id',id};
                parent.setPopFigure('compliantPop',additional);
                parent.plotEverything();
                figure(parent.getPopHandles().compliantGUI);
                handles.helpStatic.String=Module.getString(parent.getMode());
            end
        end
        
        function addLimitConstantForce(obj,src,evnt,parent,id)
            %click event on a dd constant range
            if logical(parent.getBusy())
                return;
            end
            parent.deselectAllLinks();
            handles=parent.getHandles();
            currentID=id;
            for i=1:length(parent.getConstantForceSynthesis().links)
                %if multiple selection is permitted previously selected points will be
                %retained else only current point be made selected
                if  i ~= currentID
                    parent.setSelectLinkConstant(i,0);
                elseif i == currentID
                    parent.setSelectLinkConstant(i,1);
                end
            end
            string={'Length'};
            if logical(obj.linearSpring)
                string(end+1)={'Linear Spring Stiffness'};
            end
            if ~isempty(obj.geometry)
                string(end+1)={'Compliant Flexural Stiffness'};
            end
            %add to the pop up
            handles.typeConstantPopup.Value=1;
            handles.typeConstantPopup.String=string;
            %panel
            handles.rangeValuesPanel.Visible='on';
            handles.rangeValuesPanel.Title='Set Limits for Selected Link';
            parent.setSelectedConstant([2,id]);
            parent.fillLimits();
            parent.plotEverything();
        end
        
        function deselectEverything(obj,parent)
            %deselect everything
            parent.deselectAllLinks();
            parent.deselectAllNodes();
            parent.setSelectedForce(0);
            parent.setSelectedMoment(0);
            parent.setSelectedTorsionSpring(0);
        end
        
        function clickOverview(obj,src,evnt,parent,id)
            %click on a node during overview
            if evnt.Button ~= 1
                obj.deselectEverything(parent);
                parent.setSelectLink(id,1);
                parent.plotEverything();
            end
        end
        
        function obj=clickEvent(obj,mode,parent)
            %node click event
            switch(mode)
                case Module.Overview
                    fnc=@(src,evnt)obj.clickOverview(src,evnt,parent,obj.id);
                case Module.Sketch
                    fnc=[];
                case Module.Model
                    fnc=@(src,evnt)obj.clickLinkModel(src,evnt,parent,obj.id);
                case Module.Constrain
                    fnc=@(src,evnt)obj.clickLinkConstraint(src,evnt,parent,obj.id);
                case Module.Linear
                    fnc=@(src,evnt)obj.clickLinkLinearSpring(src,evnt,parent,obj.id);
                case Module.Torsion
                    fnc=@(src,evnt)obj.clickLinkTorsionSpring(src,evnt,parent,obj.id);
                case Module.Force
                    fnc=@(src,evnt)obj.clickLinkForce(src,evnt,parent,obj.id);
                case Module.Moment
                    fnc=@(src,evnt)obj.clickLinkMoment(src,evnt,parent,obj.id);
                case Module.Compliant
                    fnc=@(src,evnt)obj.clickLinkCompliant(src,evnt,parent,obj.id);
                case Module.Free
                    fnc=[];
                case Module.Range
                    str='Kinematics:Range';
                case Module.Load
                    str='Statics:Load';
                case Module.Distance
                    str='Statics:Distance';
                case Module.MechanicalAdvantage
                    str='Statics:Mechanical Advantage';
                case Module.EnergyPlot
                    str='Statics:Energy Plot';
                case Module.PostProcessing
                    fnc=[];
                case Module.ConstantForceSynthesis
                    switch parent.getConstantForceSynthesis().mode
                        case 1 %reference link
                            fnc=@(src,evnt)parent.changeReferanceConstant(src,evnt,obj.id,1);
                        case 3 %reference linear spring
                            if logical(obj.linearSpring)
                                fnc=@(src,evnt)parent.changeReferanceConstant(src,evnt,obj.id,3);
                            else
                                fnc=[];
                            end
                        case 4%reference compliant
                            if ~isempty(obj.geometry)
                                fnc=@(src,evnt)parent.changeReferanceConstant(src,evnt,obj.id,4);
                            else
                                fnc=[];
                            end
                        case 5 %range
                            fnc=@(src,evnt)obj.addLimitConstantForce(src,evnt,parent,obj.id);
                        case 6 %add distance
                            fnc=@(src,evnt)obj.clickLinkConstantForce(src,evnt,parent,obj.id);
                        otherwise
                            fnc=[];
                    end
                otherwise
                    fnc=[];
                    
            end
            if  isgraphics(obj.line(1,1))
                obj.line(1,1).ButtonDownFcn=fnc;
            end
            %same for joints
            if  isgraphics(obj.line(2,1))
                obj.line(2,1).ButtonDownFcn=fnc;
            end
            if  isgraphics(obj.line(3,1))
                obj.line(3,1).ButtonDownFcn=fnc;
            end
            
        end
        
        function clickLinkConstantForce(obj,src,evnt,parent,id)
            %click during a bistable analysis
            if logical(parent.getBusy()) 
                return;
            end 
            handles=parent.getHandles();
            %check if link mode
            if logical(handles.linkDistanceButton.Value)
                currentID=id;
                %idList for the selected nodes
                for i=1:length(parent.getConstantForceSynthesis().links)
                    %if multiple selection is permitted previously selected points will be
                    %retained else only current point be made selected
                    if  i ~= currentID
                        parent.setSelectLinkConstant(i,0);
                    elseif i == currentID && ( obj.joints(1,1) == Joint.GroundPin || obj.joints(1,2) == Joint.GroundPin)
                        parent.setSelectLinkConstant(i,1);
                    end
                end
                if obj.joints(1,1) == Joint.GroundPin || obj.joints(1,2) == Joint.GroundPin
                    handles.distanceSelect.String='link is selected';
                    handles.distanceSelect.ForegroundColor=Colors.StatusComplete.getColor();
                    parent.setSelectedLinkConstant(id);
                else
                    handles.distanceSelect.String='link is not selected';
                    handles.distanceSelect.ForegroundColor=Colors.StatusIncomplete.getColor();
                end
                %plot
                parent.plotEverything();
            end
        end
        
        
        function selected=getSelected(obj)
            %get the selected value
            selected=obj.selected;
        end
        
        function obj=setSelected(obj,selected)
            %set the selected value
            obj.selected=selected;
        end
        
        function group=getGroup(obj)
            %get the group
            group=obj.group;
        end
        
        function xSection=getCrossSection(obj)
            %get the xSection
            xSection=obj.geometry;
        end
        
        function obj=setCrossSection(obj,xSection)
            %set the xSection
            obj.geometry=xSection;
        end
        
        function obj=setGroup(obj,group)
            %set the group
            obj.group=group;
        end
        
        function obj=makeRigid(obj)
            %make link rigid
            obj.geometry=CrossSection.empty;
        end
        
        function obj=makeCompliant(obj,thickness,width,e,type)
            %make link compliant
            obj.geometry=CrossSection(thickness,width,e,type);
        end
        
        
    end
    
end

