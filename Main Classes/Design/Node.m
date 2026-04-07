classdef Node
    %Design points definations
    properties 
        id;
        value;
        line=gobjects(0);
        string;
        constrained=[0 0];
        selected=0;
        updated=0;
    end
    
    methods
        function obj=Node(id,x,y)
            %constructor
            obj.id=id;
            obj.value=Point(x,y);
        end
        
        function disp(nodes)
            %for displaying nodes
            for i=1:length(nodes)
                nodes(i).id
                nodes(i).value
            end
        end
        
        function node=getNode(obj)
            %return the node
            node=obj.value;
        end
        
        function obj=setNode(obj,value)
            %return the node
            obj.value=value;
        end
        
        function obj=setConstrained(obj,value,index)
            %set the constrained
            obj.constrained(index)=value;
        end
        
        function constrained=getConstrained(obj,index)
            %get the constrained
            constrained=obj.constrained(index);
        end
        
        function id=getID(obj)
            %return id
            id=obj.id;
        end
        
        function obj=setID(obj,id)
            %set id
            obj.id=id;
        end
        
        function updated=getUpdated(obj)
            %return updated
            updated=obj.updated;
        end
        
        function obj=setUpdated(obj,updated)
            %set updated
            obj.updated=updated;
        end
        
        function drawNoGUI(obj,currentWorkspace,fig)
            %draw the nodes without gui
            figure(fig);
            limit=currentWorkspace.getLimit()*0.5;
            obj.line=plot(obj.value.x,obj.value.y,'Marker','+','MarkerSize',10,'Linewidth',1);
            obj.string=text(obj.value.x+limit/25,obj.value.y+limit/25,num2str(obj.id));
            color=Colors.NormalNode.getColor();
            obj.colorNode(color);
        end
        
        function obj=drawNode(obj,currentAxis,limit,mode,parent)
            %draw the node
            if isempty(obj.line) || ~isgraphics(obj.line)  || ~isgraphics(obj.string) ||  isempty(obj.string)
                %new
                obj.line=plot(currentAxis,obj.value.x,obj.value.y,'Marker','+','MarkerSize',10,'Linewidth',1);
                obj.string=text(currentAxis,obj.value.x+limit/30,obj.value.y+limit/25,num2str(obj.id));
            else
                %update
                obj.line.XData=obj.value.x;
                obj.line.YData=obj.value.y;
                obj.string.String=num2str(obj.id);
                obj.string.Position=[obj.value.x+limit/30,obj.value.y+limit/25];
            end
            %color the node
            if logical(obj.selected)
                color=Colors.Selected.getColor();
            elseif (logical(obj.constrained(1)) || ( length(obj.constrained) == 2 && logical(obj.constrained(2)) ) )  && (parent.getMode() == Module.Constrain || parent.getMode() == Module.Sketch || parent.getMode() == Module.Model)
                color=Colors.Constrained.getColor();
            else
                color=Colors.NormalNode.getColor();
            end
            obj=obj.colorNode(color);
            %right click menu
            obj=obj.rightClickMenu(mode,parent);
            %click event
            obj=obj.clickEvent(mode,parent);
        end
        
        
        function obj=drawCBCM(obj,currentAxis)
            %draw the node
            if isempty(obj.line) || ~isgraphics(obj.line)
                %new
                obj.line=plot(currentAxis,obj.value.x,obj.value.y,'Marker','+','MarkerSize',7,'Linewidth',1);
                obj.string=[];
            else
                %update
                obj.line.XData=obj.value.x;
                obj.line.YData=obj.value.y;
                obj.string.String=[];
            end
            obj.line.MarkerEdgeColor=Colors.NormalLink.getColor();
        end
        
        function obj=drawMLM(obj,currentAxis)
            %draw the node
            if isempty(obj.line) || ~isgraphics(obj.line)
                %new
                obj.line=plot(currentAxis,obj.value.x,obj.value.y,'Marker','x','MarkerSize',7,'Linewidth',1);
                obj.string=[];
            else
                %update
                obj.line.XData=obj.value.x;
                obj.line.YData=obj.value.y;
                obj.string.String=[];
            end
            obj.line.MarkerEdgeColor=Colors.NormalLink.getColor();
        end
        
        function obj=deleteNodeDrawing(obj)
            %delete the node drawing
            try
                delete(obj.line);
                delete(obj.string);
            catch
                obj.line=[];
                obj.string=[];
            end
        end
        
        function obj=colorNode(obj,color)
            %color node
            if isgraphics(obj.line)
                obj.line.MarkerEdgeColor=color;
                %color text
                obj.string.Color=color;
            end
        end
        
        function editNode(obj,src,evnt,parent,id)
            %edit node - go to constrain mode
            handles=parent.getHandles();
            parent.setSelectNode(id,1);
            %select the draggable node
            parent.plotEverything();
            %open popup
            additional={num2str(id)};
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            parent.openApp('node',pos(1),pos(2),additional);
        end
        
        function obj=rightClickMenu(obj,mode,parent)
            %right click menu
            handles=parent.getHandles();
            switch(mode)
                case Module.Overview                 
                    menu= uicontextmenu(handles.mainGUI);
                    func=@(src,evnt)obj.editNode(src,evnt,parent,obj.id);
                    uimenu(menu,'Label','Edit','Callback',func);
                    func=@(source,callbackdata)parent.deleteNode(source,callbackdata,obj.id);
                    uimenu(menu,'Label','Delete','Callback',func);
                    obj.line.UIContextMenu=menu;
                    obj.string.UIContextMenu=menu;
                case Module.Sketch
                    menu= uicontextmenu(handles.mainGUI);
                    obj.line.UIContextMenu=menu;
                    obj.string.UIContextMenu=menu;
                case Module.Model
                    menu= uicontextmenu(handles.mainGUI);
                    obj.line.UIContextMenu=menu;
                    obj.string.UIContextMenu=menu;
                case Module.Constrain
                    menu= uicontextmenu(handles.mainGUI);
                    obj.line.UIContextMenu=menu;
                    obj.string.UIContextMenu=menu;                    
                case Module.Linear
                    menu= uicontextmenu(handles.mainGUI);
                    obj.line.UIContextMenu=menu;
                    obj.string.UIContextMenu=menu;
                case Module.Torsion
                    menu= uicontextmenu(handles.mainGUI);
                    obj.line.UIContextMenu=menu;
                    obj.string.UIContextMenu=menu;
                case Module.Force
                    menu= uicontextmenu(handles.mainGUI);
                    obj.line.UIContextMenu=menu;
                    obj.string.UIContextMenu=menu;
                case Module.Moment
                    menu= uicontextmenu(handles.mainGUI);
                    obj.line.UIContextMenu=menu;
                    obj.string.UIContextMenu=menu;
                case Module.Compliant
                    str='Design:Compliant';
                case Module.Free
                    str='Kinematics:Free';
                case Module.Range
                    str='Kinematics:Range';
                case Module.Load
                    str='Statics:Load';
                case Module.Distance
                case Module.MechanicalAdvantage
                    str='Statics:Mechanical Advantage';
                case Module.EnergyPlot
                    str='Statics:Energy Plot';
                case Module.PostProcessing
                    menu= uicontextmenu(handles.mainGUI);
                    m1 = uimenu(menu,'Label',['Set Node-' num2str(obj.id)  ' X Coordinates  as X Axis'],'Callback',@(source,callbackdata)parent.setNodeXAxis(source,callbackdata,obj.id,1));
                    m2 = uimenu(menu,'Label',['Set Node-' num2str(obj.id)  ' Y Coordinates  as X Axis'],'Callback',@(source,callbackdata)parent.setNodeXAxis(source,callbackdata,obj.id,2));
                    m3 = uimenu(menu,'Label',['Add Node-' num2str(obj.id)  ' X Coordinates  to Y Axis'],'Callback',@(source,callbackdata)parent.setNodeYAxis(source,callbackdata,obj.id,1));
                    m4 = uimenu(menu,'Label',['Add Node-' num2str(obj.id)  ' Y Coordinates  to Y Axis'],'Callback',@(source,callbackdata)parent.setNodeYAxis(source,callbackdata,obj.id,2));
                    obj.line.UIContextMenu=menu;
                    obj.string.UIContextMenu=menu;
                    
            end
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
                parent.setSelectNode(id,1);
                parent.plotEverything();
            end
        end
       
        function clickNodeConstrain(obj,src,evnt,parent,id)
            %click event on a node for modelling
            handles=parent.getHandles();
            currentID=id;
            %idList for the selected nodes
            idList=[];
            for i=1:parent.getNoOfNodes()
                %if multiple selection is permitted previously selected points will be
                %retained else only current point be made selected
                if  ~logical(parent.getMultipleSelect) && i ~= currentID
                    parent.setSelectNode(i,0);
                elseif i == currentID
                    parent.setSelectNode(i,1);
                    idList{end+1}=num2str(currentID);
                    currentPointIDListPointer=length(idList);
                elseif logical(parent.getSelectNode(i))
                    idList{end+1}=num2str(i);
                end
            end
            handles.idList=idList;
            parent.updateMainHandles(handles);
            %deselect all links
            parent.deselectAllLinks();
            parent.plotEverything();
            %select the draggable node
            parent.setDraggableNode(id);
            %delete annotation
            delete(handles.annotation);
        end
        
        function clickNodeLinear(obj,src,evnt,parent,id)
            %click event on a node for modelling
            handles=parent.getHandles();
            app=parent.getApp();
            %check if we should return
            if isempty(app) || ~strcmp(app.TabGroup.SelectedTab.Title,'Type') || logical(app.convertButton.Value)
                app.gainFocus();
                return;
            end
            %idList for the selected nodes
            for i=1:parent.getNoOfNodes()
                %deselect everything
                parent.setSelectNode(i,0);
            end
            if logical(app.select1.Value)
                %looking for node 1
                app.select1.FontColor=Colors.StatusComplete.getColor();
                app.select1.Text=['Node ',num2str(id)];
                app.select1.Value=0;
                app.select2.Value=1;
                handles.node1=id;
                parent.setSelectNode(id,1);
                if handles.node2 ~= -1
                    parent.setSelectNode(handles.node2,1);
                end
                parent.plotEverything();
                if sscanf(app.select1.Text,'%*s %d') ~= sscanf(app.select2.Text,  '%*s %d')
                    %change tab
                    app.TabGroup.SelectedTab=app.magnitudeTab;
                    app.add2AvailableTabs(app.magnitudeTab);
                    app.magnitudeTab.ForegroundColor='black';
                end
                app.gainFocus();
            elseif logical(app.select2.Value)
                %looking for node 2
                app.select2.FontColor=Colors.StatusComplete.getColor();
                app.select2.Text=['Node ',num2str(id)];
                app.select1.Value=1;
                app.select2.Value=0;
                handles.node2=id;
                parent.setSelectNode(id,1);
                if handles.node1 ~= -1
                    parent.setSelectNode(handles.node1,1);
                end
                parent.plotEverything();
                if sscanf(app.select1.Text,'%*s %d') ~= sscanf(app.select2.Text,'%*s %d')
                    %change tab
                    app.TabGroup.SelectedTab=app.magnitudeTab;
                    app.add2AvailableTabs(app.magnitudeTab);
                    app.magnitudeTab.ForegroundColor='black';
                end
                app.gainFocus();
            else
                %not necessary
                parent.setSelectNode(id,0);
                parent.plotEverything();
            end
            guidata(handles.mainGUI,handles);
        end
        
        function clickNodeDistance(obj,src,evnt,parent,id)
            %click event on a node for modelling
            if logical(parent.getDistanceAnalysis().busy)
                return;
            end 
            popHandles=parent.getPopHandles();
            if ~isempty(popHandles) && popHandles.type.Value >2
                %idList for the selected nodes
                for i=1:length(parent.getDistanceAnalysis().static.nodes)
                    parent.setSelectNode3(i,0);
                end
                parent.setSelectNode3(id,1);
                popHandles.component.BackgroundColor=Colors.StatusComplete.getColor();
                popHandles.component.String='SELECTED';
                %plot
                parent.plotEverything();
                %draw the distance
                if isnan(str2double(popHandles.target1.String))
                    popHandles.target1.String=num2str(round(parent.getWorkspace().getLimit()*0.05));
                end
                switch popHandles.type.Value
                    case 1
                        type=2;
                        target=1;
                    case 2
                        type=2;
                        target=2;
                    case 3
                        type=1;
                        target=1;
                    case 4
                        type=1;
                        target=2;
                end
                target(2)=str2double(popHandles.target1.String);
                parent.addDistanceAnalysisType(type,target);
                figure(popHandles.distanceGUI);
            end
            
        end
        
        function clickNodeBistable(obj,src,evnt,parent,id)
            %click event on a node during bistable analysis
            if logical(parent.getBistable().busy)
                return;
            end
            %check if it has a slider
            popHandles=parent.getPopHandles();
            if ~isempty(popHandles) && popHandles.type.Value >2
                if logical(parent.getBistable().getSlider(id))
                    %idList for the selected nodes
                    for i=1:length(parent.getBistable().static.nodes)
                        parent.setSelectNode4(i,0);
                    end
                    parent.setSelectNode4(id,1);
                    popHandles.component.BackgroundColor=Colors.StatusComplete.getColor();
                    popHandles.component.String='SELECTED';
                    %plot
                    parent.plotEverything();
                    %try to plot distances
                    if isnan(str2double(popHandles.target1.String))
                        popHandles.target1.String=num2str(-round(parent.getWorkspace().getLimit()*0.05));
                    end
                    if isnan(str2double(popHandles.target2.String))
                        popHandles.target2.String=num2str(round(parent.getWorkspace().getLimit()*0.05));
                    end
                    switch popHandles.type.Value
                        case 1
                            type=2;
                            target=1;
                        case 2
                            type=2;
                            target=2;
                        case 3
                            type=1;
                            target=1;
                    end
                    target(2:3)=[str2double(popHandles.target1.String),str2double(popHandles.target2.String)];
                    parent.addBistableType(type,target);
                    
                else
                    uiwait(msgbox('Select a slider node.','Error','error','modal'));
                end
                figure(popHandles.energyGUI);
            end
        end
        
        function clickNodeKinematicSynthesis(obj,src,evnt,parent,id)
            %click event on a node for kinematic synthesis
            handles=parent.getHandles();
            mode=parent.getKinematicSynthesisMode();
            %check if node mode
            currentID=id;    
            switch mode
                case 1
                    %plot
                    %idList for the selected nodes
                    parent.updateKinematicSynthesisNode(id);
                    parent.plotEverything();
                case 3
                    %idList for the selected nodes
                    parent.setSelectedNodeKinematicSynthesis(id);
                    %panel
                    handles.typeConstantPopup.Value=1;
                    handles.rangeValuesPanel.Visible='on';
                    handles.typeConstantPopup.String={'x';'y';};
                    handles.rangeValuesPanel.Title='Set Limits for Selected Node';
                    parent.fillLimitsKinematicSynthesis();
            end
            
        end
        
        function clickNodeFlexuralSynthesis(obj,src,evnt,parent,id)
            %click event on a node during bistable analysis
            if logical(parent.getBusy())
                return;
            end
            popHandles=parent.getPopHandles();
            if ~isempty(popHandles) && popHandles.type.Value >2
                %idList for the selected nodes
                for i=1:length(parent.getFlexuralStiffnessSynthesis().static.nodes)
                    parent.setSelectNode6(i,0);
                end
                parent.setSelectNode6(id,1);
                popHandles.component.BackgroundColor=Colors.StatusComplete.getColor();
                popHandles.component.String='SELECTED';
                %plot
                parent.plotEverything();
                %draw the distance
                if ~isnan(str2double(popHandles.target1.String))  && strcmp(popHandles.component.String,'SELECTED')
                    switch popHandles.type.Value
                        case 1
                            type=2;
                            target=1;
                        case 2
                            type=2;
                            target=2;
                        case 3
                            type=1;
                            target=1;
                        case 4
                            type=1;
                            target=2;
                    end
                    target(2)=str2double(popHandles.target1.String);
                    parent.addDistanceAnalysisType(type,target);
                end
               figure(popHandles.energyGUI); 
            end
            
        end
        
        function clickNodeConstantForce(obj,src,evnt,parent,id)
            %click event on a node during constant force
            if logical(parent.getBusy())
                return;
            end 
            %check if it has a slider
            if logical(parent.getConstantForceSynthesis().getSlider(id))
                handles=parent.getHandles();
                %check if node mode
                if logical(handles.nodeDistanceButton.Value)
                    currentID=id;
                    %idList for the selected nodes
                    for i=1:length(parent.getConstantForceSynthesis().nodes)
                        %if multiple selection is permitted previously selected points will be
                        %retained else only current point be made selected
                        if  i ~= currentID
                            parent.setSelectNodeConstant(i,0);
                        elseif i == currentID
                            parent.setSelectNodeConstant(i,1);
                        end
                    end
                    handles.distanceSelect.String='node is selected';
                    handles.distanceSelect.ForegroundColor=Colors.StatusComplete.getColor();
                    parent.setSelectedNodeConstant(id);
                    %plot
                    parent.plotEverything();
                end
            end
        end
        
        function clickNodeConstantForce2(obj,src,evnt,parent,id)
            %click event on a node during constant force
            if logical(parent.getBusy())
                return;
            end
            parent.deselectAllLinks();
            handles=parent.getHandles();
            %check if node mode
            currentID=id;
            %idList for the selected nodes
            for i=1:length(parent.getConstantForceSynthesis().nodes)
                %if multiple selection is permitted previously selected points will be
                %retained else only current point be made selected
                if  i ~= currentID
                    parent.setSelectNodeConstant(i,0);
                elseif i == currentID
                    parent.setSelectNodeConstant(i,1);
                end
            end
            %panel
            handles.typeConstantPopup.Value=1;
            handles.rangeValuesPanel.Visible='on';
            handles.typeConstantPopup.String={'x';'y';};
            handles.rangeValuesPanel.Title='Set Limits for Selected Node';
            parent.setSelectedConstant([1,id]);
            parent.fillLimits();
            parent.plotEverything();
        end
        
        function obj=clickEvent(obj,mode,parent)
            %node click event
            switch(mode)
                case Module.Overview
                    fnc=@(src,evnt)obj.clickOverview(src,evnt,parent,obj.id);
                case Module.Sketch
                    fnc=[];
                case Module.Model
                    fnc=[];
                    %fnc=@(src,evnt)obj.clickNodeModel(src,evnt,parent,obj.id);
                case Module.Constrain
                    fnc=@(src,evnt)obj.clickNodeConstrain(src,evnt,parent,obj.id);
                case Module.Linear
                    fnc=@(src,evnt)obj.clickNodeLinear(src,evnt,parent,obj.id);
                case Module.Torsion
                    fnc=[];
                case Module.Force
                    fnc=[];
                case Module.Moment
                    fnc=[];
                case Module.Compliant
                    fnc=[];
                case Module.Free
                    fnc=[];
                case Module.Range
                    fnc=[];
                case Module.Load
                    fnc=[];
                case Module.Distance
                    fnc=@(src,evnt)obj.clickNodeDistance(src,evnt,parent,obj.id);
                case Module.MechanicalAdvantage
                    fnc=[];
                case Module.EnergyPlot
                    fnc=@(src,evnt)obj.clickNodeBistable(src,evnt,parent,obj.id);
                case Module.PostProcessing
                    fnc=[];
                case Module.KinematicSynthesis
                    fnc=@(src,evnt)obj.clickNodeKinematicSynthesis(src,evnt,parent,obj.id);
                case Module.FlexuralStiffnessSynthesis
                    fnc=@(src,evnt)obj.clickNodeFlexuralSynthesis(src,evnt,parent,obj.id);
                case Module.BiStableSynthesis
                    fnc=[];
                case Module.ConstantForceSynthesis
                    switch parent.getConstantForceSynthesis().mode
                        case 5 %add range values
                            fnc=@(src,evnt)obj.clickNodeConstantForce2(src,evnt,parent,obj.id);
                        case 6 %add distance
                            fnc=@(src,evnt)obj.clickNodeConstantForce(src,evnt,parent,obj.id);
                        otherwise
                            fnc=[];
                    end
                otherwise
                    fnc=[];
            end
            obj.line.ButtonDownFcn=fnc;
            obj.string.ButtonDownFcn=fnc;
            
        end
        
        function selected=getSelected(obj)
            %get the selected value
            selected=obj.selected;
        end
        
        function obj=setSelected(obj,selected)
            %set the selected value
            obj.selected=selected;
        end
    end
    
end

