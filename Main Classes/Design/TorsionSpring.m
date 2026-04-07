classdef TorsionSpring
    %torsion spring class
    properties
        id;
        link1;
        link2;
        node;
        stiffness;
        theta0=[];
        line=gobjects(1,1);
        text=gobjects(1,1);
        selected=0;
        active=1;
        drawSpring=1;
    end
    
    methods
        function   obj=TorsionSpring(id,link1,link2,node,stiffness)
            %constructor
            obj.id=id;
            obj.link1=link1;
            obj.link2=link2;
            obj.stiffness=stiffness;
            obj.node=node;
        end
        
        function id=getID(obj)
            %get id
            id=obj.id;
        end
        
        function obj=setID(obj,id)
            %set id
            obj.id=id;
        end
        
        function obj=setInitialAngle(obj,linkList)
            %set the initial angles
            obj.theta0=[0,0];
            if obj.link1 > 0
                obj.theta0(1,1)=linkList{obj.link1}.theta0;
            end
            if obj.link2>0
                 obj.theta0(1,2)=linkList{obj.link2}.theta0;
            end
        end
        
        function selected=getSelected(obj)
            %get selected
            selected=obj.selected;
        end
        
        function obj=setSelected(obj,selected)
            %set selected
            obj.selected=selected;
        end
        
        function stiffness=getStiffness(obj)
            %get stifness
            stiffness=obj.stiffness;
        end
        
        function obj=setStiffness(obj,stiffness)
            %set stifness
            obj.stiffness=stiffness;
        end
        
        function obj=drawNoGUI(obj,fig,currentWorkspace,nodes)
            %draw without GUI mode
            limit=currentWorkspace.getLimit()*0.4;
            angles = linspace(0,9*pi,100);
            x = angles.*cos(angles)/30;
            y = angles.*sin(angles)/30;
            node=nodes(obj.node).getNode();
            line=plot(fig,x*(limit/15)+node.x,y*(limit/15)+node.y);
            textLine=text(fig,node.x+limit/15,node.y+limit/15,num2str(obj.stiffness) );
            textLine.Color=Colors.NormalLink.getColor();
            line.Color=Colors.NormalLink.getColor();
        end
        
        function obj=drawStatic(obj,parent,nodes,rigidLinkList)
            %draw the torsion spring during statics
            if logical(obj.drawSpring)
                angles = linspace(0,9*pi,100);
                x = angles.*cos(angles)/30;
                y = angles.*sin(angles)/30;
                node=nodes(obj.node).getNode();
                limit=parent.getWorkspace.getLimit()*0.4;
                handles=parent.getHandles();
                menu= uicontextmenu(handles.mainGUI);
                lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
                if ~isgraphics(obj.line)
                    obj.line=plot(x*(limit/15)+node.x,y*(limit/15)+node.y);
                    unit=[loadString '.' lengthString '/rad'] ;
                    obj.text=text(node.x+limit/15,node.y+limit/15,[num2str(obj.stiffness),' ',unit] );
                    obj.line.ButtonDownFcn=[];
                    obj.text.ButtonDownFcn=[];
                    obj.line.UIContextMenu=menu;
                    obj.text.UIContextMenu=menu;
                else
                    unit=[loadString '.' lengthString '/rad'] ;
                    obj.line.XData=x*(limit/15)+node.x;
                    obj.line.YData=y*(limit/15)+node.y;
                    obj.text.Position=[node.x+limit/15,node.y+limit/15];
                    obj.text.String=[num2str(obj.stiffness),' ',unit];
                    obj.line.ButtonDownFcn=[];
                    obj.text.ButtonDownFcn=[];
                end
                if logical(obj.selected)
                    obj.text.Color=Colors.Selected.getColor();
                    obj.line.Color=Colors.Selected.getColor();
                else
                    obj.text.Color=Colors.NormalLink.getColor();
                    obj.line.Color=Colors.NormalLink.getColor();
                end
            end
        end
        
        function editTorsionSpring(obj,src,evnt,parent,id)
            %edit force 
            handles=parent.getHandles();
            parent.setSelectedTorsionSpring(id);
            %select the draggable node
            parent.plotEverything();
            %open popup
            %open the spring popup
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            parent.openApp('torsionSpringClick',pos(1),pos(2),{});
            app=parent.getApp();
            app.updateSpring(id);
            app.magnitudeTab.ForegroundColor='black';
            app.typeTab.ForegroundColor=Colors.Inactive.getColor();
            parent.plotEverything();
            app.gainFocus();
        end
        
        function obj=draw(obj,parent)
            %draw the torsion spring
            angles = linspace(0,9*pi,100);
            x = angles.*cos(angles)/30;
            y = angles.*sin(angles)/30;
            node=parent.getNode(obj.node);
            limit=parent.getWorkspace.getLimit()*0.4;
            handles=parent.getHandles();
            menu= uicontextmenu(handles.mainGUI);
            if parent.getMode == Module.Overview
                uimenu(menu,'Label','Edit','Callback',@(source,callbackdata) obj.editTorsionSpring(source,callbackdata,parent,obj.id));
                uimenu(menu,'Label','Delete','Callback',@(source,callbackdata) parent.deleteTorsionSpring(source,callbackdata,obj.id));
            end
            lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
            loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
            if ~isgraphics(obj.line)
                obj.line=plot(handles.designPlot,x*(limit/15)+node.x,y*(limit/15)+node.y);
                unit=[loadString '.' lengthString '/rad'] ;
                obj.text=text(handles.designPlot,node.x+limit/15,node.y+limit/15,[num2str(obj.stiffness),' ',unit] );
                obj.line.UIContextMenu=menu;
                obj.text.UIContextMenu=menu;
            else
                unit=[loadString '.' lengthString '/rad'] ;
                obj.line.XData=x*(limit/15)+node.x;
                obj.line.YData=y*(limit/15)+node.y;
                obj.text.Position=[node.x+limit/15,node.y+limit/15];
                obj.text.String=[num2str(obj.stiffness),' ',unit];
            end
            switch(parent.getMode())
                case {Module.Torsion,Module.Overview}
                    obj.line.ButtonDownFcn=@(src,evnt)obj.clickTorsionSpring(src,evnt,parent,obj.id);
                    obj.text.ButtonDownFcn=@(src,evnt)obj.clickTorsionSpring(src,evnt,parent,obj.id);
                case Module.ConstantForceSynthesis
                    obj.line.ButtonDownFcn=@(src,evnt)parent.changeReferanceConstant(src,evnt,obj.id,2);
                    obj.text.ButtonDownFcn=@(src,evnt)parent.changeReferanceConstant(src,evnt,obj.id,2);
            end
            if logical(obj.selected)
                obj.text.Color=Colors.Selected.getColor();
                obj.line.Color=Colors.Selected.getColor();
            else
                obj.text.Color=Colors.NormalLink.getColor();
                obj.line.Color=Colors.NormalLink.getColor();
            end
        end
        
        function obj=drawPRB(obj,ax,workspace)
            %draw during PRB
            point=obj.node;
            limit=workspace.getLimit()*0.2;
            angles = linspace(0,9*pi,100);
            x = angles.*cos(angles)/30;
            y = angles.*sin(angles)/30;
            lengthString=LengthUnit.getString(workspace.unitLength);
            loadString=ForceUnit.getString(workspace.unitForce);
            if ~isgraphics(obj.line)
                obj.line=plot(ax,x*(limit/15)+point.x,y*(limit/15)+point.y);
                unit=[loadString '.' lengthString '/rad'] ;
                obj.text=text(ax,point.x+limit/15,point.y+limit/15,[num2str(obj.stiffness),' ',unit],'FontSize', 8 );
            else
                unit=[loadString '.' lengthString '/rad'] ;
                obj.line.XData=x*(limit/15)+point.x;
                obj.line.YData=y*(limit/15)+point.y;
                obj.text.Position=[point.x+limit/15,point.y+limit/15];
                obj.text.String=[num2str(obj.stiffness),' ',unit];
            end
            obj.text.Color=Colors.Inactive.getColor();
            obj.line.Color=Colors.Inactive.getColor();
        end
        
        function obj=drawTorsionSpring(obj,nodes,parent)
            %draw the torsion spring
            angles = linspace(0,9*pi,100);
            x = angles.*cos(angles)/30;
            y = angles.*sin(angles)/30;
            node=nodes(obj.node).getNode();
            limit=parent.getWorkspace.getLimit()*0.4;
            handles=parent.getHandles();
            menu= uicontextmenu(handles.mainGUI);
            lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
            loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
            if ~isgraphics(obj.line)
                obj.line=plot(handles.designPlot,x*(limit/15)+node.x,y*(limit/15)+node.y);
                unit=[loadString '.' lengthString '/rad'] ;
                obj.text=text(handles.designPlot,node.x+limit/15,node.y+limit/15,[num2str(obj.stiffness),' ',unit] );
                obj.line.UIContextMenu=menu;
                obj.text.UIContextMenu=menu;
            else
                unit=[loadString '.' lengthString '/rad'] ;
                obj.line.XData=x*(limit/15)+node.x;
                obj.line.YData=y*(limit/15)+node.y;
                obj.text.Position=[node.x+limit/15,node.y+limit/15];
                obj.text.String=[num2str(obj.stiffness),' ',unit];
            end
            switch(parent.getMode())
                case Module.Torsion
                    obj.line.ButtonDownFcn=@(src,evnt)obj.clickTorsionSpring(src,evnt,parent,obj.id);
                    obj.text.ButtonDownFcn=@(src,evnt)obj.clickTorsionSpring(src,evnt,parent,obj.id);
                case Module.ConstantForceSynthesis
                    switch parent.getConstantForceSynthesis().mode
                        case 2
                            obj.line.ButtonDownFcn=@(src,evnt)parent.changeReferanceConstant(src,evnt,obj.id,2);
                            obj.text.ButtonDownFcn=@(src,evnt)parent.changeReferanceConstant(src,evnt,obj.id,2);
                        case 5
                            obj.line.ButtonDownFcn=@(src,evnt)obj.setRangeConstant(src,evnt,parent,obj.id);
                            obj.text.ButtonDownFcn=@(src,evnt)obj.setRangeConstant(src,evnt,parent,obj.id);
                    end
            end
            if logical(obj.selected)
                obj.text.Color=Colors.Selected.getColor();
                obj.line.Color=Colors.Selected.getColor();
            else
                obj.text.Color=Colors.NormalNode.getColor();
                obj.line.Color=Colors.NormalNode.getColor();
            end
        end
        
        function obj=deleteDrawing(obj)
            %delete drawing
            delete(obj.line);
            delete(obj.text);
        end
        
        function node=getNode(obj)
            %get node
            node=obj.node;
        end
        
        function obj=setNode(obj,node)
            %set node
            obj.node=node;
        end
        
        function links=getLinks(obj)
            %get links
            links=[obj.link1 obj.link2];
        end
        
        function obj=setLinks(obj,links)
            %set links
            obj.link1=links(1);
            obj.link2=links(2);
        end
        
        function deselectEverything(obj,parent)
            %deselect everything
            parent.deselectAllLinks();
            parent.deselectAllNodes();
            parent.setSelectedForce(0);
            parent.setSelectedMoment(0);
            parent.setSelectedTorsionSpring(0);
        end
        
        function clickTorsionSpring(obj,src,evnt,parent,id)
            %clicking a torsion spring
            if parent.getMode == Module.Overview
                if evnt.Button ~= 1
                    obj.deselectEverything(parent);
                    parent.setSelectedTorsionSpring(id);
                    parent.plotEverything();
                end
            else
                handles=parent.getHandles();
                parent.setSelectedTorsionSpring(id);
                %open the spring popup
                app=parent.getApp();
                app.updateSpring(id);
                app.magnitudeTab.ForegroundColor='black';
                app.typeTab.ForegroundColor=Colors.Inactive.getColor();
                parent.plotEverything();
                app.gainFocus();
                handles.helpStatic.String=Module.getString(parent.getMode());
            end
        end
        
        function setRangeConstant(obj,src,evnt,parent,id)
            %clicking a torsion spring for range 
            if logical(parent.getBusy())
                return;
            end
            handles=parent.getHandles();
            parent.deselectAllLinks();
            currentID=id;
            for i=1:length(parent.getConstantForceSynthesis().torsionSprings)
                %if multiple selection is permitted previously selected points will be
                %retained else only current point be made selected
                if  i ~= currentID
                    parent.setSelectTorsionConstant(i,0);
                elseif i == currentID
                    parent.setSelectTorsionConstant(i,1);
                end
            end
            %add to the pop up
            handles.typeConstantPopup.Value=1;
            handles.typeConstantPopup.String='Torsion Spring Stiffness';
            %panel
            handles.rangeValuesPanel.Visible='on';
            handles.rangeValuesPanel.Title='Set Limits for Selected Torsion Spring';
            parent.setSelectedConstant([3,id]);
            parent.fillLimits();
            parent.plotEverything();
        end
        
        function obj=colorSpring(obj,color)
            %color the spring
            obj.text.Color=color;
            obj.line.Color=color;
        end
        
    end
    
end

