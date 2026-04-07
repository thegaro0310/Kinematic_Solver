classdef Moment
    %Force class during design
    
    properties
        id;
        link;
        line=gobjects(1,2);
        text=gobjects(1,1);
        magnitude=0;
        node;
        distance;
        selected=0;
        active=1;
        moved=0;
        angle0=0;
    end
    
    methods
        
        function   obj=Moment(id,link,magnitude,distance)
            %constructor
            obj.id=id;
            obj.link=link;
            obj.magnitude=magnitude;
            obj.distance=distance;
        end
        
        function   obj=changeMoment(obj,magnitude,distance)
            %constructor
            delete(obj.line);
            delete(obj.text);
            obj.magnitude=magnitude;
            obj.distance=distance;
        end
        
        function obj=updateMagnitude(obj,value)
            %update magnitude
            obj.magnitude=value;
        end
        
        function id=getID(obj)
            %get it
            id=obj.id;
        end
        
        function obj=setID(obj,id)
            %set id
            obj.id=id;
        end
        
        function link=getLink(obj)
            %get it
            link=obj.link;
        end
        
        function obj=setLink(obj,link)
            %set link
            obj.link=link;
        end
        
        function selected=getSelected(obj)
            %get selected
            selected=obj.selected;
        end
        
        function obj=setSelected(obj,selected)
            %set selected
            obj.selected=selected;
        end
        
        
        function obj=deleteDrawing(obj)
            %delete drawing
            for i=1:size(obj.line,1)
                for j=1:2
                    if ~isempty(obj.line(i,j)) && isgraphics(obj.line(i,j))
                        delete(obj.line(i,j));
                    end
                end
            end
            if ~isempty(obj.text(1)) && isgraphics(obj.text(1))
                delete(obj.text(1));
            end

        end  
        
         function obj=drawStatic(obj,parent,nodes,allBeams,power)
            %draw the force
            node=allBeams{obj.link}.nodes;
            node1=nodes(node(1)).getNode() ;
            node2=nodes(node(2)).getNode();
            lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
            loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
            unit=[loadString '.' lengthString] ;
            handles=parent.getHandles();
            %calculate node
            node=Point(node1.x+(node2.x-node1.x)/100*obj.distance,node1.y+(node2.y-node1.y)/100*obj.distance);
            obj=obj.drawMoment(parent.getWorkspace(),node,power,unit,handles.designPlot);
            obj.text.Visible='on';
            if logical(obj.active)
                color=Colors.NormalNode.getColor();
            else
                color=Colors.Inactive.getColor();
            end
            rightClick= uicontextmenu(handles.mainGUI);
            if parent.getMode() == Module.Distance
                m1 = uimenu(rightClick,'Label','Set as the Unknown Force','Callback',@(source,callbackdata)parent.rightClickLoad(source,callbackdata,obj.id,1,2));
                m1 = uimenu(rightClick,'Label','Activate','Callback',@(source,callbackdata)parent.rightClickLoad(source,callbackdata,obj.id,2,2));
                m1 = uimenu(rightClick,'Label','Activate All','Callback',@(source,callbackdata)parent.rightClickLoad(source,callbackdata,obj.id,3,2));
                m1 = uimenu(rightClick,'Label','Deactivate All','Callback',@(source,callbackdata)parent.rightClickLoad(source,callbackdata,obj.id,4,2));
            elseif parent.getMode() == Module.Load
                m1 = uimenu(rightClick,'Label','Active','Callback',@(source,callbackdata)parent.rightClickLoadLoad(source,callbackdata,obj.id,1,2));
                if logical(obj.active)
                    m1.Checked='on';
                else
                    m1.Checked='off';
                end
                m1 = uimenu(rightClick,'Label','Activate All','Callback',@(source,callbackdata)parent.rightClickLoadLoad(source,callbackdata,obj.id,2,2));
                m1 = uimenu(rightClick,'Label','Deactivate All','Callback',@(source,callbackdata)parent.rightClickLoadLoad(source,callbackdata,obj.id,3,2));
            elseif parent.getMode() == Module.MechanicalAdvantage
                m1 = uimenu(rightClick,'Label','Set as Input Load','Callback',@(source,callbackdata)parent.rightClickAdvantage(source,callbackdata,obj.id,1,2));
                m1 = uimenu(rightClick,'Label','Set as Output Load','Callback',@(source,callbackdata)parent.rightClickAdvantage(source,callbackdata,obj.id,2,2));
            elseif parent.getMode() ==  Module.PostProcessing
                m1 = uimenu(rightClick,'Label',['Set Moment-' num2str(obj.id)  ' Magnitude as X Axis'],'Callback',@(source,callbackdata)parent.setMomentRightClick(source,callbackdata,obj.id,1,1));
                m1 = uimenu(rightClick,'Label',['Add Moment-' num2str(obj.id)  ' Magnitude  to Y Axis'],'Callback',@(source,callbackdata)parent.setMomentRightClick(source,callbackdata,obj.id,1,2));
            end
            %color
            if ~isempty(obj.text(1)) && isgraphics(obj.text(1))
                obj.text(1).Color=color;
                obj.text(1).ButtonDownFcn=[];
                obj.text(1).UIContextMenu=rightClick;
            end
            for i=1:size(obj.line,1)
                for j=1:2
                    if ~isempty(obj.line(i,j)) && isgraphics(obj.line(i,j))
                        obj.line(i,j).Color=color;
                        obj.line(i,j).ButtonDownFcn=[];
                        obj.line(i,j).UIContextMenu=rightClick;
                    end
                end
            end
        end
        
        
        function obj=drawNoGUI(obj,mainFig,workspace,nodes,allBeams,power)
            %draw the force in the developper mode
            node=allBeams{obj.link}.nodes;
            node1=nodes(node(1)).getNode() ;
            node2=nodes(node(2)).getNode();
            %calculate node
            node=Point(node1.x+(node2.x-node1.x)/100*obj.distance,node1.y+(node2.y-node1.y)/100*obj.distance);
            obj=obj.drawMoment(workspace,node,power,[],mainFig);
            color=Colors.NormalLink.getColor();
            %color
            obj.text(1).Color=color;      
            for i=1:size(obj.line,1)
                for j=1:2
                    obj.line(i,j).Color=color;
                end
            end
        end
        
        function editMoment(obj,src,evnt,parent,id)
            %edit moment 
            handles=parent.getHandles();
            parent.setSelectedMoment(id);
            %select the draggable node
            parent.plotEverything();
            %open the moment popup
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            parent.openApp('momentApp',pos(1),pos(2),{});
            app=parent.getApp();
            app.updateMoment(id);
            app.magnitudeTab.ForegroundColor='black';
            app.typeTab.ForegroundColor=Colors.Inactive.getColor();
            app.gainFocus();
            parent.plotEverything();
        end
        
        function obj=draw(obj,parent)
            %draw the force
            handles=parent.getHandles();
            lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
            loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
            unit=[loadString '.' lengthString] ;
            currentWorkspace=parent.getWorkspace();
            nodes=parent.getLinkNodes(obj.link);
            node1=parent.getNode(nodes(1));
            node2=parent.getNode(nodes(2));
            %calculate node
            node=Point(node1.x+(node2.x-node1.x)/100*obj.distance,node1.y+(node2.y-node1.y)/100*obj.distance);
            obj=obj.drawMoment(currentWorkspace,node,100,unit,handles.designPlot);
            if logical(obj.selected)
                color=Colors.Selected.getColor();
            else
                color=Colors.NormalLink.getColor();
            end
            rightClick= uicontextmenu(handles.mainGUI);
            if parent.getMode == Module.Overview
                uimenu(rightClick,'Label','Edit','Callback',@(source,callbackdata) obj.editMoment(source,callbackdata,parent,obj.id));
                uimenu(rightClick,'Label','Delete','Callback',@(source,callbackdata) parent.deleteMoment(source,callbackdata,obj.id));
            end
            %color
            if ~isempty(obj.text(1)) && isgraphics(obj.text(1))
                obj.text(1).Color=color;
                obj.text(1).ButtonDownFcn=@(src,evnt)obj.clickFunc(src,evnt,parent,obj.id);
                obj.text(1).UIContextMenu=rightClick;
            end
            for i=1:size(obj.line,1)
                for j=1:2
                    if ~isempty(obj.line(i,j)) && isgraphics(obj.line(i,j))
                        obj.line(i,j).Color=color;
                        obj.line(i,j).ButtonDownFcn=@(src,evnt)obj.clickFunc(src,evnt,parent,obj.id);
                        obj.line(i,j).UIContextMenu=rightClick;
                    end
                end
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
        
        function clickFunc(obj,src,evnt,parent,id)
            %click on a force
            if parent.getMode == Module.Overview
                if evnt.Button ~= 1
                    obj.deselectEverything(parent);
                    parent.setSelectedMoment(id);
                    parent.plotEverything();
                end
            else
                parent.deselectAllLinks();
                parent.deselectAllNodes();
                %select force
                parent.setSelectedMoment(id);
                %open the spring popup
                app=parent.getApp();
                app.updateMoment(id);
                app.magnitudeTab.ForegroundColor='black';
                app.typeTab.ForegroundColor=Colors.Inactive.getColor();
                app.gainFocus();
                parent.plotEverything();
            end
        end
        
        
        
        function obj=drawMoment(obj,currentWorkspace,node,power,unit,ax)
            %function for moemnt
            limit=currentWorkspace().getLimit()*0.5;
            stretch=limit/20;
            %force in x direction
            angles=linspace(3.1*pi/2,pi/2+2*pi,100);
            x=cos(angles)*stretch;
            y=sin(angles)*stretch*1.5;
            %negative moment
            if obj.magnitude <0
                rotation=[cos(pi) -sin(pi);sin(pi) cos(pi)];
                for i=1:length(x)
                    pointNew=rotation*[x(i);y(i)];
                    x(i)=node.x*1.02+pointNew(1);
                    y(i)=node.y+pointNew(2);
                end
                extra=0;
            else
                %if positive
                extra=stretch*1.1;
                x=x+node.x*0.98;
                y=y+node.y;
            end
            if isempty(obj.line(1,1)) || ~isgraphics(obj.line(1,1))
                obj.line(1,1)=plot(ax,x,y,'linewidth',1.5);
            else
                obj.line(1,1).XData=x;
                obj.line(1,1).YData=y;
            end
            %straight line is drawn
            %now draw the hat
            if obj.magnitude >0
                multiplier=1.2;
            else
                multiplier=0.8;
            end
            angle=atan2(y(end)-y(end-1),x(end)-x(end-1))-pi*multiplier/2;
            rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
            pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch] ;
            for i=1:3
                pointNew=rotation*[pointsX(i);pointsY(i)];
                if obj.magnitude >0
                    pointsX(i)=x(end)+pointNew(1);pointsY(i)=y(end)+pointNew(2);
                else
                    pointsX(i)=x(1)+pointNew(1);pointsY(i)=y(1)+pointNew(2);
                end
            end
            if isempty(obj.line(1,2)) || ~isgraphics(obj.line(1,2))
                obj.line(1,2)=plot(ax,pointsX,pointsY,'linewidth',1.5);
                %plot the text
                obj.text(1)=text(ax,x(end)+extra,node.y, [num2str(abs(obj.magnitude)*power/100),' ',unit]);
            else
                obj.line(1,2).XData=pointsX;
                obj.line(1,2).XData=pointsX;
                obj.line(1,2).YData=pointsY;
                obj.text(1).Position=[x(end)+extra,node.y];
                obj.text(1).String=[num2str(abs(obj.magnitude)*power/100),' ',unit];
                
            end
        end
        
        function disp(moments)
            for i=1:length(moments)
                moments(i).magnitude
            end
        end
    end
end
