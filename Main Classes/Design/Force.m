classdef Force
    %Force class during design
    
    properties
        id;
        link;
        line=gobjects(2,2);
        text=gobjects(2,1);
        xValue=0;
        yValue=0;
        magnitude=0;
        angle=0;
        distance;
        follower=0;
        selected=0;
        active=1;
        moved=0;
        path;
        kinematicEquations;
        initialDistanceX=0;
        initialDistanceY=0;
        %for followers
        xForceArray;
        yForceArray;
        node;
    end
    
    methods
        
        function   obj=Force(id,link,xValue,yValue,magnitude,angle,distance,follower)
            %constructor
            obj.id=id;
            obj.link=link;
            obj.xValue=xValue;
            obj.yValue=yValue;
            obj.magnitude=magnitude;
            obj.distance=distance;
            obj.follower=follower;
            obj.angle=angle*pi/180; 
        end
        
        function   obj=changeForce(obj,xValue,yValue,magnitude,angle,distance,follower)
            %constructor
            delete(obj.line);
            delete(obj.text);
            obj.xValue=xValue;
            obj.yValue=yValue;
            obj.magnitude=magnitude;
            obj.distance=distance;
            obj.follower=follower;
            obj.angle=angle*pi/180;
        end
        
        function obj=updateXValue(obj,xValue)
            %update x Value
            obj.xValue=xValue;
        end
        
        function obj=updateYValue(obj,yValue)
            %update y Value
            obj.yValue=yValue;
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
        
        function follower=getFollower(obj)
            %get follower property
            follower=obj.follower;
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
            if ~isempty(obj.text(2)) && isgraphics(obj.text(2))
                delete(obj.text(2));
            end
        end
        
        
        function obj=drawNoGUI(obj,mainFig,workspace,nodes,allBeams,power)
            %draw the force
            node=allBeams{obj.link}.nodes;
            node1=nodes(node(1)).getNode() ;
            node2=nodes(node(2)).getNode();
            %calculate node
            node=Point(node1.x+(node2.x-node1.x)/100*obj.distance,node1.y+(node2.y-node1.y)/100*obj.distance);
            if logical(obj.follower)
                %follower
                angleLink=atan2(node2.y-node1.y,node2.x-node1.x)+obj.angle;
                obj=obj.drawFollowerNoGUI(mainFig,workspace,node,angleLink,power);
            else
                %regular
                obj=obj.drawForceNoGUI(mainFig,workspace,node,power);
            end
            %color
            if logical(obj.active)
                color=Colors.NormalLink.getColor();
            else
                color=Colors.Inactive.getColor();
            end
            if ~isempty(obj.text(1)) && isgraphics(obj.text(1))
                obj.text(1).Color=color;
            end
            if ~isempty(obj.text(2)) && isgraphics(obj.text(2))
                obj.text(2).Color=color;
            end
            for i=1:size(obj.line,1)
                for j=1:2
                    if ~isempty(obj.line(i,j)) && isgraphics(obj.line(i,j))
                        obj.line(i,j).Color=color;
                    end
                end
            end
        end
        
        
        function obj=drawStatic(obj,parent,nodes,allBeams,power)
            %draw the force
            handles=parent.getHandles();
            node=allBeams{obj.link}.nodes;
            node1=nodes(node(1)).getNode() ;
            node2=nodes(node(2)).getNode();
            %calculate node
            node=Point(node1.x+(node2.x-node1.x)/100*obj.distance,node1.y+(node2.y-node1.y)/100*obj.distance);
            if logical(obj.follower)
                %follower
                angleLink=atan2(node2.y-node1.y,node2.x-node1.x)+obj.angle;
                obj=obj.drawFollower(parent,node,angleLink,power);
            else
                %regular
                obj=obj.drawForce(parent,node,power,handles.designPlot());
            end
            if logical(obj.active)
                color=Colors.NormalLink.getColor();
            else
                color=Colors.Inactive.getColor();
            end
            rightClick= uicontextmenu(handles.mainGUI);
            if parent.getMode() == Module.Distance
                m1 = uimenu(rightClick,'Label','Set as the Unknown Force','Callback',@(source,callbackdata)parent.rightClickLoad(source,callbackdata,obj.id,1,1));
                m1 = uimenu(rightClick,'Label','Activate','Callback',@(source,callbackdata)parent.rightClickLoad(source,callbackdata,obj.id,2,1));
                m1 = uimenu(rightClick,'Label','Activate All','Callback',@(source,callbackdata)parent.rightClickLoad(source,callbackdata,obj.id,3,1));
                m1 = uimenu(rightClick,'Label','Deactivate All','Callback',@(source,callbackdata)parent.rightClickLoad(source,callbackdata,obj.id,4,1));
            elseif parent.getMode() == Module.Load
                m1 = uimenu(rightClick,'Label','Active','Callback',@(source,callbackdata)parent.rightClickLoadLoad(source,callbackdata,obj.id,1,1));
                if logical(obj.active)
                    m1.Checked='on';
                else
                    m1.Checked='off';
                end
                m1 = uimenu(rightClick,'Label','Activate All','Callback',@(source,callbackdata)parent.rightClickLoadLoad(source,callbackdata,obj.id,2,1));
                m1 = uimenu(rightClick,'Label','Deactivate All','Callback',@(source,callbackdata)parent.rightClickLoadLoad(source,callbackdata,obj.id,3,1));
            elseif parent.getMode() == Module.MechanicalAdvantage
                m1 = uimenu(rightClick,'Label','Set as Input Load','Callback',@(source,callbackdata)parent.rightClickAdvantage(source,callbackdata,obj.id,1,1));
                m1 = uimenu(rightClick,'Label','Set as Output Load','Callback',@(source,callbackdata)parent.rightClickAdvantage(source,callbackdata,obj.id,2,1));
           elseif parent.getMode() ==  Module.PostProcessing
                    if logical(obj.follower)
                        m1 = uimenu(rightClick,'Label',['Set Force-' num2str(obj.id)  ' Magnitude  as X Axis'],'Callback',@(source,callbackdata)parent.setForceRightClick(source,callbackdata,obj.id,1,1));
                        m1 = uimenu(rightClick,'Label',['Add Force-' num2str(obj.id)  ' Magnitude  to Y Axis'],'Callback',@(source,callbackdata)parent.setForceRightClick(source,callbackdata,obj.id,1,2));
                    else
                        if logical(obj.xValue)
                            m2 = uimenu(rightClick,'Label',['Set Force-' num2str(obj.id)  ' X Magnitude  as X Axis'],'Callback',@(source,callbackdata)parent.setForceRightClick(source,callbackdata,obj.id,2,1));
                            m2 = uimenu(rightClick,'Label',['Add Force-' num2str(obj.id)  ' X Magnitude  to Y Axis'],'Callback',@(source,callbackdata)parent.setForceRightClick(source,callbackdata,obj.id,2,2));
                        end
                        if logical(obj.yValue)
                            m3 = uimenu(rightClick,'Label',['Set Force-' num2str(obj.id)  ' Y Magnitude  as X Axis'],'Callback',@(source,callbackdata)parent.setForceRightClick(source,callbackdata,obj.id,3,1));
                            m3 = uimenu(rightClick,'Label',['Add Force-' num2str(obj.id)  ' Y Magnitude  to Y Axis'],'Callback',@(source,callbackdata)parent.setForceRightClick(source,callbackdata,obj.id,3,2));
                        end
                    end
            end
            
            %color
            if ~isempty(obj.text(1)) && isgraphics(obj.text(1))
                obj.text(1).Color=color;
                obj.text(1).ButtonDownFcn=[];
                obj.text(1).UIContextMenu=rightClick;
                obj.text(1).Visible='on';
            end
            if ~isempty(obj.text(2)) && isgraphics(obj.text(2))
                obj.text(2).Color=color;
                obj.text(2).ButtonDownFcn=[];
                obj.text(2).UIContextMenu=rightClick;
                obj.text(2).Visible='on';
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
        
        function editForce(obj,src,evnt,parent,id)
            %edit force 
            handles=parent.getHandles();
            parent.setSelectedForce(id);
            %select the draggable node
            parent.plotEverything();
            %open popup
            %open the force popup
            pos=[0.8*handles.mainGUI.Position(3),0.5*handles.mainGUI.Position(4)];
            parent.openApp('forceApp',pos(1),pos(2),{});
            app=parent.getApp();
            app.updateForce(id);
            app.magnitudeTab.ForegroundColor='black';
            app.typeTab.ForegroundColor=Colors.Inactive.getColor();
            app.gainFocus();
            parent.plotEverything();
        end
        
        function obj=draw(obj,parent)
            %draw the force
            handles=parent.getHandles();
            nodes=parent.getLinkNodes(obj.link);
            node1=parent.getNode(nodes(1));
            node2=parent.getNode(nodes(2));
            %calculate node
            node=Point(node1.x+(node2.x-node1.x)/100*obj.distance,node1.y+(node2.y-node1.y)/100*obj.distance);
            if logical(obj.follower)
                %follower
                angleLink=atan2(node2.y-node1.y,node2.x-node1.x)+obj.angle;
                obj=obj.drawFollower(parent,node,angleLink,100,handles.designPlot);
            else
                %regular
                obj=obj.drawForce(parent,node,100,handles.designPlot);
            end
            if logical(obj.selected)
                color=Colors.Selected.getColor();
            else
                color=Colors.NormalLink.getColor();
            end
            rightClick= uicontextmenu(handles.mainGUI);
            if parent.getMode == Module.Overview
                uimenu(rightClick,'Label','Edit','Callback',@(source,callbackdata) obj.editForce(source,callbackdata,parent,obj.id));
                uimenu(rightClick,'Label','Delete','Callback',@(source,callbackdata) parent.deleteForce(source,callbackdata,obj.id));
            end
            %color
            if ~isempty(obj.text(1)) && isgraphics(obj.text(1))
                obj.text(1).Color=color;
                obj.text(1).ButtonDownFcn=@(src,evnt)obj.clickFunc(src,evnt,parent,obj.id);
                obj.text(1).UIContextMenu=rightClick;
            end
            if ~isempty(obj.text(2)) && isgraphics(obj.text(2))
                obj.text(2).Color=color;
                obj.text(2).ButtonDownFcn=@(src,evnt)obj.clickFunc(src,evnt,parent,obj.id);
                obj.text(2).UIContextMenu=rightClick;
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
                    parent.setSelectedForce(id);
                    parent.plotEverything();
                end
            else
                parent.deselectAllLinks();
                parent.deselectAllNodes();
                %select force
                parent.setSelectedForce(id);
                %open the spring popup
                app=parent.getApp();
                app.updateForce(id);
                app.magnitudeTab.ForegroundColor='black';
                app.typeTab.ForegroundColor=Colors.Inactive.getColor();
                app.gainFocus();
                parent.plotEverything();
            end
        end
        
        function obj=drawFollower(obj,parent,node,angleLink,power,ax)
            %function for plotting follower forces
            unit=ForceUnit.getString(parent.getWorkspace().unitForce);
            limit=parent.getWorkspace().getLimit()*0.5;
            stretch=limit/20;
            %plot the straight line
            straightLineX=[0 stretch*3];
            straightLineY=[0 0];
            angle=-pi/2;
            rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
            pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
            for i=1:3
                pointNew=rotation*[pointsX(i);pointsY(i)];
                pointsX(i)=+stretch*3+pointNew(1);pointsY(i)=+pointNew(2);
            end
            rotation=[cos(angleLink) -sin(angleLink);sin(angleLink) cos(angleLink)];
            for i=1:2
                pointNew=rotation*[straightLineX(i);straightLineY(i)];
                straightLineX(i)=node.x+pointNew(1);
                straightLineY(i)=node.y+pointNew(2);
            end
            for i=1:3
                pointNew=rotation*[pointsX(i);pointsY(i)];
                pointsX(i)=node.x+pointNew(1);pointsY(i)=node.y+pointNew(2);
            end
            if isempty(obj.line(1,1)) || ~isgraphics(obj.line(1,1))
                obj.line(1,1)=plot(ax,straightLineX,straightLineY,'linewidth',1.5);
                obj.line(1,2)=plot(ax,pointsX,pointsY,'linewidth',1.5);
                %plot the text
                obj.text(1)=text(ax,straightLineX(end)+0.75*stretch,straightLineY(end)+0.75*stretch, [num2str(abs(obj.magnitude)*power/100),' ',unit]);
            else
                obj.line(1,1).XData=straightLineX;
                obj.line(1,1).YData=straightLineY;
                obj.line(1,2).XData=pointsX;
                obj.line(1,2).YData=pointsY;
                obj.text(1).Position=[straightLineX(end)+0.75*stretch,straightLineY(end)+0.75*stretch];
                obj.text(1).String=[num2str(abs(obj.magnitude)*power/100),' ',unit];
            end
        end
        
        
        function obj=drawFollowerNoGUI(obj,workspace,node,angleLink,power)
            %function for plotting follower forces
            limit=workspace.getLimit()*0.5;
            stretch=limit/20;
            %plot the straight line
            straightLineX=[0 stretch*3];
            straightLineY=[0 0];
            angle=-pi/2;
            rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
            pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
            for i=1:3
                pointNew=rotation*[pointsX(i);pointsY(i)];
                pointsX(i)=+stretch*3+pointNew(1);pointsY(i)=+pointNew(2);
            end
            rotation=[cos(angleLink) -sin(angleLink);sin(angleLink) cos(angleLink)];
            for i=1:2
                pointNew=rotation*[straightLineX(i);straightLineY(i)];
                straightLineX(i)=node.x+pointNew(1);
                straightLineY(i)=node.y+pointNew(2);
            end
            for i=1:3
                pointNew=rotation*[pointsX(i);pointsY(i)];
                pointsX(i)=node.x+pointNew(1);pointsY(i)=node.y+pointNew(2);
            end
            if isempty(obj.line(1,1)) || ~isgraphics(obj.line(1,1))
                obj.line(1,1)=plot(handles.designPlot,straightLineX,straightLineY,'linewidth',1.5);
                obj.line(1,2)=plot(handles.designPlot,pointsX,pointsY,'linewidth',1.5);
                %plot the text
                obj.text(1)=text(straightLineX(end)+0.75*stretch,straightLineY(end)+0.75*stretch, [num2str(abs(obj.magnitude)*power/100),' ']);
            else
                obj.line(1,1).XData=straightLineX;
                obj.line(1,1).YData=straightLineY;
                obj.line(1,2).XData=pointsX;
                obj.line(1,2).YData=pointsY;
                obj.text(1).Position=[straightLineX(end)+0.75*stretch,straightLineY(end)+0.75*stretch];
                obj.text(1).String=[num2str(abs(obj.magnitude)*power/100)];
            end
        end
        
      function obj=drawForceNoGUI(obj,mainGUI,workspace,node,power)
            %function for forces
            limit=workspace.getLimit()*0.5;
            stretch=limit/20;
            %force in x direction
            
            if obj.xValue > 0
                %plot the straight line
                if isempty(obj.line(1,1)) || ~isgraphics(obj.line(1,1))
                    obj.line(1,1)=plot(mainGUI,[node.x node.x+stretch*3],[node.y node.y],'linewidth',1.5);
                else
                    obj.line(1,1).XData=[node.x node.x+stretch*3];
                    obj.line(1,1).YData=[node.y node.y];
                end
                
                angle=-pi/2;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x+stretch*3+pointNew(1);pointsY(i)=node.y+pointNew(2);
                end
                %plot the hat and the text
                if isempty(obj.line(1,2)) || ~isgraphics(obj.line(1,2))
                    obj.line(1,2)=plot(mainGUI,pointsX,pointsY,'linewidth',1.5);
                    %plot the text
                    obj.text(1)=text(mainGUI,node.x+stretch,node.y-stretch, [num2str(abs(obj.xValue)*power/100)]);
                else
                    obj.line(1,2).XData=pointsX;
                    obj.line(1,2).YData=pointsY;
                    obj.text(1).Position=[node.x+stretch,node.y-stretch];
                    obj.text(1).String=[num2str(abs(obj.xValue)*power/100)];
                end
                %negative x force
            elseif obj.xValue < 0
                %plot the straight line
                if isempty(obj.line(1,1)) || ~isgraphics(obj.line(1,1))
                    obj.line(1,1)=plot(mainGUI,[node.x node.x-stretch*3],[node.y node.y],'linewidth',1.5);
                else
                    obj.line(1,1).XData=[node.x node.x-stretch*3];
                    obj.line(1,1).YData=[node.y node.y];
                end
                
                angle=pi/2;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x-stretch*3+pointNew(1);
                    pointsY(i)=node.y+pointNew(2);
                end
                %plot the hat and the text
                if isempty(obj.line(1,2)) || ~isgraphics(obj.line(1,2))
                    obj.line(1,2)=plot(mainGUI,pointsX,pointsY,'linewidth',1.5);
                    %plot the text
                    obj.text(1)=text(mainGUI,node.x-2*stretch,node.y-stretch, [num2str(abs(obj.xValue)*power/100)]);
                else
                    obj.line(1,2).XData=pointsX;
                    obj.line(1,2).YData=pointsY;
                    obj.text(1).Position=[node.x-2*stretch,node.y-stretch];
                    obj.text(1).String=[num2str(abs(obj.xValue)*power/100)];
                end
            end
            %positive y force
            if obj.yValue > 0
                %plot the straight line
                if isempty(obj.line(2,1)) || ~isgraphics(obj.line(2,1))
                    obj.line(2,1)=plot(mainGUI,[node.x node.x],[node.y node.y+stretch*3],'linewidth',1.5);
                else
                    obj.line(2,1).XData=[node.x node.x];
                    obj.line(2,1).YData=[node.y node.y+stretch*3];
                end
                
                angle=0;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x+pointNew(1);pointsY(i)=node.y+pointNew(2)+stretch*3;
                end
                %plot the hat and the text
                if isempty(obj.line(2,2)) || ~isgraphics(obj.line(2,2))
                    obj.line(2,2)=plot(mainGUI,pointsX,pointsY,'linewidth',1.5);
                    %plot the text
                    obj.text(2)=text(mainGUI,node.x,node.y+stretch*3.75, [num2str(abs(obj.yValue)*power/100)],'HorizontalAlignment','center');
                else
                    obj.line(2,2).XData=pointsX;
                    obj.line(2,2).YData=pointsY;
                    obj.text(2).Position=[node.x,node.y+stretch*3.75];
                    obj.text(2).String=[num2str(abs(obj.yValue)*power/100)];
                end
                %negative y force
            elseif obj.yValue < 0
                %plot the straight line
                if isempty(obj.line(2,1)) || ~isgraphics(obj.line(2,1))
                    obj.line(2,1)=plot(mainGUI,[node.x node.x],[node.y node.y+-stretch*3],'linewidth',1.5);
                else
                    obj.line(2,1).XData=[node.x node.x];
                    obj.line(2,1).YData=[node.y node.y-stretch*3];
                end
                angle=pi;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x+pointNew(1);pointsY(i)=node.y-stretch*3+pointNew(2);
                end
                %plot the hat and the text
                if isempty(obj.line(2,2)) || ~isgraphics(obj.line(2,2))
                    obj.line(2,2)=plot(mainGUI,pointsX,pointsY,'linewidth',1.5);
                    %plot the text
                    obj.text(2)=text(mainGUI,node.x,node.y-stretch*3.75, [num2str(abs(obj.yValue)*power/100)],'HorizontalAlignment','center');
                else
                    obj.line(2,2).XData=pointsX;
                    obj.line(2,2).YData=pointsY;
                    obj.text(2).Position=[node.x,node.y-stretch*3.75];
                    obj.text(2).String=[num2str(abs(obj.yValue)*power/100)];
                end
                
            end
        end
         
        
        function obj=drawForce(obj,parent,node,power,ax)
            %function for forces
            unit=ForceUnit.getString(parent.getWorkspace().unitForce);
            limit=parent.getWorkspace().getLimit()*0.5;
            stretch=limit/20;
            %force in x direction
            if obj.xValue > 0
                %plot the straight line
                if isempty(obj.line(1,1)) || ~isgraphics(obj.line(1,1))
                    obj.line(1,1)=plot(ax,[node.x node.x+stretch*3],[node.y node.y],'linewidth',1.5);
                else
                    obj.line(1,1).XData=[node.x node.x+stretch*3];
                    obj.line(1,1).YData=[node.y node.y];
                end
                
                angle=-pi/2;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x+stretch*3+pointNew(1);pointsY(i)=node.y+pointNew(2);
                end
                %plot the hat and the text
                if isempty(obj.line(1,2)) || ~isgraphics(obj.line(1,2))
                    obj.line(1,2)=plot(ax,pointsX,pointsY,'linewidth',1.5);
                    %plot the text
                    obj.text(1)=text(ax,node.x+stretch,node.y-stretch, [num2str(abs(obj.xValue)*power/100),' ',unit]);
                else
                    obj.line(1,2).XData=pointsX;
                    obj.line(1,2).YData=pointsY;
                    obj.text(1).Position=[node.x+stretch,node.y-stretch];
                    obj.text(1).String=[num2str(abs(obj.xValue)*power/100),' ',unit];
                end
                %negative x force
            elseif obj.xValue < 0
                %plot the straight line
                if isempty(obj.line(1,1)) || ~isgraphics(obj.line(1,1))
                    obj.line(1,1)=plot(ax,[node.x node.x-stretch*3],[node.y node.y],'linewidth',1.5);
                else
                    obj.line(1,1).XData=[node.x node.x-stretch*3];
                    obj.line(1,1).YData=[node.y node.y];
                end
                
                angle=pi/2;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x-stretch*3+pointNew(1);
                    pointsY(i)=node.y+pointNew(2);
                end
                %plot the hat and the text
                if isempty(obj.line(1,2)) || ~isgraphics(obj.line(1,2))
                    obj.line(1,2)=plot(ax,pointsX,pointsY,'linewidth',1.5);
                    %plot the text
                    obj.text(1)=text(ax,node.x-2*stretch,node.y-stretch, [num2str(abs(obj.xValue)*power/100),' ',unit]);
                else
                    obj.line(1,2).XData=pointsX;
                    obj.line(1,2).YData=pointsY;
                    obj.text(1).Position=[node.x-2*stretch,node.y-stretch];
                    obj.text(1).String=[num2str(abs(obj.xValue)*power/100),' ',unit];
                end
            end
            %positive y force
            if obj.yValue > 0
                %plot the straight line
                if isempty(obj.line(2,1)) || ~isgraphics(obj.line(2,1))
                    obj.line(2,1)=plot(ax,[node.x node.x],[node.y node.y+stretch*3],'linewidth',1.5);
                else
                    obj.line(2,1).XData=[node.x node.x];
                    obj.line(2,1).YData=[node.y node.y+stretch*3];
                end
                
                angle=0;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x+pointNew(1);pointsY(i)=node.y+pointNew(2)+stretch*3;
                end
                %plot the hat and the text
                if isempty(obj.line(2,2)) || ~isgraphics(obj.line(2,2))
                    obj.line(2,2)=plot(ax,pointsX,pointsY,'linewidth',1.5);
                    %plot the text
                    obj.text(2)=text(ax,node.x,node.y+stretch*3.75, [num2str(abs(obj.yValue)*power/100),' ',unit],'HorizontalAlignment','center');
                else
                    obj.line(2,2).XData=pointsX;
                    obj.line(2,2).YData=pointsY;
                    obj.text(2).Position=[node.x,node.y+stretch*3.75];
                    obj.text(2).String=[num2str(abs(obj.yValue)*power/100),' ',unit];
                end
                %negative y force
            elseif obj.yValue < 0
                %plot the straight line
                if isempty(obj.line(2,1)) || ~isgraphics(obj.line(2,1))
                    obj.line(2,1)=plot(ax,[node.x node.x],[node.y node.y+-stretch*3],'linewidth',1.5);
                else
                    obj.line(2,1).XData=[node.x node.x];
                    obj.line(2,1).YData=[node.y node.y-stretch*3];
                end
                angle=pi;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x+pointNew(1);pointsY(i)=node.y-stretch*3+pointNew(2);
                end
                %plot the hat and the text
                if isempty(obj.line(2,2)) || ~isgraphics(obj.line(2,2))
                    obj.line(2,2)=plot(ax,pointsX,pointsY,'linewidth',1.5);
                    %plot the text
                    obj.text(2)=text(ax,node.x,node.y-stretch*3.75, [num2str(abs(obj.yValue)*power/100),' ',unit],'HorizontalAlignment','center');
                else
                    obj.line(2,2).XData=pointsX;
                    obj.line(2,2).YData=pointsY;
                    obj.text(2).Position=[node.x,node.y-stretch*3.75];
                    obj.text(2).String=[num2str(abs(obj.yValue)*power/100),' ',unit];
                end
                
            end
        end
    end
end
