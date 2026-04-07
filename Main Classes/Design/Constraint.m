classdef Constraint
    %CONSTRAINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id;
        type;
        distance=[];
        value;
        direction;
        link;
        line=gobjects(1,3);
        text=[];
    end
    
    methods
        
        function obj=Constraint(id,parent,link,value,type,direction)
            %constructor
            obj.id=id;
            obj.value=value;
            obj.type=type;
            obj.link=link;
            obj.direction=direction;
            %plot
            %obj=obj.draw(parent);
        end
        
        function type=getType(obj)
            %get the constraint type
            type=obj.type;
        end
        
        function value=getValue(obj)
            %get the value
            value=obj.value;
        end
        
        function link=getLink(obj,pos)
            %get the link
            link=obj.link(1,pos);
        end
        
        function id=getID(obj)
            %get the id
            id=obj.id;
        end
        
        function obj=setID(obj,id)
            %set the id
            obj.id=id;
        end
        
        
        function obj=draw(obj,parent)
            %draw main function
            handles=parent.getHandles();
            limit=parent.getWorkspace().getLimit();
            point1=parent.getLink(obj.link(1,1)).getNode(1);
            point2=parent.getLink(obj.link(1,1)).getNode(2);
            switch(obj.type)
                case ConstraintType.Length
                    obj=obj.drawLengthType(handles.designPlot,point1,point2,limit,parent);
                case ConstraintType.AngleX
                    obj=obj.drawAngleXY(handles.designPlot,point1,limit,parent,1);
                case  ConstraintType.AngleY
                    obj=obj.drawAngleXY(handles.designPlot,point1,limit,parent,2);
                case  ConstraintType.AngleLink
                    point11=parent.getLink(obj.link(1,2)).getNode(1);
                    point22=parent.getLink(obj.link(1,2)).getNode(2);
                    obj=obj.drawAngleBetweenConstraint(point1,point2,point11,point22,handles.designPlot,limit,parent);
            end
        end
        
        function obj=deleteConstraintDrawing(obj)
            %delete drawing
            delete(obj.line);
            delete(obj.text);
        end
        
        function obj=drawLengthType(obj,axis,point1,point2,limit,parent)
            %draw the constraint for the length type
            angle=atan2(point2.y-point1.y,point2.x-point1.x);
            %the two perpendicular lines to the link
            dst=limit/20;
            xData1=[node1.x+dst*cos(angle+pi/2) node1.x+dst*cos(angle-pi/2)];
            yData1=[node1.y+dst*sin(angle+pi/2) node1.y+dst*sin(angle-pi/2)];
            xData2=[node2.x+dst*cos(angle+pi/2) node2.x+dst*cos(angle-pi/2)];
            yData2=[node2.y+dst*sin(angle+pi/2) node2.y+dst*sin(angle-pi/2)];
            
            if ~isgraphics(obj.line(1,1))
                obj.line(1,1)=plot(axis,xData1,yData1,'Color',Colors.Constrained.getColor(),'LineWidth',1);
                obj.line(1,2)=plot(axis,xData2,yData2,'Color',Colors.Constrained.getColor(),'LineWidth',1);
            else
                obj.line(1,1).XData=xData1;
                obj.line(1,1).YData=yData1;
                obj.line(1,2).XData=xData2;
                obj.line(1,2).YData=yData2;
            end
            %the paralel line to the link
            if isempty(obj.distance)
                dst=limit/15;
                obj.distance=limit/15;
            else
                dst=obj.distance;
            end
            if dst >0
                %position at right
                xData1=[point1.x+dst*cos(angle-pi/2) point2.x+dst*cos(angle-pi/2)];
                yData1=[point1.y+dst*sin(angle-pi/2) point2.y+dst*sin(angle-pi/2)];
                if ~isgraphics(obj.line(1,3))
                    obj.line(1,3)=plot(axis,xData1,yData1,'Color',Colors.Constrained.getColor(),'LineWidth',1);
                else
                    obj.line(1,3).XData=xData1;
                    obj.line(1,3).YData=yData1;
                end
                middleX=sum(xData1)/2+limit/20;
                middleY=sum(yData1)/2+limit/20;
            else
                %position at left
                xData1=[point1.x+abs(dst)*cos(angle+pi/2) point2.x+abs(dst)*cos(angle+pi/2)];
                yData1=[point1.y+abs(dst)*sin(angle+pi/2) point2.y+abs(dst)*sin(angle+pi/2)];
                if ~isgraphics(obj.line(1,3))
                    obj.line(1,3)=plot(axis,xData1,yData1,'Color',Colors.Constrained.getColor(),'LineWidth',1);
                else
                    obj.line(1,3).XData=xData1;
                    obj.line(1,3).YData=yData1;
                end
                middleX=sum(xData1)/2+limit/20;
                middleY=sum(yData1)/2+limit/20;
            end
            %position the text
            if angle >pi/2
                angle =-(pi-angle);
            end
            if 	isempty(obj.text)
                obj.text=text(middleX,middleY,num2str(obj.value),'Color',Colors.Constrained.getColor());
                obj.text.Rotation=angle*180/pi;
            else
                obj.text.Position=[middleX middleY];
                obj.text.String=num2str(obj.value);
                obj.text.rotation=angle*180/pi;
            end
            %button click function
            obj=obj.clickEvent(parent);
        end
        
        function obj=drawAngleXY(obj,axis,point1,limit,parent,type)
            %draw x(1) or y(2) type constraint
            %the paralel line to the link
            if isempty(obj.distance)
                dst=limit/10;
                obj.distance=dst;
            else
                dst=obj.distance;
            end
            if type == 1
                coordinates=findArc(dst,point1,0,obj.value);
            elseif type == 2
                coordinates=findArc(dst,point1,90,obj.value+90);
            end
            if ~isgraphics(obj.line(1,1))
                obj.line(1,1)=plot(axis,coordinates.x,coordinates.y,'Color',Colors.Constrained.getColor(),'LineWidth',1);
            else
                obj.line(1,1).XData=coordinates.x;
                obj.line(1,1).YData=coordinates.y;
            end
            %plot the dashed line along the x or y axis
            if type == 1
                if ~isgraphics(obj.line(1,2))
                    obj.line(1,2)=plot(axis,[point1.x point1.x+dst*2],[point1.y point1.y],'Color',Colors.Constrained.getColor(),'LineWidth',1,'LineStyle','--');
                else
                    obj.line(1,2).XData=[point1.x point1.x+dst*2];
                    obj.line(1,2).YData=[point1.y point1.y];
                end
            elseif type == 2
                if ~isgraphics(obj.line(1,2))
                    obj.line(1,2)=plot(axis,[point1.x point1.x],[point1.y point1.y+dst*2],'Color',Colors.Constrained.getColor(),'LineWidth',1,'LineStyle','--');
                else
                    obj.line(1,2).XData=[point1.x point1.x];
                    obj.line(1,2).YData=[point1.y point1.y+dst*2];
                end
            end
            
            %plot the text
            if type == 1
                if isempty(obj.text)
                    handles.constraints(constraintID).text=text(point1.x+dst*1.1*cosd(obj.value/2),point1.y+dst*1.1*sind(obj.value/2),num2str(obj.value),'Color',Colors.Constrained.getColor());
                    obj.text.Rotation=obj.value/2;
                else
                    obj.text.Position=[point1.x+dst*1.1*cosd(obj.value/2),point1.y+dst*1.1*sind(obj.value/2)];
                    obj.text.String=num2str(obj.value);
                    obj.text.rotation=obj.value;
                end
            elseif type == 2
                if isempty(obj.text)
                    obj.text=text(point1.x+dst*1.1*cosd(obj.value/2+90),point1.y+dst*1.1*sind(obj.value/2+90),num2str(obj.value),'Color',Colors.Constrained.getColor());
                    obj.text.Rotation=obj.value;
                else
                    obj.text.Position=[point1.x+dst*1.1*cosd(obj.value/2+90),point1.y+dst*1.1*sind(obj.value/2+90)];
                    obj.text.String=num2str(obj.value);
                    obj.text.rotation=obj.value;
                end
            end
            %button click function
            obj=obj.clickEvent(parent);
        end
        
        function clickConstraint(obj,src,evnt,parent,id)
            %click event
            parent.setSelectedConstraint(id);
        end
        
        function obj=clickEvent(obj,parent)
            %constraint click event
            switch(obj.type)
                case ConstraintType.Length
                    fnc=@(src,evnt)obj.clickConstraint(src,evnt,parent,obj.id);
                    obj.line(1,3).ButtonDownFcn=fnc;
                case ConstraintType.AngleX
                    fnc=@(src,evnt)obj.clickConstraint(src,evnt,parent,obj.id);
                    obj.line(1,1).ButtonDownFcn=fnc;
                case  ConstraintType.AngleY
                    fnc=@(src,evnt)obj.clickConstraint(src,evnt,parent,obj.id);
                    obj.line(1,1).ButtonDownFcn=fnc;
                case  ConstraintType.AngleLink
                    fnc=@(src,evnt)obj.clickConstraint(src,evnt,parent,obj.id);
            end
            for i=1:length(obj.line)
                if isgraphics(obj.line(1,i))
                    obj.line(1,i).ButtonDownFcn=fnc;
                end
            end
            obj.text.ButtonDownFcn=fnc;
            
        end
        
        
        function obj=drawAngleBetweenConstraint(obj,point1_1,point2_1,point1_2,point2_2,axis,limit,parent)
            %for drawing  angle constraints between two vectors
            
            %get the angle 1
            if bj.direction(1,1) == 1
                %from 1 to 2
                angle1=atan2(point2_1.y-point1_1.y,point2_1.x-point1_1.x )*180/pi;
                startingNode=point1_1;
            else
                angle1=atan2(point1_1.y-point2_1.y,point1_1.x-point2_1.x )*180/pi;
                startingNode=point2_1;
            end
            %get the angle 2
            if obj.direction(1,2) == 1
                %from 1 to 2
                angle2=atan2(point2_2.y-point1_2.y,point2_2.x-point1_2.x )*180/pi;
            else
                angle2=atan2(point1_2.y-point2_2.y,point1_2.x-point2_2.x )*180/pi;
            end
            
            %plot the arc
            if isempty(obj.distance)
                dst=limit/10;
                obj.distance=dst;
            else
                dst=obj.distance;
            end
            coordinates=findArc(dst,startingNode,angle1,angle2);
            
            if obj.line(1,1) == 0
                obj.line(1,1)=plot(axis,coordinates.x,coordinates.y,'Color',Colors.Constrained.getColor(),'LineWidth',1);
            else
                obj.line(1,1).XData=coordinates.x;
                obj.line(1,1).YData=coordinates.y;
            end
            
            if isempty(obj.text)
                obj.text=text(startingNode.x+dst*1.1*cosd(0.5*(angle1+angle2)),startingNode.y+dst*1.1*sind(0.5*(angle1+angle2)),num2str(obj.value),'Color',Colors.Constrained.getColor());
                if 0.5*(angle1+angle2) > 180
                    obj.text.Rotation=0.5*(angle1+angle2)-180;
                else
                    obj.text.Rotation=0.5*(angle1+angle2);
                end
            else
                obj.text.Position=[startingNode.x+dst*1.1*cosd(0.5*(angle1+angle2)),startingNode.y+dst*1.1*sind(0.5*(angle1+angle2))];
                obj.text.String=num2str(obj.value);
                if 0.5*(angle1+angle2) > 180
                    obj.text.Rotation=0.5*(angle1+angle2)-180;
                else
                    obj.text.Rotation=0.5*(angle1+angle2);
                end
            end
            %button click function
            obj=obj.clickEvent(parent);
        end
        function coordinates=findArc(obj,radius,center,angle1,angle2)
            %function for drawing a arc at a center between two angles
            if angle1> angle2
                angles= linspace( angle2,angle1, 100);
            else
                angles= linspace( angle1,angle2, 100);
            end
            coordinates.x = radius*cosd(angles) + center.x;
            coordinates.y = radius*sind(angles) + center.y;
            
        end
        
    end
end

