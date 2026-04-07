classdef Helper
    %helper methods
    properties
    end
    
    methods(Static)
        function [valid,number]=getNumber(string)
            number=str2double(string);
            if isnan(number)
                valid=0;
            else
                valid=1;
            end
        end
        
        function [xMin,xMax,yMin,yMax]=pixel2Coord(rect,axis)
            %converts from pixel rectangle to coordinate
            xLim=xlim(axis);
            yLim=ylim(axis);
            xMin=xLim(1)+(xLim(2)-xLim(1))*(rect(1)-axis.Position(1))/(axis.Position(3)-axis.Position(1)); 
            yMin=yLim(1)+(yLim(2)-yLim(1))*(rect(2)-axis.Position(2))/(axis.Position(4)-axis.Position(2)); 
            xMax=xLim(1)+(xLim(2)-xLim(1))*(rect(1)+rect(3)-axis.Position(1))/(axis.Position(3)-axis.Position(1)); 
            yMax=yLim(1)+(yLim(2)-yLim(1))*(rect(2)+rect(4)-axis.Position(2))/(axis.Position(4)-axis.Position(2)); 
        end
        
        function [x,y]=axis2Normalized(axis,currentPoint)
            %converts to a point on a axis to normalized
            xLim=xlim(axis);
            yLim=ylim(axis);
            x=axis.Position(1)+axis.Position(3)*(currentPoint(1)-xLim(1))/(xLim(2)-xLim(1)); 
            y=axis.Position(2)+axis.Position(4)*(currentPoint(2)-yLim(1))/(yLim(2)-yLim(1)); 
        end
        
        function angle=angleBetween2Vectors(vec1,vec2,mode,deg)
            %angle between two vectors
            A.x=vec1(1);A.y=vec1(2);
            B.x=vec2(1);B.y=vec2(2);
            if mode==1
                %format
                angle=atan2(B.y,B.x)-atan2(A.y,A.x);
                if logical(deg)
                    angle=angle*180/pi;
                end
            else
                %do not format
                angle=atan2(B.y,B.x)-atan2(A.y,A.x);
                if logical(deg)
                    angle=angle*180/pi;
                end
            end
        end
        
        function check=checkLegitLink(joints)
            joint=zeros(1,2);
            switch joints(1,1)
                case Joint.GroundWelded
                    joint(1,1)=-1;
                case Joint.GroundPin
                    joint(1,1)=-2;
                case Joint.GroundSlider
                    joint(1,1)=-3;
                case Joint.NA
                    joint(1,1)=4;
                case Joint.Welded
                    joint(1,1)=1;
                case Joint.Pin
                    joint(1,1)=2;
                case Joint.Slider
                    joint(1,1)=3;
            end
            switch joints(1,2)
                case Joint.GroundWelded
                    joint(1,2)=-1;
                case Joint.GroundPin
                    joint(1,2)=-2;
                case Joint.GroundSlider
                    joint(1,2)=-3;
                case Joint.NA
                    joint(1,2)=4;
                case Joint.Welded
                    joint(1,2)=1;
                case Joint.Pin
                    joint(1,2)=2;
                case Joint.Slider
                    joint(1,2)=3;
            end
            
            legitConnections=[-2 1;-2 2;-2 -3;2 1; 2 2;1 1;-1 2; -1 1;2 -3;1 -3;1 3;-2 3;2 3;-1 3];
            check=ismember(joint,legitConnections,'rows') || ismember([joint(1,2) joint(1,1)],legitConnections,'rows');
        end
        
        function check=checkLegitLinearSpring(joints)
            %check if the link can be converted to linear spring
            joint=zeros(1,2);
            switch joints(1,1)
                case Joint.GroundWelded
                    joint(1,1)=-1;
                case Joint.GroundPin
                    joint(1,1)=-2;
                case Joint.GroundSlider
                    joint(1,1)=-3;
                case Joint.NA
                    joint(1,1)=4;
                case Joint.Welded
                    joint(1,1)=1;
                case Joint.Pin
                    joint(1,1)=2;
                case Joint.Slider
                    joint(1,1)=3;
            end
            switch joints(1,2)
                case Joint.GroundWelded
                    joint(1,2)=-1;
                case Joint.GroundPin
                    joint(1,2)=-2;
                case Joint.GroundSlider
                    joint(1,2)=-3;
                case Joint.NA
                    joint(1,2)=4;
                case Joint.Welded
                    joint(1,2)=1;
                case Joint.Pin
                    joint(1,2)=2;
                case Joint.Slider
                    joint(1,2)=3;
            end
            
            linearSprings=[-2 1;-2 2;2 1;2 2;1 1;-1 2;-1 1;2 3;1 3;-2 3;2 3;];
            check=ismember(joint,linearSprings,'rows') || ismember([joint(1,2) joint(1,1)],linearSprings,'rows');
        end
        
        function linkID=findLink(currentLink,links,rigidLinks,type)
            %find corresponding link           
            switch type
                case 1
                    %normal link
                    linkNodes=links(currentLink).getNodes();
                    for i=1:length(rigidLinks)
                        if length(union(linkNodes,rigidLinks{i}.nodes)) == 2
                            linkID=i;
                            return;
                        end
                    end
                case 2
                    %slider link
                    linkJoints=links(currentLink).getJoints();
                    if linkJoints(1) == Joint.GroundSlider
                        node=links(currentLink).getNode(1);
                    else
                        node=links(currentLink).getNode(2);
                    end
                    for i=1:length(rigidLinks)
                        if (rigidLinks{i}.nodes(1) == node && rigidLinks{i}.joints(1) == Joint.GroundSlider) ...
                                || (rigidLinks{i}.nodes(2) == node && rigidLinks{i}.joints(2) == Joint.GroundSlider)
                            linkID=i;
                            return;
                        end
                    end
                case 3
                    %driver
                    linkJoints=links(currentLink).getJoints();
                    if linkJoints(1) == Joint.GroundPin
                        node=links(currentLink).getNode(1);
                    else
                        node=links(currentLink).getNode(2);
                    end
                    for i=1:length(rigidLinks)
                        if rigidLinks{i}.nodes(1) == node && rigidLinks{i}.joints(1) == Joint.GroundPin
                            linkID=i;
                            return;
                        end
                    end
                    
            end
        end
        
    end
    
end

