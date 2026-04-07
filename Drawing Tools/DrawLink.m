classdef DrawLink
    %drawing links
    properties
    end
    
    methods(Static)
        function [x,y]=drawLine(nodes,joints,limit,isSpring)
            %draw the line
            if ~((joints(1,1)== Joint.GroundSlider && joints(1,2)== Joint.GroundWelded) || (joints(1,2)== Joint.GroundSlider && joints(1,1)== Joint.GroundWelded))
                if ~logical(isSpring)
                    x(1,1)=nodes(1,1).x;x(1,2)=nodes(1,2).x;
                    y(1,1)=nodes(1,1).y;y(1,2)=nodes(1,2).y;
                else
                    ne = 10; a = sqrt((nodes(1,1).y-nodes(1,2).y)^2+(nodes(1,1).x-nodes(1,2).x)^2); ro = limit/100;
                    [x(1,:),y(1,:)] = linearSpring(nodes(1,1).x,nodes(1,1).y,nodes(1,2).x,nodes(1,2).y,ne,a,ro);
                end
            else
                x=[];
                y=[];
            end
        end
        
        function [xJoint1,yJoint1,xJoint2,yJoint2]=drawJoints(nodes,joints,limit,angleSlider)
            %draw the joints
            %drawing parameters
            xJoint1=[];
            yJoint1=[];
            xJoint2=[];
            yJoint2=[];
            if isempty(angleSlider)
                angle=atan2((nodes(1,2).y-nodes(1,1).y),(nodes(1,2).x-nodes(1,1).x));
                angle=angle*180/pi;
            else
                angle=angleSlider;
            end
            jointRadius=limit/50;
            sliderWidth=limit/7;
            sliderHeight=limit/14;
            numOfCirclePoints=50;
            rotationMatrix=[cosd(angle) -sind(angle); sind(angle) cosd(angle)];
            %joint 1
            switch joints(1,1)
                case {Joint.GroundSlider,Joint.Slider}
                    xJoint1(1,1)=-sliderWidth/2;yJoint1(1,1)=-sliderHeight/2;
                    xJoint1(2,1)=-sliderWidth/2;yJoint1(2,1)=sliderHeight/2;
                    xJoint1(3,1)=sliderWidth/2;yJoint1(3,1)=sliderHeight/2;
                    xJoint1(4,1)=sliderWidth/2;yJoint1(4,1)=-sliderHeight/2;
                    
                    %rotate the element
                    for i=1:length(xJoint1)
                        pos=rotationMatrix*[xJoint1(i);yJoint1(i)];
                        xJoint1(i)=pos(1)+nodes(1,1).x;
                        yJoint1(i)=pos(2)+nodes(1,1).y;
                    end
                case {Joint.Welded,Joint.GroundWelded}
                    if joints(1,2) ~= Joint.GroundSlider
                        %a rectangle
                        xJoint1(1,1)=nodes(1,1).x-limit/50;yJoint1(1,1)=nodes(1,1).y-limit/50;
                        xJoint1(2,1)=xJoint1(1,1);yJoint1(2,1)=yJoint1(1,1)+limit/25;
                        xJoint1(3,1)=xJoint1(1,1)+limit/25;yJoint1(3,1)=yJoint1(2,1);
                        xJoint1(4,1)=xJoint1(3,1);yJoint1(4,1)=yJoint1(1,1);
                    end
                case {Joint.GroundPin,Joint.Pin}
                    angles = linspace(0,2*pi, numOfCirclePoints);
                    xJoint1 = jointRadius*cos(angles);
                    yJoint1 = jointRadius*sin(angles);
                    xJoint1=xJoint1'+nodes(1,1).x;
                    yJoint1=yJoint1'+nodes(1,1).y;
            end
            %joint 2
            switch joints(1,2)
                case {Joint.GroundSlider,Joint.Slider}
                    xJoint2(1,1)=-sliderWidth/2;yJoint2(1,1)=-sliderHeight/2;
                    xJoint2(2,1)=-sliderWidth/2;yJoint2(2,1)=sliderHeight/2;
                    xJoint2(3,1)=sliderWidth/2;yJoint2(3,1)=sliderHeight/2;
                    xJoint2(4,1)=sliderWidth/2;yJoint2(4,1)=-sliderHeight/2;
                    %rotate the element
                    for i=1:length(xJoint2)
                        pos=rotationMatrix*[xJoint2(i);yJoint2(i)];
                        xJoint2(i)=pos(1)+nodes(1,2).x;
                        yJoint2(i)=pos(2)+nodes(1,2).y;
                        
                    end
                case {Joint.Welded,Joint.GroundWelded}
                    %a rectangle
                    xJoint2(1,1)=nodes(1,2).x-limit/50;yJoint2(1,1)=nodes(1,2).y-limit/50;
                    xJoint2(2,1)=xJoint2(1,1);yJoint2(2,1)=yJoint2(1,1)+limit/25;
                    xJoint2(3,1)=xJoint2(1,1)+limit/25;yJoint2(3,1)=yJoint2(2,1);
                    xJoint2(4,1)=xJoint2(3,1);yJoint2(4,1)=yJoint2(1,1);
                    
                case {Joint.GroundPin,Joint.Pin}
                    angles = linspace(0,2*pi, numOfCirclePoints);
                    xJoint2 = jointRadius*cos(angles);
                    yJoint2 = jointRadius*sin(angles);
                    xJoint2=xJoint2+nodes(1,2).x;
                    yJoint2=yJoint2+nodes(1,2).y;
            end
        end
        
        function [xGround1,yGround1,xGround2,yGround2]=drawGrounds(nodes,joints,limit,angleSlider)
            %draw grounds
            xGround1=[];
            yGround1=[];
            xGround2=[];
            yGround2=[];
            if isempty(angleSlider)
                angle=atan2((nodes(1,2).y-nodes(1,1).y),(nodes(1,2).x-nodes(1,1).x));
                angle=angle*180/pi;
            else
                angle=angleSlider;
            end
            rotationMatrix=[cosd(angle) -sind(angle); sind(angle) cosd(angle)];
            %draw the ground
            xList=[-0.5,0.5,-0.5,-0.35,-0.25,-0.1,0,0.15,0.25,0.4,0.5,0.65]*limit/5;
            yList=[0,0,0,-0.25,0,-0.25,0,-0.25,0,-0.25,0,-0.25]*limit/5;
            
            switch joints(1,1)
                case Joint.GroundSlider
                    index=1;
                    xGround1=zeros(1,6);
                    yGround1=zeros(1,6);
                    offset=rotationMatrix*[0;-limit/28];
                    for i=1:2:11
                        point1=rotationMatrix*[xList(i);yList(i)]+[offset(1);offset(2)];
                        point2=rotationMatrix*[xList(i+1);yList(i+1)]+[offset(1);offset(2)];
                        xGround1(index,1)=point1(1)+nodes(1,1).x;xGround1(index,2)=point2(1)+nodes(1,1).x;
                        yGround1(index,1)=point1(2)+nodes(1,1).y;yGround1(index,2)=point2(2)+nodes(1,1).y;
                        index=index+1;
                    end
                    
                case Joint.GroundPin
                    if nodes(1,2).y>nodes(1,1).y
                        %triangle
                        xGround1(1,1)=nodes(1,1).x;yGround1(1,1)=nodes(1,1).y;
                        xGround1(1,2)=nodes(1,1).x-limit/25;yGround1(1,2)=nodes(1,1).y-limit/15;
                        xGround1(1,3)=xGround1(1,2)+2*limit/25;yGround1(1,3)=yGround1(1,2);
                        %ground
                        index=2;
                        for i=1:2:11
                            xGround1(index,1)=xList(i)+nodes(1,1).x;xGround1(index,2)=xList(i+1)+nodes(1,1).x;
                            yGround1(index,1)=yList(i)+nodes(1,1).y-limit/15;yGround1(index,2)=yList(i+1)+nodes(1,1).y-limit/15;
                            index=index+1;
                        end
                    else
                        %triangle
                        xGround1(1,1)=nodes(1,1).x;yGround1(1,1)=nodes(1,1).y;
                        xGround1(1,2)=nodes(1,1).x-limit/25;yGround1(1,2)=nodes(1,1).y+limit/15;
                        xGround1(1,3)=xGround1(1,2)+2*limit/25;yGround1(1,3)=yGround1(1,2);
                        angle=pi;
                        rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                        index=2;
                        for i=1:2:11
                            point1=rotation*[xList(i);yList(i)];
                            point2=rotation*[xList(i+1);yList(i+1)];
                            xGround1(index,1)=point1(1)+nodes(1,1).x;xGround1(index,2)=point2(1)+nodes(1,1).x;
                            yGround1(index,1)=point1(2)+nodes(1,1).y+limit/15;yGround1(index,2)=point2(2)+nodes(1,1).y+limit/15;
                            index=index+1;
                        end
                    end
                case Joint.GroundWelded
                    if joints(1,2) ~= Joint.GroundSlider
                        if nodes(1,2).y>nodes(1,1).y
                            index=1;
                            xGround1=zeros(1,6);
                            yGround1=zeros(1,6);
                            for i=1:2:11
                                xGround1(index,1)=xList(i)+nodes(1,1).x;xGround1(index,2)=xList(i+1)+nodes(1,1).x;
                                yGround1(index,1)=yList(i)+nodes(1,1).y-limit/50;yGround1(index,2)=yList(i+1)+nodes(1,1).y-limit/50;
                                index=index+1;
                            end
                        else
                            angle=pi;
                            rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                            index=1;
                            xGround1=zeros(1,6);
                            yGround1=zeros(1,6);
                            for i=1:2:11
                                point1=rotation*[xList(i);yList(i)];
                                point2=rotation*[xList(i+1);yList(i+1)];
                                xGround1(index,1)=point1(1)+nodes(1,1).x;xGround1(index,2)=point2(1)+nodes(1,1).x;
                                yGround1(index,1)=point1(2)+nodes(1,1).y+limit/50;yGround1(index,2)=point2(2)+nodes(1,1).y+limit/50;
                                index=index+1;
                            end
                        end
                    end
            end
            
            switch joints(1,2)
                case Joint.GroundSlider
                    index=1;
                    xGround2=zeros(1,6);
                    yGround2=zeros(1,6);
                    offset=rotationMatrix*[0;-limit/28];
                    for i=1:2:11
                        point1=rotationMatrix*[xList(i);yList(i)]+[offset(1);offset(2)];
                        point2=rotationMatrix*[xList(i+1);yList(i+1)]+[offset(1);offset(2)];
                        xGround2(index,1)=point1(1)+nodes(1,2).x;xGround2(index,2)=point2(1)+nodes(1,2).x;
                        yGround2(index,1)=point1(2)+nodes(1,2).y;yGround2(index,2)=point2(2)+nodes(1,2).y;
                        index=index+1;
                    end
                case Joint.GroundPin
                    if nodes(1,1).y>nodes(1,2).y
                        %triangle
                        xGround2(1,1)=nodes(1,2).x;yGround2(1,1)=nodes(1,2).y;
                        xGround2(1,2)=nodes(1,2).x-limit/25;yGround2(1,2)=nodes(1,2).y-limit/15;
                        xGround2(1,3)=xGround2(1,2)+2*limit/25;yGround2(1,3)=yGround2(1,2);
                        %ground
                        index=2;
                        for i=1:2:11
                            xGround2(index,1)=xList(i)+nodes(1,2).x;xGround2(index,2)=xList(i+1)+nodes(1,2).x;
                            yGround2(index,1)=yList(i)+nodes(1,2).y-limit/15;yGround2(index,2)=yList(i+1)+nodes(1,2).y-limit/15;
                            index=index+1;
                        end
                    else
                        %triangle
                        xGround2(1,1)=nodes(1,2).x;yGround2(1,1)=nodes(1,2).y;
                        xGround2(1,2)=nodes(1,2).x-limit/25;yGround2(1,2)=nodes(1,2).y+limit/15;
                        xGround2(1,3)=xGround2(1,2)+2*limit/25;yGround2(1,3)=yGround2(1,2);
                        angle=pi;
                        rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                        index=2;
                        for i=1:2:11
                            point1=rotation*[xList(i);yList(i)];
                            point2=rotation*[xList(i+1);yList(i+1)];
                            xGround2(index,1)=point1(1)+nodes(1,2).x;xGround2(index,2)=point2(1)+nodes(1,2).x;
                            yGround2(index,1)=point1(2)+nodes(1,2).y+limit/15;yGround2(index,2)=point2(2)+nodes(1,2).y+limit/15;
                            index=index+1;
                        end
                    end
                case Joint.GroundWelded
                    if nodes(1,1).y>nodes(1,2).y
                        index=1;
                        xGround2=zeros(1,6);
                        yGround2=zeros(1,6);
                        for i=1:2:11
                            xGround2(index,1)=xList(i)+nodes(1,2).x;xGround2(index,2)=xList(i+1)+nodes(1,2).x;
                            yGround2(index,1)=yList(i)+nodes(1,2).y-limit/50;yGround2(index,2)=yList(i+1)+nodes(1,2).y-limit/50;
                            index=index+1;
                        end
                    else
                        angle=pi;
                        rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                        index=1;
                        xGround2=zeros(1,6);
                        yGround2=zeros(1,6);
                        for i=1:2:11
                            point1=rotation*[xList(i);yList(i)];
                            point2=rotation*[xList(i+1);yList(i+1)];
                            xGround2(index,1)=point1(1)+nodes(1,2).x;xGround2(index,2)=point2(1)+nodes(1,2).x;
                            yGround2(index,1)=point1(2)+nodes(1,2).y+limit/50;yGround2(index,2)=point2(2)+nodes(1,2).y+limit/50;
                            index=index+1;
                        end
                    end
            end
        end
        
        
    end
    
end

