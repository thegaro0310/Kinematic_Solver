classdef CrossSection
    %a structure type cross section class
    properties
        thickness=0;
        width=0;
        E=0;
        type=[];
        %prb details
        prbModel=0;
        order=0;
        %other models
        segments=[];
        torsionSprings=TorsionSpring.empty;
        nodes=Node.empty;
        inverse=false;
    end
    
    methods
        function obj=CrossSection(thickness,width,E,type)
            %constructor
            obj.thickness=thickness;
            obj.width=width;
            obj.E=E;
            obj.prbModel=[];
            obj.segments=[];
            obj.order=0;
            obj.type=type;
        end
        
        function I=getI(obj)
            %area moment of inertia
            I=1/12*obj.thickness^3*obj.width;
        end
        
        function A=getA(obj)
            %area moment of inertia
            A=obj.thickness*obj.width;
        end
        
        function obj=fillNodes(obj,node1,node2,workspace,reverse)
            %produce nodes
            obj.torsionSprings=TorsionSpring.empty;
            beamLength=sqrt((node2.x-node1.x)^2+(node2.y-node1.y)^2);
            if obj.type == BeamType.PRB
                prbModelCur=obj.prbModel;
                if logical(reverse)
                    prbModelCur(:,2)=flipud(prbModelCur(:,2));
                    prbModelCur=flipud(prbModelCur);
                    obj.inverse=true;
                end
                index=1;
                for i=2:size(prbModelCur,1)
                    lengthSegment=sum(prbModelCur(1:i-1,1));
                    x=node1.x+(node2.x-node1.x)*lengthSegment;
                    y=node1.y+(node2.y-node1.y)*lengthSegment;
                    if ~isinf(prbModelCur(i,2))
                        magnitude=prbModelCur(i,2)*obj.E*workspace.EFactor()*obj.getI()/beamLength*workspace.lengthFactor()^3;
                        magnitude=magnitude/workspace.torsionSpringFactor();
                        obj.torsionSprings(index)=TorsionSpring(0,0,0,Point(x,y),magnitude);
                        index=index+1;
                    end
                end
            elseif obj.type == BeamType.CBCM ||  obj.type == BeamType.Mlinear
                for i=2:size(obj.segments,1)
                    lengthSegment=sum(obj.segments(1:i-1,1));
                    x=node1.x+(node2.x-node1.x)*lengthSegment;
                    y=node1.y+(node2.y-node1.y)*lengthSegment;
                    obj.nodes(i-1)=Node(0,x,y);
                end
            end
        end
        
        function obj=drawNodes(obj,mainFig,limit,parent)
            %draw nodes
            for i=1:length(obj.torsionSprings)
                obj.torsionSprings(i)=obj.torsionSprings(i).drawPRB(mainFig,parent.getWorkspace());
            end
            if obj.type == BeamType.CBCM
                for i=1:length(obj.nodes)
                    obj.nodes(i)=obj.nodes(i).drawCBCM(mainFig);
                end
            else
                for i=1:length(obj.nodes)
                    obj.nodes(i)=obj.nodes(i).drawMLM(mainFig);
                end
            end
        end
        
        function obj=deleteNodes(obj)
            %delete nodes
            for i=1:length(obj.torsionSprings)
                obj.torsionSprings(i)=obj.torsionSprings(i).deleteDrawing();
            end
            for i=1:length(obj.nodes)
                obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
            end
        end
    end
    
end

