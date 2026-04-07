classdef Connection
    %connection class for 
    
    properties
        nodes;
        joints;
        sliderAngle;
        geometry
        linearSpringMagnitude;
        segments;
        type;
    end
    
    methods
        function obj=Connection(nodes,joints,sliderAngle,geometry,linearSpringMagnitude)
            %constructor
            obj.nodes=nodes;
            obj.joints=joints;
            obj.sliderAngle=sliderAngle*pi/180;
            obj.geometry=geometry;
            obj.linearSpringMagnitude=linearSpringMagnitude;
            if isempty(geometry)
                obj.type=BeamType.Kinematics;
            else
                obj.type=geometry.type;
            end
        end
        
        function beam = convert2Beam(obj,id,allNodes,workspace)
            %convert connection to beam
            point1=allNodes(obj.nodes(1)).getNode();
            point2=allNodes(obj.nodes(2)).getNode();
            angle=atan2(point2.y-point1.y,point2.x-point1.x);
            length=sqrt((point2.y-point1.y)^2+(point2.x-point1.x)^2);
%             if obj.type~=BeamType.Kinematics
%                 obj.type=BeamType.BCM;
%             end
            switch obj.type
                case BeamType.Kinematics
                    beam=KinematicsBeam(id,obj.nodes,obj.joints,obj.sliderAngle,angle,length,obj.geometry,obj.linearSpringMagnitude,allNodes);
                case BeamType.PRB
                    beam=PrbBeam(id,obj.nodes,obj.joints,angle,length,obj.geometry,workspace);
                case BeamType.BCM
                    beam=BcmBeam(id,obj.nodes,obj.joints,angle,length,obj.geometry);
                case BeamType.CBCM
                    beam=CbcmBeam(id,obj.nodes,obj.joints,angle,length,obj.geometry); 
                case BeamType.Linear
                    beam=LinearBeam(id,obj.nodes,obj.joints,angle,length,obj.geometry);
                case BeamType.Mlinear
                    beam=MlinearBeam(id,obj.nodes,obj.joints,angle,length,obj.geometry); 
            end
        end
        
        function variableCount=getVariableCount(obj)
            %get variable count
%             if obj.type~=BeamType.Kinematics
%                 obj.type=BeamType.BCM;
%             end 
            switch obj.type
                case BeamType.Kinematics
                    variableCount=2;
                case BeamType.PRB
                    variableCount=sum(~isinf(obj.geometry.prbModel(:,3)))+size(obj.geometry.prbModel,1);
                    if obj.joints(1,1) == Joint.GroundWelded || obj.joints(1,2) == Joint.GroundWelded
                        variableCount=variableCount-1;
                    end
                case {BeamType.BCM,BeamType.Linear}
                    variableCount=4; 
                case {BeamType.CBCM, BeamType.Mlinear}
                    variableCount=4*length(obj.geometry.segments);     
                otherwise
                    variableCount=2;
            end
        end
        
    end
    
end

