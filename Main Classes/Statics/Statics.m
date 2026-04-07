classdef Statics
    %statics class
    
    properties
        nodes=Node.empty;
        forces=Force.empty;
        moments=Moment.empty;
        torsionSprings=TorsionSpring.empty;
        kinematic=Kinematics.empty;
        workspace;
        origin;
    end
    
    methods
        
        function obj=Statics(links,nodes,forces,moments,torsionSprings,workspace)
            %constructor
            %error check
            if isempty(forces)
                forces=Force.empty;
            end
            if isempty(moments)
                moments=Moment.empty;
            end
            if isempty(torsionSprings)
                torsionSprings=TorsionSpring.empty;
            end
            %save
            obj.nodes=nodes;
            obj.kinematic=Kinematics(links,nodes,workspace);
            obj.forces=forces;
            obj.moments=moments;
            obj.torsionSprings=torsionSprings;
            obj.workspace=workspace;
            obj=obj.prepare();
        end
        
        function allNodes=addAllNodes(~,nodes)
            %add all nodes to the list
            allNodes=zeros(1,length(nodes)*2);
            for i=1:length(nodes)
                node=nodes(i).getNode();
                allNodes(1,2*i-1)=node.x;
                allNodes(1,2*i)=node.y;
            end
        end
                
        function obj=prepare(obj)
            %update links in forces moments and torsion springs
            %forces
            for i=1:length(obj.forces)
                link=obj.forces(i).getLink();
                if ~isa(obj.kinematic.allBeams{link},'KinematicsBeam')
                    %only can be at 100 percemt
                    if obj.forces(i).distance ~= 0 || (obj.forces(i).distance == 0 && logical(obj.kinematic.allBeams{link}.inverted))
                        obj.forces(i).distance=100;
                    end                        
                elseif logical(obj.kinematic.allBeams{link}.inverted)
                    %reversed, update
                    obj.forces(i).distance=100-obj.forces(i).distance;
                    if logical(obj.forces(i).follower)
                        %change angle for the follower
                        obj.forces(i).angle=obj.forces(i).angle+pi;
                    end
                end
                obj.forces(i).node=obj.kinematic.allBeams{link}.nodes(1,1);
            end
            
            %moments
            for i=1:length(obj.moments)
                link=obj.moments(i).getLink();
                if ~isa(obj.kinematic.allBeams{link},'KinematicsBeam')
                    %only can be at 100 percemt
                    if obj.moments(i).distance ~= 0 || (obj.moments(i).distance == 0 && logical(obj.kinematic.allBeams{link}.inverted))
                        obj.moments(i).distance=100;
                    end
                elseif logical(obj.kinematic.allBeams{link}.inverted)
                    %reversed, update
                    obj.moments(i).distance=100-obj.moments(i).distance;
                end
                obj.moments(i).node=obj.kinematic.allBeams{link}.nodes(1,1);
            end
            %torsionSprings
%             for i=1:length(obj.torsionSprings)
%                 linksTorsion=obj.torsionSprings(i).getLinks();
%                 newLinks=[0 0];
%                 if logical(linksTorsion(1))
%                     newLinks(1)=obj.findLink(links(linksTorsion(1)).getNodes(),obj.kinematic.rigidLinkList);
%                 end
%                 if logical(linksTorsion(2))
%                     newLinks(2)=obj.findLink(links(linksTorsion(2)).getNodes(),obj.kinematic.rigidLinkList);
%                 end
%                 
%                 obj.torsionSprings(i)=obj.torsionSprings(i).setLinks(abs(newLinks));
%             end
           
            %establish the origin
            obj.origin=0;
            for i=1:length(obj.kinematic.allBeams)
                if obj.kinematic.allBeams{i}.joints(1,1) == Joint.GroundWelded || obj.kinematic.allBeams{i}.joints(1,1) == Joint.GroundPin
                    obj.origin=obj.kinematic.allBeams{i}.nodes(1,1);
                    break;
                end
            end
            %find the path between forces and origin
            %for i=1:handles.forceID
            distance=zeros(length(obj.forces),1);
            for i=1:length(obj.forces)
                obj.forces(i).path=[obj.findPath(obj.origin,obj.forces(i).node,obj.kinematic.adjacencyMatrix),obj.forces(i).node]; 
                if length(obj.forces(i).path) > 1 && ( obj.kinematic.allBeams{obj.forces(i).link}.nodes(1,2) == obj.forces(i).path(end-1) )
                    %already coming from second node
                    distance(i)=100-obj.forces(i).distance;
                else
                    distance(i)=obj.forces(i).distance;
                    obj.forces(i).path=[obj.forces(i).path,obj.kinematic.allBeams{obj.forces(i).link}.nodes(1,2)];
                end
            end
            
            %find kinematic equations between origin and forces
            for i=1:length(obj.forces)
                obj.forces(i).kinematicEquations=obj.kinematic.kinematicEquations(obj.forces(i).path,1);
                obj.forces(i).kinematicEquations.beamList(obj.forces(i).link)=obj.forces(i).kinematicEquations.beamList(obj.forces(i).link)*distance(i)/100; 
            end
            
            %initial angles
            for i=1:length(obj.torsionSprings)
                obj.torsionSprings(i)=obj.torsionSprings(i).setInitialAngle(obj.kinematic.allBeams);
            end
            
        end

        
        function path  =findPath(~,startNode,endNode,adjMatrix )
            %obj.findPath Find path between two nodes
            
            distances=zeros(length(adjMatrix),length(adjMatrix));
            distancesIndex=ones(length(adjMatrix));
            vertex=startNode;
            queue=zeros(length(adjMatrix));
            queueIndex=1;
            queue(queueIndex)=vertex;
            while(vertex ~= endNode)
                vertex=queue(queueIndex);
                queueIndex=queueIndex-1;
                neighbors=find(adjMatrix(vertex,:)==1);
                for i=1:length(neighbors)
                    if distancesIndex(neighbors(i)) == 1
                        distances(neighbors(i),:)=distances(vertex,:);
                        distances(neighbors(i),distancesIndex(vertex))=vertex;
                        distancesIndex(neighbors(i))=distancesIndex(vertex)+1;
                        queueIndex=queueIndex+1;
                        queue(queueIndex)=neighbors(i);
                    end
                end
            end
            path=distances(endNode,:);
            path=path(path>0);
        end
        
        
        
        
        
    end
    
    
end

