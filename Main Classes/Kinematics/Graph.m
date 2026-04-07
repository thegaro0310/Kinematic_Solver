classdef Graph < handle
    %Graph    
    properties
        adjacencyMatrix=[];
        allEdges=[];
        allNeighbors=Vector.empty;
        count=0;
        baseVectors;
        base;
    end
    
    methods
         function   obj=Graph(adj)
             %store adjacency matrix
             obj.adjacencyMatrix=adj;
             for i=1:size(obj.adjacencyMatrix,1)
                 neighbors=find(adj(i,:) == 1);
                 for j=1:length(neighbors)
                     %add edge
                     obj.allEdges(end+1,:)=[i,neighbors(j)];
                     %delete connection
                     adj(neighbors(j),i)=0;
                 end
             end
             %find base cycles
             for i=1:size(adj,1)
                 if ~isempty(find(adj(i,:)==1, 1))
                    obj.base=obj.findCycleBase(i);
                    break;
                 end
             end
%              obj.baseVectors=zeros(length(base),size(obj.allEdges,1));
%              %find the base vectors
%              for i=1:length(base)
%                  obj.baseVectors(i,:)=obj.getEdgeVector(base(i));
%              end
         end
         
         
         function location=getEdgeIndex(obj,edge)
             %get the edge index from all edges
             location=0;
             for i=1:size(obj.allEdges,1)
                if length(unique([obj.allEdges(i,:),edge])) == 2
                    location=i;
                    return;
                end
             end
         end
         
         function cycles=findCycles(obj,startNode)
             %find cycles
             cycles=Path.empty;
             %find the base vectors
             for i=1:size(obj.baseVectors,1)
                 path=obj.getPathFromEdgeVector(obj.baseVectors(i,:),startNode);
                 if ~path.isEmpty()
                     cycles(end+1)=path;
                 end
             end
             m=1;
             v=1:size(obj.baseVectors,1);
             for i=1:size(obj.baseVectors,1)
                 %for different length combinations
                 combinations=nchoosek(v,i);
                 for j=1:size(combinations,1)
                     %number of combinations
                     vector=zeros(1,size(obj.baseVectors,2));
                     for k=1:i
                         %length of individual combination
                         vector=xor(vector,obj.baseVectors(combinations(j,k),:));
                         path=obj.getPathFromEdgeVector(vector,startNode);
                         m=m+1;
                     end
                     if ~path.isEmpty()
                         cycles(end+1)=path;
                     end
                 end
             end
         end
         
         
         function path=getPathFromEdgeVector(obj,vector,startNode)
            %get path if possible
            edges=find(vector==1);
            vectorMatrix=zeros(size(edges,1),2);
            
            for i=1:length(edges)
                vectorMatrix(i,:)=[obj.allEdges(edges(i),1),obj.allEdges(edges(i),2)]; 
            end
            path=Path([]);
            if length(find(vectorMatrix == startNode)) == 2
                %we can start and end from start node
                [u,v]=find(vectorMatrix == startNode);
                if v(2)== 1
                    lastConnection=vectorMatrix(u(2),2);
                else
                    lastConnection=vectorMatrix(u(2),1);
                end
                u=u(1);
                v=v(1);
                for i=1:size(vectorMatrix,1)
                    %add an edge
                    if v == 1
                        path=path.addEdge([vectorMatrix(u,1),vectorMatrix(u,2)]);
                        otherNode=vectorMatrix(u,2);
                    else
                        path=path.addEdge([vectorMatrix(u,2),vectorMatrix(u,1)]);
                        otherNode=vectorMatrix(u,1);
                    end
                    vectorMatrix(u,1)=0;vectorMatrix(u,2)=0;
                    [u,v]=find(vectorMatrix == otherNode);
                    %check if even (means not possible)
                    if mod(length(u),2) == 0 && i ~=size(vectorMatrix,1)
                        if otherNode~=startNode
                            path=Path([]);
                        end
                        return;
                    else
                        for j=1:length(u)
                            if i ~= size(vectorMatrix,1) -1
                                if vectorMatrix(u(j),mod(v(j),2)+1) ~= startNode && vectorMatrix(u(j),mod(v(j),2)+1) ~= lastConnection
                                    %other node can not be starting node
                                    %and connection node
                                    u=u(j); 
                                    v=v(j);
                                    break;
                                end
                            elseif vectorMatrix(u(j),mod(v(j),2)+1) ~= startNode
                                %other node can not be starting node
                                u=u(j);
                                v=v(j);
                                break;
                            end
                        end
                    end
                    
                end
            end
         end
         
         function edgeVector=getEdgeVector(obj,path)
            %get edge vector from a path
            edgeVector=zeros(1,size(obj.allEdges,1));
            for i=1:size(path.edges,1)
                edgeVector(obj.getEdgeIndex(path.edges(i,:)))=1;
            end
         end
         
         function cycles=findCycleBase(obj,startNode)
            %find base cycles
            cycles=Path.empty;
            [parent,backEdge]=findCycleBackEdge(obj,startNode);
            for i=1:size(backEdge,1)
                cycle=[backEdge(i,2),backEdge(i,1)];
                previousNode=backEdge(i,1);
                while previousNode ~= backEdge(i,2)
                    currentNode=parent(previousNode); 
                    cycle(end+1,:)=[previousNode,currentNode];
                    previousNode=currentNode ;
                end
                cycles(end+1)=Path(cycle);
            end
         end
         
         function [parent,backEdge]=findCycleBackEdge(obj,startNode)
            %find back edges 
            n=size(obj.adjacencyMatrix,1);
            parent=zeros(n,1);
            backEdge=[];
            visited=zeros(n,1);
            adj=obj.adjacencyMatrix;
            
            currentNode=startNode;
            visited(currentNode)=1;
            while 1
                %find neighbors
                neighbor=find(adj(currentNode,:) == 1);     
                if ~isempty(neighbor)
                    child=neighbor(1);
                    adj(currentNode,child)=0;
                    adj(child,currentNode)=0;
                    if logical(visited(child))
                        backEdge(end+1,:)=[currentNode,child];
                    else
                        parent(child)=currentNode;
                        visited(child)=1;
                        currentNode=child;
                    end
                    
                else
                    currentNode=parent(currentNode);
                end
                %check if start node
                if currentNode == startNode
                    break;
                end
            end    
         end
         
         function cycle=findRandomCycle(obj,startNode)
             %find cycles
             while 1
                 lengthRandom=randi(size(obj.baseVectors,1));
                 %dont stop until you find one
                 combinations=[];
                 while length(unique(combinations)) ~=lengthRandom
                    combinations(end+1)=randi(size(obj.baseVectors,1));
                    combinations=unique(combinations);
                 end
                 vector=zeros(1,size(obj.baseVectors,2));
                 for i=1:lengthRandom
                     vector=xor(vector,obj.baseVectors(combinations(i),:));
                 end
                 path=obj.getPathFromEdgeVector(vector,startNode);
                 if ~path.isEmpty()
                     cycle=path;
                     break;
                 end
             end
         end
         
         function path=findRandomPath(obj,startNode,endNode,repeated)
            %find a random path between two paths
            n=size(obj.adjacencyMatrix,1);
            parent=zeros(n,1);
            visited=zeros(n,1);
            adj=obj.adjacencyMatrix;
            currentNode=startNode;
            randomPath=currentNode;
            visited(currentNode)=1;
            while 1
                %find neighbors
                neighbor=find(adj(currentNode,:) == 1); 
                if ~isempty(neighbor)
                    child=neighbor(randi(length(neighbor)));
                    adj(currentNode,child)=0;
                    adj(child,currentNode)=0;
                    if visited(child) > repeated
                        continue;
                    else
                        parent(child)=currentNode;
                        visited(child)=visited(child)+1;
                        randomPath(end+1)=child;
                        currentNode=child;                   
                    end
                    
                else
                    randomPath(end)=[];
                    currentNode=parent(currentNode);
                end
                %check if start node
                if currentNode == endNode
                    break;
                end
            end
            path=Path([]);
            for i=1:length(randomPath)-1
                path=path.addEdge([randomPath(i) randomPath(i+1)]); 
            end
         end
         
         function allPaths=findAllPaths(obj,startNode,endNode)
            %find all paths between start node and end node
            obj.count=0;
            paths=findAllPathsHelper(obj,startNode,endNode,[]);
            startNodes=find(paths==startNode);
            endNodes=find(paths==endNode);
            allPaths=Path.empty;
            for i=1:length(startNodes)
                edgeArray=zeros(endNodes(i)-startNodes(i),2);               
                for j=1:1:endNodes(i)-startNodes(i)
                    edgeArray(j,:)=[paths(1,startNodes(i)+j-1),paths(1,startNodes(i)+j)];
                end
                allPaths(i)=Path(edgeArray);
            end
         end
         
         function paths=findAllPathsHelper(obj,startNode,endNode,path)
             %find all paths between start node and end node
             if obj.count==1000
                 paths=[];
                return;
             end
             path=[path,startNode];
             if startNode == endNode
                 paths=path;
                 obj.count=obj.count+1;
                 return;
             end
             %find all neighbours
             neighbours=obj.allNeighbors(startNode).elements;
             paths = [];
             for i=1:length(neighbours)
                if isempty(find(path==neighbours(i), 1))
                    newPaths=obj.findAllPathsHelper(neighbours(i),endNode,path) ;
                    paths=[paths,newPaths] ;
                end        
             end
               
         end
    end
    
end

