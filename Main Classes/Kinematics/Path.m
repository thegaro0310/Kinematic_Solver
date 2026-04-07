classdef Path
    %path class
    
    properties
        edges=[];
    end
    
    methods
        
        function obj=Path(edges)
            %make a path
            obj.edges=edges;
        end
        
        function obj=addEdge(obj,edge)
            %add a edge
            obj.edges(end+1,:)=edge;
        end
        
        function elements=getUnique(obj)
            %get unique elements
            elements=unique(obj.getExtendedArray())';
        end
        
        function array=getExtendedArray(obj)
            %get the extended array
            array=cell(size(obj.edges,1)*2+1,2) ;
            for i=1:size(obj.edges,1)
                if mod(i,2) == 1
                    %odd
                    array(2*i-1,:)={strcat(num2str(obj.edges(i,1)),'F' ),strcat(num2str(obj.edges(i,1)),'B' )};
                    array(2*i,:)={strcat(num2str(obj.edges(i,1)),'B' ),strcat(num2str(obj.edges(i,2)),'B' )};
                else
                    array(2*i-1,:)={strcat(num2str(obj.edges(i,1)),'B' ),strcat(num2str(obj.edges(i,1)),'F' )};
                    array(2*i,:)={strcat(num2str(obj.edges(i,1)),'F' ),strcat(num2str(obj.edges(i,2)),'F' )};
                end
            end
            %last element
            if mod(size(obj.edges,1),2) == 1
                array(end,:)={strcat(num2str(obj.edges(end,2)),'F' ),strcat(num2str(obj.edges(end,2)),'B' )};
            else
                array(end,:)={strcat(num2str(obj.edges(end,2)),'F' ),strcat(num2str(obj.edges(end,2)),'B' )};
            end
        end

        
        function nicePath=getPath(obj,totalSize)
            %get representation
            nicePath=zeros(length(obj),totalSize);
            for k=1:length(obj)
                if ~isempty(obj(k).edges)
                    nicePath(k,1:2)=[obj(k).edges(1,1) obj(k).edges(1,2)];
                    for i=2:size(obj(k).edges,1)
                        nicePath(k,i+1)=obj(k).edges(i,2);
                    end
                end
            end
        end
        
        function nodes=getAllNodes(obj)
            %returns all unique nodes in the path
            nicePath=obj.getPath();
            nodes=unique(nicePath);
        end
        
        function disp(n)
            %convert into an array
            for i=1:length(n)
                n(i).getPath(10)
            end
        end
        
        
        function empty=isEmpty(obj)
            %check if it is empty
            if isempty(obj.edges)
                empty=1;
            else
                empty=0;
            end
        end
    end
    
end

