classdef Vector
    %a vector class for storing a sequence 
    properties
        elements;
    end
    
    methods
        function   obj=Vector(elements)
            obj.elements=elements;
        end
        
        function   obj=addElement(obj,element)
            obj.elements(end+1)=element;
        end
        
        function disp(a)
            for i=1:length(a)
                a(i).elements
            end
        end
    end
    
end

