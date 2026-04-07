classdef WorkSpace
    %work space of the current design window
    properties
        sizeX;
        sizeY;
        unitLength;
        unitForce;
        origin;
    end
    
    methods
        function obj=WorkSpace(sizeX,sizeY,unitLength,unitForce)
            obj.sizeX=sizeX;
            obj.sizeY=sizeY;
            obj.unitLength=unitLength;
            obj.unitForce=unitForce;
            obj.origin=[0 0];
        end
        
        function xSize=getX(obj)
            %return x size
            xSize=obj.sizeX;
        end
        
        function ySize=getY(obj)
            %return y size
            ySize=obj.sizeY;
        end
        
        
        function limit=getLimit(obj)
            %return max size
            limit=max([obj.sizeX obj.sizeY])*2;
        end
        
        function position=forcePosition(obj)
            %position for force
            switch(obj.unitForce)
                case ForceUnit.Nanonewton
                    position=1;
                case ForceUnit.Micronewton
                    position=2;
                case ForceUnit.Dyne
                    position=3;
                case ForceUnit.Millinewton
                    position=4;
                case ForceUnit.Newton
                    position=5;
                case ForceUnit.Kilonewton
                    position=6;
                case ForceUnit.Micropound
                    position=7;
                case ForceUnit.Millipound
                    position=8;
                case ForceUnit.Pound
                    position=9;
                case ForceUnit.Kilopound
                    position=10;
            end
        end
        
        
        function position=lengthPosition(obj)
            %conversion factor for length
            switch(obj.unitLength)
                case LengthUnit.Nanometer
                    position=1;
                case LengthUnit.Micrometer
                    position=2;
                case LengthUnit.Millimeter
                    position=3;
                case LengthUnit.Centimeter
                    position=4;
                case LengthUnit.Meter
                    position=5;
                case LengthUnit.Milliinch
                    position=6;
                case LengthUnit.Inch
                    position=7;
                case LengthUnit.Foot
                    position=8;   
            end
        end
        
        
        
        function conversion=forceFactor(obj)
            %conversion factor for force
            switch(obj.unitForce)
                case ForceUnit.Nanonewton
                    conversion=1e-9;
                case ForceUnit.Micronewton
                    conversion=1e-6;
                case ForceUnit.Dyne
                    conversion=1e-5;
                case ForceUnit.Millinewton
                    conversion=1e-3;
                case ForceUnit.Newton
                    conversion=1;
                case ForceUnit.Kilonewton
                    conversion=1e3;
                case ForceUnit.Micropound
                    conversion=4.44822162e-6;
                case ForceUnit.Millipound
                    conversion=4.44822162e-3;
                case ForceUnit.Pound
                    conversion=4.44822162;
                case ForceUnit.Kilopound
                    conversion=4.44822162e+3;
            end
        end
        
        function conversion=lengthFactor(obj)
            %conversion factor for length
            switch(obj.unitLength)
                case LengthUnit.Nanometer
                    conversion=1e-9;
                case LengthUnit.Micrometer
                    conversion=1e-6;
                case LengthUnit.Millimeter
                    conversion=1e-3;
                case LengthUnit.Centimeter
                    conversion=1e-2;
                case LengthUnit.Meter
                    conversion=1;
                case LengthUnit.Milliinch
                    conversion=0.02541e-3;
                case LengthUnit.Inch
                    conversion=0.0254;
                case LengthUnit.Foot
                    conversion=0.3048;   
            end
        end
        
        function conversion=momentFactor(obj)
            %conversion factor for moment
            conversion=obj.lengthFactor()*obj.forceFactor();
        end
        
        function conversion=torsionSpringFactor(obj)
            %conversion factor for torsion spring
            conversion=obj.lengthFactor()*obj.forceFactor();
        end
        
        function conversion=linearSpringFactor(obj)
            %conversion factor for linear spring
            conversion=obj.forceFactor()/obj.lengthFactor();
        end
        
        function conversion=EFactor(obj)
            %conversion factor for pressure
            if obj.unitLength == LengthUnit.Inch ||obj.unitLength == LengthUnit.Foot || obj.unitLength ==  LengthUnit.Milliinch
                %ksi
                conversion=	6894757.293178;
            else
                %gpa
                conversion=1e+9;
            end
        end
        
        function string=getEString(obj)
            %get E string
            if obj.unitLength == LengthUnit.Inch || obj.unitLength == LengthUnit.Foot ||obj.unitLength ==  LengthUnit.Milliinch
                string='Ksi';
            else
                string='GPa';
            end
        end
        
        function disp(nodes)
            %for displaying nodes
            for i=1:length(nodes)
                nodes(i).sizeX
                nodes(i).sizeY
                nodes(i).unitLength
                nodes(i).unitForce
            end
        end
        
    end
    
end

