classdef LengthUnit
    %enum class for length
    methods(Static)
        function str=getString(unit)
            switch(unit)
                case LengthUnit.Nanometer
                    str='nm';
                case LengthUnit.Micrometer
                    str='µm';
                case LengthUnit.Millimeter
                    str='mm';
                case LengthUnit.Centimeter
                    str='cm';
                case LengthUnit.Meter
                    str='m';
                case LengthUnit.Milliinch
                    str='mInch';
                case LengthUnit.Inch
                    str='inch';    
                case LengthUnit.Foot
                    str='ft';     
            end
        end
        
        function unit=lengthPosition(position)
            %position for force
            switch(position)
                case 1
                    unit=LengthUnit.Nanometer;
                case 2
                    unit= LengthUnit.Micrometer;
                case 3
                    unit=LengthUnit.Millimeter;
                case 4
                    unit=LengthUnit.Centimeter;
                case 5
                    unit=LengthUnit.Meter;
                case 6
                    unit=LengthUnit.Milliinch;
                case 7
                    unit=LengthUnit.Inch;
                case 8
                    unit=LengthUnit.Foot;
            end
        end
        
    end
    enumeration
        Nanometer,Micrometer,Millimeter,Centimeter,Meter,Milliinch,Inch,Foot   
    end
    
end

