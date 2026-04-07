classdef ForceUnit
    %enum class for force
    methods(Static)
        function str=getString(unit)
            switch(unit)
                case ForceUnit.Nanonewton
                    str='nN';
                case ForceUnit.Micronewton
                    str='µN';
                case ForceUnit.Dyne
                    str='dyne';
                case ForceUnit.Millinewton
                    str='mN';
                case ForceUnit.Newton
                    str='N';
                case ForceUnit.Kilonewton
                    str='kN';
                case ForceUnit.Micropound
                    str='µIb';
                case ForceUnit.Millipound
                    str='mIb';
                case ForceUnit.Pound
                    str='Ib';
                case ForceUnit.Kilopound
                    str='kIb';
            end
        end
        
        function unit=forcePosition(position)
            %position for force
            switch(position)
                case 1
                    unit=ForceUnit.Nanonewton;
                case 2
                    unit=ForceUnit.Micronewton;
                case 3
                    unit=ForceUnit.Dyne;
                case 4
                    unit=ForceUnit.Millinewton;
                case 5
                    unit=ForceUnit.Newton;
                case 6
                    unit=ForceUnit.Kilonewton;
                case 7
                    unit=ForceUnit.Micropound;
                case 8
                    unit=ForceUnit.Millipound;
                case 9
                    unit=ForceUnit.Pound;
                case 10
                    unit=ForceUnit.Kilopound;
            end
        end
    end
    enumeration
        Nanonewton,Dyne,Micronewton,Millinewton,Newton,Kilonewton,Micropound,Millipound,Pound,Kilopound
    end
    
end


