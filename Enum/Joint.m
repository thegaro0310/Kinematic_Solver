classdef Joint
    %enum class for joints
    methods(Static)
        function newJoint=getJoint(oldJoint)
            switch oldJoint
                case -3
                    newJoint=Joint.GroundSlider;
                case -2
                    newJoint=Joint.GroundPin;
                case -1
                    newJoint=Joint.GroundWelded;
                case 1
                    newJoint=Joint.Welded;
                case 2
                    newJoint=Joint.Pin;
                case 3
                    newJoint=Joint.Slider;
                otherwise
                    newJoint=Joint.NA;
            end
        end
    end
    enumeration
        Pin,GroundPin,Welded,GroundWelded,Slider,GroundSlider,NA
    end
    
end

