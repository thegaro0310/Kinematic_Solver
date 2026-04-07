classdef Module
    %enum class for current module
    methods(Static)
        function str=getString(unit)
            switch(unit)
                 case Module.Overview
                    str='Right Click a component to edit';
                case Module.Sketch
                    str='Sketch the mechanism with the mouse - Press Esc to pause sketching';
                case Module.Model
                    str='Select a link to modify joints - Add a rigid plate by selecting more than one link while holding ctrl';
                case Module.Constrain
                    str='Select a link or a node to modify. You should specify all length constraints before adding angle constraints.';
                case Module.Linear
                    str='Select a linear spring to modify the stiffness';
                case Module.Torsion
                    str='Select a torsion spring to modify the stiffness';
                case Module.Force
                    str='Select a force to modify - Select  a link to add a force';
                case Module.Moment
                    str='Select a moment to modify - Select a link to add a moment';
                case Module.Compliant
                    str='Select a link to make it compliant';
                case Module.Free
                    str='Drag a link or a slider to animate the linkage';
                case Module.Range
                    str='Kinematics:Range';
                case Module.Load
                    str='Activate or deactivate forces with right click';
                case Module.Distance
                    str='Target Distance:not set,Input Force:not set(right click)';
                case Module.MechanicalAdvantage
                    str='Input Load and Output Load are not set by right click';
                case Module.EnergyPlot
                    str='Select a slider node or a crank link to start analysis.';
                case Module.PostProcessing
                    str='Select a slider node or a crank link to start analysis.';
                case Module.KinematicSynthesis
                    str='Load a target coupler curve and select the coupler point and driving link by right click.';
                case Module.FlexuralStiffnessSynthesis
                    str='First add a distance target and then choose optimization variables by right clicking.';
                case Module.BiStableSynthesis
                    str='Please wait until analysis is complete.';
                case Module.ConstantForceSynthesis
                    str='Select optimization variables by right clicking at links.';
                case Module.Other
                    str='Post-Processing Module has started.';
            end
        end
    end
    enumeration
        Overview,Sketch,Model,Constrain,Linear,Torsion,Force,Moment,Compliant,Free,Range,Load,Distance,MechanicalAdvantage,EnergyPlot,PostProcessing,KinematicSynthesis,FlexuralStiffnessSynthesis,BiStableSynthesis,ConstantForceSynthesis,Other
    end
    
end

