classdef NewtonRaphsonSolver < handle
    % NewtonRaphsonSolver: A fast KKT-based equilibrium solver with displacement control
    % Replaces the fminsearch/fmincon nested optimization in DistanceAnalysis
    
    properties
        loadSim         % Instance of LoadAnalysis
        x_current       % Current state vector
        lambda_current  % Current Lagrange multipliers for kinematic loops
        currentForce    % Current guessed magnitude of the force
        initialDispX    % Baseline X position of the target node
        forceIndex = 1; % Index of the force to manipulate
    end
    
    methods
        function obj = NewtonRaphsonSolver(loadSim, initialForceGuess)
            obj.loadSim = loadSim;
            obj.currentForce = initialForceGuess;
            
            % 1. Get the initial state vector size and starting guess 'x0'
            [~, ~, obj.x_current] = loadSim.static.kinematic.findInputMatrix(...
                zeros(loadSim.static.kinematic.allBeams{end}.id + loadSim.static.kinematic.allBeams{end}.degreesOfFreedom + 3, 1), []);
            
            % 2. Initialize Lagrange Multipliers for kinematic loop constraints
            [~, ceq, ~, ~] = loadSim.fMinConCostraint(obj.x_current, []);
            obj.lambda_current = zeros(length(ceq), 1);
            
            % 3. Find the initial baseline X distance before forces are applied
            [outputMatrix, indexList, ~] = loadSim.static.kinematic.findInputMatrix(obj.x_current, []);
            [obj.initialDispX, ~] = loadSim.static.kinematic.solveStaticEquations(outputMatrix, loadSim.static.forces(obj.forceIndex).kinematicEquations, indexList);
            loadSim.static.forces(obj.forceIndex).initialDistanceX = obj.initialDispX;
            loadSim.static.forces(obj.forceIndex).active = 1; % Activate the push force
        end
        
        function [dispX, reqForce] = step(obj, targetDispXRelative)
            targetDispX = obj.initialDispX + targetDispXRelative;
            dForce = 0.5; % Perturbation amount to calculate stiffness
            
            % Outer Secant Loop to find the required force for the target displacement
            for secantIter = 1:20
                % 1. Solve true equilibrium at the current guessed force
                [x_val, lam_val, ~] = obj.solveEquilibriumNR(obj.x_current, obj.lambda_current, obj.currentForce);
                
                % Get the resulting X displacement from the solved state
                [outMat, idxList, ~] = obj.loadSim.static.kinematic.findInputMatrix(x_val, []);
                [dispX_val, ~, ~, ~] = obj.loadSim.static.kinematic.solveStaticEquations(outMat, obj.loadSim.static.forces(obj.forceIndex).kinematicEquations, idxList);
                
                error = dispX_val - targetDispX;
                if abs(error) < 1e-4 % Converged to target displacement
                    obj.x_current = x_val;
                    obj.lambda_current = lam_val;
                    break;
                end
                
                % 2. Perturb force to calculate system tangent stiffness
                [x_pert, ~, ~] = obj.solveEquilibriumNR(obj.x_current, obj.lambda_current, obj.currentForce + dForce);
                [outPert, idxPert, ~] = obj.loadSim.static.kinematic.findInputMatrix(x_pert, []);
                [dispXPert, ~, ~, ~] = obj.loadSim.static.kinematic.solveStaticEquations(outPert, obj.loadSim.static.forces(obj.forceIndex).kinematicEquations, idxPert);
                
                % 3. Update force guess based on secant slope
                stiffness = dForce / (dispXPert - dispX_val);
                obj.currentForce = obj.currentForce - error * stiffness;
            end
            
            dispX = dispX_val - obj.initialDispX;
            reqForce = obj.currentForce;
        end
        
        function [x, lambda, exitflag] = solveEquilibriumNR(obj, x0, lambda0, forceMag)
            % Solves the equilibrium balance and kinematic loop constraints 
            % simultaneously via matrix inversion using the KKT conditions.
            obj.loadSim.static.forces(obj.forceIndex).xValue = forceMag;
            x = x0;
            lambda = lambda0;
            tol = 1e-5;
            
            for i = 1:20
                [~, g] = obj.loadSim.fMinConFunc(x, [], 100);
                lam_struct.eqnonlin = lambda;
                H = obj.loadSim.hessianfcn(x, lam_struct, [], 100);
                [~, ceq, ~, gradceq] = obj.loadSim.fMinConCostraint(x, []);
                J_T = gradceq; J = J_T';
                R_x = g + J_T * lambda; R_lam = ceq;
                Residual = -[R_x; R_lam];
                if norm(Residual) < tol, exitflag = 1; return; end
                KKT = [H, J_T; J, sparse(length(ceq), length(ceq))] + 1e-8 * speye(length(x)+length(ceq));
                delta = KKT \ Residual;
                x = x + delta(1:length(x)); lambda = lambda + delta(length(x)+1:end);
            end
            exitflag = 0; % Failed to converge
        end
    end
end
