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
            % Add one extra multiplier for the displacement constraint
            obj.lambda_current = zeros(length(ceq) + 1, 1);
            
            % 3. Find the initial baseline X distance before forces are applied
            [outputMatrix, indexList, ~] = loadSim.static.kinematic.findInputMatrix(obj.x_current, []);
            [obj.initialDispX, ~, ~, ~] = loadSim.static.kinematic.solveStaticEquations(outputMatrix, loadSim.static.forces(obj.forceIndex).kinematicEquations, indexList);
            loadSim.static.forces(obj.forceIndex).initialDistanceX = obj.initialDispX;
            loadSim.static.forces(obj.forceIndex).active = 1; % Activate the force element (magnitude overridden by constraint)
        end
        
        function [dispX, reqForce] = step(obj, targetDispXRelative, solverType)
            % True displacement control: No secant loops, no guessing forces!
            if nargin < 3
                solverType = 'custom'; % Default to the custom NR solver
            end
            
            targetDispX = obj.initialDispX + targetDispXRelative;
            
            if strcmpi(solverType, 'fsolve')
                [x_val, lam_val, exitflag] = obj.solveEquilibriumFsolve(obj.x_current, obj.lambda_current, targetDispX);
            else
                [x_val, lam_val, exitflag] = obj.solveEquilibriumNR(obj.x_current, obj.lambda_current, targetDispX);
            end
            
            if exitflag
                obj.x_current = x_val;
                obj.lambda_current = lam_val;
            else
                warning('Newton-Raphson failed to converge for target displacement %.3f. Try smaller steps.', targetDispXRelative);
            end
            
            % Return the target displacement directly and extract the required force
            % from the Lagrange multiplier associated with the displacement constraint.
            dispX = targetDispXRelative;
            reqForce = -lam_val(end); 
            obj.currentForce = reqForce;
        end
        
        function [x, lambda, exitflag] = solveEquilibriumNR(obj, x0, lambda0, targetDispX)
            % Solves the equilibrium balance and kinematic loop constraints 
            % simultaneously via matrix inversion using the KKT conditions.
            
            % Enforce displacement control by setting explicit external force to 0.
            % The reaction force to achieve the target displacement will emerge naturally 
            % as the Lagrange multiplier for the new constraint.
            obj.loadSim.static.forces(obj.forceIndex).xValue = 0;
            obj.loadSim.static.forces(obj.forceIndex).yValue = 0;
            
            x = x0;
            lambda = lambda0;
            tol = 1e-5;
            maxIter = 30;
            
            for iter = 1:maxIter
                % 1. Evaluate Energy Gradient (g) and Hessian (H) with no external force
                [~, g] = obj.loadSim.fMinConFunc(x, [], 100);
                lam_struct.eqnonlin = lambda(1:end-1);
                H = obj.loadSim.hessianfcn(x, lam_struct, [], 100);
                
                % 2. Evaluate Kinematic constraints
                [~, ceq_kin, ~, gradceq_kin] = obj.loadSim.fMinConCostraint(x, []);
                J_kin_T = gradceq_kin;
                
                % 3. Evaluate Displacement constraint
                [outMat, idxList, ~] = obj.loadSim.static.kinematic.findInputMatrix(x, []);
                [dispX_current, ~, grad_disp, hess_disp] = obj.loadSim.static.kinematic.solveStaticEquations(...
                    outMat, obj.loadSim.static.forces(obj.forceIndex).kinematicEquations, idxList);
                
                ceq_disp = dispX_current - targetDispX;
                J_disp_T = grad_disp(:, 1); % X-direction gradient
                
                % Assemble full constraint arrays
                ceq = [ceq_kin; ceq_disp];
                J_T = [J_kin_T, J_disp_T];
                J = J_T';
                
                % Include displacement Hessian contribution (lambda_disp * H_disp)
                H_total = H + lambda(end) * hess_disp{1};
                
                % Compute residuals
                R_x = g + J_T * lambda;
                R_lam = ceq;
                Residual = -[R_x; R_lam];
                
                % Check convergence
                if norm(Residual) < tol
                    exitflag = 1;
                    return;
                end
                
                % Build and solve KKT Matrix
                KKT = [H_total, J_T; J, sparse(length(ceq), length(ceq))] + 1e-8 * speye(length(x)+length(ceq));
                delta = KKT \ Residual;
                
                % Newton Update
                x = x + delta(1:length(x)); 
                lambda = lambda + delta(length(x)+1:end);
            end
            
            exitflag = 0; % Failed to converge
        end
        
        function [x, lambda, exitflag] = solveEquilibriumFsolve(obj, x0, lambda0, targetDispX)
            % Solves the equilibrium KKT system using MATLAB's built-in fsolve.
            
            % Enforce displacement control by setting explicit external force to 0.
            obj.loadSim.static.forces(obj.forceIndex).xValue = 0;
            obj.loadSim.static.forces(obj.forceIndex).yValue = 0;
            
            z0 = [x0; lambda0]; % Combine state and multipliers into a single vector
            
            options = optimoptions('fsolve', ...
                'Display', 'iter', ...
                'SpecifyObjectiveGradient', true, ... % fsolve uses this name for the Jacobian
                'Algorithm', 'trust-region-dogleg', ...
                'FunctionTolerance', 1e-8, ...
                'StepTolerance', 1e-8);
                
            % Define the residual function handle
            residualFunc = @(z) calculateResiduals(z, obj, targetDispX);
            
            % Call fsolve
            [z_sol, ~, exitflag, ~] = fsolve(residualFunc, z0, options);
            
            if exitflag > 0
                % Unpack the solution vector
                x = z_sol(1:length(x0));
                lambda = z_sol(length(x0)+1:end);
            else
                % Return original values on failure
                x = x0;
                lambda = lambda0;
                exitflag = 0;
            end
            
            % --- Nested Residual Function ---
            function [F, Jac] = calculateResiduals(z, solverObj, target)
                x_current = z(1:length(solverObj.x_current));
                lambda_current = z(length(solverObj.x_current)+1:end);
                
                [~, g] = solverObj.loadSim.fMinConFunc(x_current, [], 100);
                lam_struct.eqnonlin = lambda_current(1:end-1);
                H = solverObj.loadSim.hessianfcn(x_current, lam_struct, [], 100);
                
                [~, ceq_kin, ~, gradceq_kin] = solverObj.loadSim.fMinConCostraint(x_current, []);
                [outMat, idxList, ~] = solverObj.loadSim.static.kinematic.findInputMatrix(x_current, []);
                [dispX_curr, ~, grad_disp, hess_disp] = solverObj.loadSim.static.kinematic.solveStaticEquations(...
                    outMat, solverObj.loadSim.static.forces(solverObj.forceIndex).kinematicEquations, idxList);
                
                ceq = [ceq_kin; dispX_curr - target];
                J_T = [gradceq_kin, grad_disp(:, 1)];
                
                F = [g + J_T * lambda_current; ceq];
                
                if nargout > 1
                    J = J_T';
                    H_total = H + lambda_current(end) * hess_disp{1};
                    Jac = [H_total, J_T; J, sparse(length(ceq), length(ceq))];
                end
            end
        end
    end
end
