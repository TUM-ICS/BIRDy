function [M, C, G, kineticEnergy, potentialEnergy] = computeDynamicModel(robotName, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, Mass, InertiaCOM, Ia, Gravity, Q, Qp, options, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

if nargin < 8 || isempty(options)
    options.algorithm = 'lagrange';
    options.verif = false;
end

[~,~,nbDOF] = size(HT_cmi_world_Moment);

%% Robot Dynamics:

% Potential Energy:
P = sym(zeros(nbDOF,1));
for i=1:nbDOF
    P(i) = Mass(i)*Gravity'*HT_cmi_world_Moment(1:3,4,i);
end
potentialEnergy = sum(P);

switch (options.algorithm)
    
    case 'lagrange'
        % Compute the symbolic equation for the inertia tensor:
        
        fprintf('Computing the inertia tensor M... \n');
        M = sym(zeros(nbDOF));
        for i = 1:nbDOF
            M = M + Mass(i)*J_cmi_world_Moment(1:3,:,i)'*J_cmi_world_Moment(1:3,:,i) + J_cmi_world_Moment(4:6,:,i)'*HT_cmi_world_Moment(1:3,1:3,i)*InertiaCOM(:,:,i)*HT_cmi_world_Moment(1:3,1:3,i)'*J_cmi_world_Moment(4:6,:,i);
        end
        M = M + diag(Ia); % Adding actuators inertia
        
        fprintf('Simplifying M... \n');
        M = simplify(M,'Seconds',30);
        
        % Compute the symbolic equation for the centripetal and Coriolis matrix using Christoffel symbol:
        
        fprintf('Computing the centripetal and Coriolis matrix C using Christoffel symbol... \n');
        C = sym(zeros(nbDOF));
        for k = 1:nbDOF
            for j = 1:nbDOF
                for i = 1:nbDOF
                    C(k,j) = C(k,j) + (1/2)*(diff(M(k,j),Q(i)) + diff(M(k,i),Q(j)) - diff(M(i,j),Q(k)))*Qp(i);
                end
            end
        end
        fprintf('Simplifying C... \n');
        C = simplify(C,'Seconds',30);
        
        % Compute the symbolic equation for the gravitational torques vector:
        
        fprintf('Computing the gravitational torques vector G... \n');
        G = sym(zeros(nbDOF,1));
        for i = 1:nbDOF
            G(i) = diff(potentialEnergy,Q(i));    % Partial derrivation of PT wrt q_i
        end
        fprintf('Simplifying G... \n');
        G = simplify(G,'Seconds',30);
        
    case 'newton'
        fprintf('Computing the inertia tensor M, Coriolis matrix C and gravitational torques vector G... \n');
        % Compute the symbolic equation for the inertia tensor, centripetal/Coriolis matrix and gravity torque vector at the same time:
        M_i = sym(zeros(nbDOF,nbDOF,nbDOF));
        C_i = sym(zeros(nbDOF,nbDOF,nbDOF));
        G_i = sym(zeros(nbDOF,nbDOF));
        
        for i = 1:nbDOF
            M_i(:,:,i) = Mass(i)*J_cmi_world_Moment(1:3,:,i)'*J_cmi_world_Moment(1:3,:,i) + J_cmi_world_Moment(4:6,:,i)'*HT_cmi_world_Moment(1:3,1:3,i)*InertiaCOM(:,:,i)*HT_cmi_world_Moment(1:3,1:3,i)'*J_cmi_world_Moment(4:6,:,i);
            C_i(:,:,i) = Mass(i)*J_cmi_world_Moment(1:3,:,i)'*Jd_cmi_world_Moment(1:3,:,i) + J_cmi_world_Moment(4:6,:,i)'*HT_cmi_world_Moment(1:3,1:3,i)*InertiaCOM(:,:,i)*HT_cmi_world_Moment(1:3,1:3,i)'*Jd_cmi_world_Moment(4:6,:,i) +  J_cmi_world_Moment(4:6,:,i)'*Skew(J_cmi_world_Moment(4:6,:,i)*Qp)*HT_cmi_world_Moment(1:3,1:3,i)*InertiaCOM(:,:,i)*HT_cmi_world_Moment(1:3,1:3,i)'*J_cmi_world_Moment(4:6,:,i);
            G_i(:,i) = Mass(i)*J_cmi_world_Moment(1:3,:,i)'*Gravity;
        end
        if nbDOF>1
            fprintf('Simplifying M... \n');
            %         M = simplify(sumnd(M_i,3) + diag(Ia),'Seconds',30);
            M = sumnd(M_i,3) + diag(Ia); % Adding actuators inertia;
            
            fprintf('Simplifying C... \n');
            %         C = simplify(sumnd(C_i,3),'Seconds',30);
            C = sumnd(C_i,3);
            
            fprintf('Simplifying G... \n');
            %         G = simplify(sumnd(G_i,3),'Seconds',30);
            G = sumnd(G_i,2);
        else
            M = M_i + diag(Ia); % Adding actuators inertia;
            C = C_i;
            G = G_i;
        end
        
    otherwise
        error('Robot Dynamic Computation: Unknown Algorithm');
end

if strcmp(robotName, 'TX40') || strcmp(robotName, 'RX90') % Take the coupling between joints 5 and 6 into account.
    M(5,6) = M(5,6) + Ia(6);
    M(6,5) = M(6,5) + Ia(6);
end

% Kinetic Energy:
kineticEnergy = (1/2)*Qp'*M*Qp;

%% Verificaton Routines:

if options.verif == true
    % Verification of dynamic properties:
    fprintf('Verification of Dynamic properties...\n');
    error_M = M'-M;
    error_M = simplify(error_M,'Seconds',30);
    if error_M == zeros(nbDOF,nbDOF)
        fprintf('First dynamic property verified: M is symmetric.\n');
    else
        error('First dynamic property violated: M is NOT symmetric.\n');
    end
    
    M_dot = timeDerivative(M, Q, Qp);
    N = M_dot - 2*C;
    N = simplify(N,'Seconds',30);
    error_N = N'+N;
    error_N = simplify(error_N,'Seconds',30);
    
    if error_N == zeros(nbDOF,nbDOF)
        fprintf('Second dynamic property verified: M_dot - 2*C is skew symmetric.\n');
    else
        error('Second dynamic property violated: M_dot - 2*C is NOT skew symmetric.\n');
    end
end
end


function M = sumnd(M,dim)
s=size(M);
M=permute(M,[setdiff(1:ndims(M),dim),dim]);
M=reshape(M,[],s(dim));
M=sum(M,2);
s(dim)=1;
M=reshape(M,s);
end
