function [compileTime] = recompileMex(list, benchmarkSettings)

% Maximum values:
nbSamples = 6e4;

tic
disp("####################################################################################");
disp("################             Recompile .mex functions...            ################");
disp("####################################################################################");
disp(" ");

% cfg = coder.config('mex');
% cfg.GenerateReport=true; % enable a code generation report

if (list(1) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Utils/torqueVector_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Utils/torqueVector_mex' -d 'Benchmark/Robot_Identification_Algorithms/Utils/codegen/mex/torqueVector' -O enable:openmp -O enable:inline -report torqueVector ...
        -args {coder.typeof(ones(20,nbSamples+1), [], 1)}
end

if (list(2) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Utils/observationMatrix_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Utils/observationMatrix_mex' -d 'Benchmark/Robot_Identification_Algorithms/Utils/codegen/mex/observationMatrix' -O enable:openmp -O enable:inline -report observationMatrix ...
        -args {coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(1), [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof(ones(20,nbSamples+1), [], 1), coder.typeof(ones(20,nbSamples+1), [], 1), coder.typeof(ones(20,nbSamples+1), [], 1)}
end

if (list(3) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Utils/integrateClosedLoopDynamics_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Utils/integrateClosedLoopDynamics_mex' -d 'Benchmark/Robot_Identification_Algorithms/Utils/codegen/mex/integrateClosedLoopDynamics' -O enable:openmp -O enable:inline -report integrateClosedLoopDynamics ...
        -args {coder.typeof(ones(100,6e4), [], 1),coder.typeof(ones(100,1), [], 1),coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof(ones(1), [], 1), coder.typeof(ones(1), [], 1), coder.typeof(ones(1), [], 1), coder.typeof(ones(1), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1)}
end

if (list(4) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Kalman/EKF/ekf_opt_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Kalman/EKF/ekf_opt_mex' -d 'Benchmark/Robot_Identification_Algorithms/Kalman/Utils/codegen/mex/ekf_opt' -O enable:openmp -O enable:inline -report ekf_opt ...
        -args {coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof(ones(2*20+100,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(1,1), [], 1),coder.typeof(ones(100,50), [], 1),coder.typeof(ones(20,20), [], 1),coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.newtype('logical', [], 0)}
end

if (list(5) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Kalman/EKF/srekf_opt_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Kalman/EKF/srekf_opt_mex' -d 'Benchmark/Robot_Identification_Algorithms/Kalman/Utils/codegen/mex/srekf_opt' -O enable:openmp -O enable:inline -report srekf_opt ...
        -args {coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof(ones(2*20+100,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(1,1), [], 1),coder.typeof(ones(100,50), [], 1),coder.typeof(ones(20,20), [], 1),coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.newtype('logical', [], 0)}
end

if (list(6) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Kalman/UKF/ukf_opt_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Kalman/UKF/ukf_opt_mex' -d 'Benchmark/Robot_Identification_Algorithms/Kalman/Utils/codegen/mex/ukf_opt' -O enable:openmp -O enable:inline -report ukf_opt ...
        -args {coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof(ones(2*20+100,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.newtype('logical', [], 0), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(1,1), [], 1),coder.typeof(ones(100,50), [], 1),coder.typeof(ones(20,20), [], 1),coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.newtype('logical', [], 0)}
end

if (list(7) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Kalman/UKF/srukf_opt_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Kalman/UKF/srukf_opt_mex' -d 'Benchmark/Robot_Identification_Algorithms/Kalman/Utils/codegen/mex/srukf_opt' -O enable:openmp -O enable:inline -report srukf_opt ...
        -args {coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof(ones(2*20+100,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(1,1), [], 1),coder.typeof(ones(100,50), [], 1),coder.typeof(ones(20,20), [], 1),coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.newtype('logical', [], 0)}
end

if (list(8) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Kalman/CDKF/cdkf_opt_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Kalman/CDKF/cdkf_opt_mex' -d 'Benchmark/Robot_Identification_Algorithms/Kalman/Utils/codegen/mex/cdkf_opt' -O enable:openmp -O enable:inline -report cdkf_opt ...
        -args {coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof(ones(2*20+100,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(0, [], 0), coder.newtype('logical', [], 0), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(1,1), [], 1),coder.typeof(ones(100,50), [], 1),coder.typeof(ones(20,20), [], 1),coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.newtype('logical', [], 0)}
end

if (list(9) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Kalman/CDKF/srcdkf_opt_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Kalman/CDKF/srcdkf_opt_mex' -d 'Benchmark/Robot_Identification_Algorithms/Kalman/Utils/codegen/mex/srcdkf_opt' -O enable:openmp -O enable:inline -report srcdkf_opt ...
        -args {coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof(ones(2*20+100,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(0, [], 0), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(1,1), [], 1),coder.typeof(ones(100,50), [], 1),coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1),coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.newtype('logical', [], 0)}
end

if (list(10) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Kalman/PF/pf_opt_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Kalman/PF/pf_opt_mex' -d 'Benchmark/Robot_Identification_Algorithms/Kalman/Utils/codegen/mex/pf_opt' -O enable:openmp -O enable:inline -report pf_opt ...
        -args {coder.typeof(0, [], 0), coder.typeof(0, [], 0), coder.typeof(ones(2*20+100,100000), [], 1), coder.typeof(ones(1,100000), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(2*20+100,2*20+100), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(1,1), [], 1),coder.typeof(ones(100,50), [], 1),coder.typeof(ones(20,20), [], 1),coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.newtype('logical', [], 0)}
end

if (list(11) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/ML/compute_ML_Cost_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/ML/compute_ML_Cost_mex' -d 'Benchmark/Robot_Identification_Algorithms/ML/codegen/mex/compute_ML_Cost' -O enable:openmp -O enable:inline -report compute_ML_Cost ...
        -args {coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(100,1), [], 1), coder.typeof(ones(20,nbSamples), [], 1), coder.typeof(ones(20,nbSamples), [], 1), coder.typeof(ones(20,nbSamples), [], 1), coder.typeof(ones(20,nbSamples), [], 1), coder.typeof(ones(4*20), [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.newtype('logical', [], 0)}
end

if (list(12) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Utils/integrateClosedLoopNoisyDynamics_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Utils/integrateClosedLoopNoisyDynamics_mex' -d 'Benchmark/Robot_Identification_Algorithms/Utils/codegen/mex/integrateClosedLoopNoisyDynamics' -O enable:openmp -O enable:inline -report integrateClosedLoopNoisyDynamics ...
        -args {coder.typeof(ones(100,6e4), [], 1),coder.typeof(ones(100,1), [], 1),coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,4), [], 1), coder.typeof(ones(3,1), [], 1), coder.typeof(ones(1), [], 1), coder.typeof(ones(1), [], 1), coder.typeof(ones(1), [], 1), coder.typeof(ones(1), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(20,20), [], 1), coder.typeof(ones(1,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof(ones(20,1), [], 1), coder.typeof('mysuperrobotblablablablablabla', [], 1), coder.typeof(ones(20,1), [], 1)}
end

if (list(13) == 1)
    disp('Recompiling Benchmark/Robot_Identification_Algorithms/Utils/weightedObsservationTorque_mex');
    codegen -v -config:mex -o 'Benchmark/Robot_Identification_Algorithms/Utils/weightedObsservationTorque_mex' -d 'Benchmark/Robot_Identification_Algorithms/Utils/codegen/mex/weightedObsservationTorque' -O enable:openmp -O enable:inline -report weightedObsservationTorque ...
        -args {coder.typeof(ones(6e4,100), [], 1),coder.typeof(ones(6e4,1), [], 1),coder.typeof(ones(50), [], 1),coder.typeof(ones(1), [], 1),coder.typeof(ones(1), [], 1)}
end

compileTime = toc;
end
