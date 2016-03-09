% load 
fnameIn = '../data/linearization_terms';
load(fnameIn,'stateDesired','dt','A','B','ABResidual','Q','R');

[A,B,Q] = affinifyLinearization(A,B,ABResidual,Q);

%% save
fnameOut = '../data/affine_linearization_terms';
save(fnameOut,'stateDesired','dt','A','B','Q','R');