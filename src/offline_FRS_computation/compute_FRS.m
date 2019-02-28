function out = compute_FRS(prob)
%% parse inputs
    disp('Extracting parameters')
    t = prob.t ; % time
    z = prob.z ; % states
    k = prob.k ; % trajectory parameters
    f = prob.f ; % dynamics
    hZ = prob.hZ ; % state space as a semi-algebraic set
    hK = prob.hK ; % param space as a semi-algebraic set
    hZ0 = prob.hZ0 ; % initial conds as a semi-algebraic set
    degree = prob.degree ; % degree of w polynomial
    int_ZK = prob.int_ZK ; % cost function (integral of w(z,k) over Z x K)
    
    % time horizon as a semi-algebraic set (default is t \in [0,1])
    if isfield(prob,'hT')
       hT = prob.hT ;
    else 
       hT = t * (1-t);
    end
    
    % tracking error function g (default is to not have g, so we don't need
    % to compute the q decision variable, which makes the offline FRS
    % computation less memory-intensive)
    if isfield(prob, 'g')
        g = prob.g ;
    end
    
    
%% define variables
    disp('Defining problem variables')
    
    % initialize program and indeterminate vars
    prog = spotsosprog;
    prog = prog.withIndeterminate(t) ;
    prog = prog.withIndeterminate(z) ;
    prog = prog.withIndeterminate(k) ;
    
    % create monomials for (v,w,q) polynomials
    vmon = monomials([t;z;k], 0:degree) ;
    wmon = monomials([z;k], 0:degree) ;

    if exist('g','var')
        q = msspoly(zeros([size(g,2),1])) ;
        qmon = monomials([t;z;k], 0:degree) ;
        for qidx = 1:length(q)
            [prog, q(qidx), ~] = prog.newFreePoly(qmon) ;
        end
    end    
    
    % create (v,w,q) decision variable polynomials
    [prog, v, ~] = prog.newFreePoly(vmon) ;
    [prog, w, wcoeff] = prog.newFreePoly(wmon) ;
    if exist('g','var')
        for qidx = 1:length(q)
            [prog, q(qidx), ~] = prog.newFreePoly(qmon) ;
        end
    end

    % create variables for the equality constraints of the program (D)
    t0 = 0 ;
    v0 = subs(v,t,t0) ;
    dvdt = diff(v,t) ;
    dvdz = diff(v,z) ;
    Lfv = dvdt + dvdz*f ;
    if exist('g','var')
        Lgv = dvdz*g ;
    end

%% define constraints
    disp('Defining constraints')
    tic

    % if tracking error is included, we need the following constraints:
    if exist('g','var')
        % -Lfv - q > 0 on T x Z x K
        prog = sosOnK(prog, -Lfv - ones(size(q))'*q, [t;z;k], [hT; hZ; hK], degree) ;
    else
        % -Lfv > 0 on T x Z x K
        prog = sosOnK(prog, -Lfv, [t;z;k], [hT; hZ; hK], degree) ;
    end

    % v(t,.) + w > 1 on T x Z x K
    if reduce_vmon
        prog = sosOnK(prog, v + w - 1, [t;z;k], [hT; hZ; hK], degree, vmonspec) ;
    else
        prog = sosOnK(prog, v + w - 1, [t;z;k], [hT; hZ; hK], degree) ;
    end

    % w > 0 on Z x K
    prog = sosOnK(prog, w, [z;k], [hZ; hK], degree) ;

    % -v(0,.) > 0 on Z0 x K
    prog = sosOnK(prog, -v0, [z(zidx);k], [hZ0; hK], degree) ;

    % v(t,.) > 0 on T x dX x K if boundary is active
    if enforce_boundary
        for dzidx = 1:length(hZ)
            prog = sosOnK(prog, v, [t;z;k], [hT; hZ ; -hZ(dzidx); hK], degree) ;
        end
    end
    
    % if tracking error is included, we need the following constraints:
    % q - Lgv > 0 on T x Z x K
    % q + Lgv > 0 on T x Z x K
    % q > 0 on T x Z x K
    if exist('g','var')
        for qidx = 1:length(q)
            prog = sosOnK(prog, q(qidx) - Lgv(qidx), [t;z;k], [hT;hZ;hK], degree) ;
            prog = sosOnK(prog, q(qidx) + Lgv(qidx), [t;z;k], [hT;hZ;hK], degree) ;
            prog = sosOnK(prog, q(qidx), [t;z;k], [hT;hZ;hK], degree) ;
        end
    end    

    toc

%% create problem object for computation
    disp('Setting up FRS problem')
    obj = int_ZK(wmon)' * wcoeff ; 
    out.prog = prog ;
    out.wmon = wmon ;
    out.wcoeff = wcoeff ;
    out.obj_func = obj ;
    options = spot_sdp_default_options() ;
    options.verbose = 1 ; 
    options.domain_size = 1;
    options.solveroptions = [];

%% solve for FRS
    disp('Solving for the FRS')
    start_tic = tic ;
    sol = prog.minimize(obj, @spot_mosek, options);
    end_time = toc(start_tic) ;

%% extract problem info
    disp('Extracting results')
    out.v = sol.eval(v) ;
    out.w = sol.eval(w) ;
    out.final_cost = sol.eval(obj) ;
    out.duration = end_time ;

    disp([num2str(end_time/3600), ' hrs elapsed solving problem'])
end
