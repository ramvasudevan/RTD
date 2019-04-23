clear ; clc ; close all

T = 0.5 ;
dt = 0.05 ;
wmax = 1 ;
vmax = 1.25 ;

cost_heading_weight = 0 ;
cost_location_weight = 1 ;

notes = '_segway' ;

%% create symbolic trajectory
disp('Computing symbolic trajectory')
syms w0 v0 kw kv real

% set up wdes and vdes
wdes = kw*wmax ;
vdes = (vmax/2)*kv + (vmax/2) ;

% set up initial condition
z = [0 ; 0; 0 ; w0 ; v0] ;

tmax = 300 ; % timer just in case

tcur = tic ;
for tidx = 0:dt:T
    k1 = symbolic_segway_dynamics(z, kw,kv) ;
    k2 = symbolic_segway_dynamics(z + (dt/2).*k1, kw,kv) ;
    k3 = symbolic_segway_dynamics(z + (dt/2).*k2, kw,kv) ;
    k4 = symbolic_segway_dynamics(z + dt.*k3, kw,kv) ;

    z = z + (dt/6).*(k1 + 2*k2 + 2*k3 + k4) ;
    
    if toc(tcur) > tmax
        break
    end
end
toc(tcur)

%% create symbolic cost function and its derivatives
disp('Generating symbolic cost function and gradients')

tic
syms xdes ydes hdes real

lw = cost_location_weight ; hw = cost_heading_weight ;
xy = z(1:2) ; hh = z(3) ;

% cost
C = (lw/2).*(xy(1) - xdes).^2 + (lw/2).*(xy(2) - ydes).^2 + hw.*(1-cos(hh-hdes)) ;

% gradient
dC = [diff(C,'kw',1), diff(C,'kv',1)] ;

% Hessian
d2C = [diff(dC(1),'kw'), diff(dC(1),'kv') ;
       diff(dC(2),'kw'), diff(dC(2),'kv')] ;

toc

%% check gradients against numeric gradients
v0i = rand(1) ;
w0i = rand(1) ;
xdesi = rand(1) ;
ydesi = rand(1) ;
kvi = rand(1) ;
kwi = rand(1) ;

% fn to plug in to cost
s = @(kwsub,kvsub) double(subs(C,[v0 w0 xdes ydes kw kv],[v0i w0i xdesi ydesi kwsub kvsub])) ;

% numeric gradient
h = 1e-8 ;
dC_num = [(s(kwi+h,kvi) - s(kwi-h,kvi))./(2*h),...
          (s(kwi,kvi+h) - s(kwi,kvi-h))./(2*h)] ;
      
% analytic gradient
dC_anl = double(subs(dC,[v0 w0 xdes ydes kw kv],[v0i w0i xdesi ydesi kwi, kvi])) ;

if norm(dC_num - dC_anl) > 10e-6
    disp('Uh oh!')
else
    disp('Gradients match!')
end

%% save precomputed functions
disp('Saving precomputed functions')
tic
matlabFunction(z,'Vars',[w0 v0 kw kv],'File',['zfn',notes,'.m']) ;
if cost_heading_weight > 0
    matlabFunction(C,'Vars',[w0 v0 xdes ydes hdes kw kv],'File',['Cfn',notes,'.m']) ;
    matlabFunction(dC,'Vars',[w0 v0 xdes ydes hdes kw kv],'File',['dCfn',notes,'.m']) ;
else
    matlabFunction(C,'Vars',[w0 v0 xdes ydes kw kv],'File',['Cfn',notes,'.m']) ;
    matlabFunction(dC,'Vars',[w0 v0 xdes ydes kw kv],'File',['dCfn',notes,'.m']) ;
end
toc

%% symbolic segway dynamics
function zd = symbolic_segway_dynamics(z,wdes,vdes)
    % extract the states
    h = z(3) ;
    w = z(4) ;
    v = z(5) ;
    
    % determine the inputs
    Kg = 2.95 ;
    Ka = 3.0 ;
    
    g = Kg*(wdes - w) ;
    a = Ka*(vdes - v) ;
    
    % calculate the derivatives
    xd = v*cos(h) ;
    yd = v*sin(h) ;
    hd = w ;
    wd = g ;
    vd = a ;
    
    % make the output
    zd=[xd;yd;hd;wd;vd];
end