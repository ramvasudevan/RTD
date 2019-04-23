function J_out = eval_J(k,Jcoeffs,Jpowers,N)
% J_out = eval_J(k,Jcoeffs,Jpowers,N)
%
% Given a Jacobian matrix represented as a coefficients matrix and a powers
% matrix (see diff_wk_wrt_k), evaluate the jacobian at k (Nk-by-1)

    Nk = length(k);
    
    J_out = zeros(N,Nk) ;
 
    for idx = 1:Nk
        J_out(:,idx) = Jcoeffs(1+N*(idx-1):N*idx,:) * ...
            (prod(repmat(k',size(Jpowers,1),1).^Jpowers(:,(idx-1)*Nk+1:Nk*idx),2)) ;
    end
end