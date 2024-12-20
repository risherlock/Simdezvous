function [dq] = qerr(qd, qf)
% Error quaternion
%
% Inputs:
%   qf: Feedback quaternion
%   qd: Desired quaternion
%
% Output:
%   dq: Rotation from qf to qd

  dq = qprod(qinv(qd), qf);
end
