% ********************************************************************
% Software License Agreement (BSD License)
%
% Copyright (c) 2016, Johns Hopkins University
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
%
% * Redistributions of source code must retain the above copyright
% notice, this list of conditions and the following disclaimer.
% * Redistributions in binary form must reproduce the above
% copyright notice, this list of conditions and the following
% disclaimer in the documentation and/or other materials provided
% with the distribution.
% * Neither the name of the Johns Hopkins University nor the names of its
% contributors may be used to endorse or promote products derived
% from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
% *********************************************************************/
% 
%
% - Author: Qianli Ma, qianli.ma622@gmail.com, Johns Hopkins University
%
% Dependencies:
%              rvctools/r 

function collision = IsEdgeInCollisionGJK(ObjPRM, P1, P2)

collision = 0;
n_step  = floor(ObjPRM.dt*sqrt((P1(1) - P2(1))*(P1(1) - P2(1)) + ...
                               (P1(2) - P2(2))*(P1(2) - P2(2))));

if n_step == 0
    return;
end

% Note: length(properties(ObjPRM.Robot)) should be equal to length(P1)
% C-space steps
c_sp_step = zeros(length(properties(ObjPRM.GetRobot)), n_step);

for i = 1:length(P1)
    c_sp_step(i,:) = linspace(P1(i), P2(i), n_step);
end

% Check if the C-space line hits the obstacle
Obs = ObjPRM.GetObs;
for i = 1:length(Obs)
    % Convert to body frame of the obstacle
    t_test = rot2(Obs(i).ang)'*([c_sp_step(1,:); c_sp_step(2,:)] ...
                              - [Obs(i).tx; Obs(i).ty]);
    for j = 1:n_step
        if  abs(t_test(1,j)/Obs(i).ra)^(2/Obs(i).eps) + ...
            abs(t_test(2,j)/Obs(i).rb)^(2/Obs(i).eps) <= 1 
            collision = 1;
            return
        end
    end
end

% Check if the rabbit hits the obstacle or arena, step by step
Arena = ObjPRM.GetArena;
for i = 1:n_step
    
    in_collision = ObjPRM.GetRobot.IsSampleInCollision(c_sp_step(:,i),...
                                                     Arena, Obs,'e-s-GJK');
    if in_collision
        return;
    end
end
end




