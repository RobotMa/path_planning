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
% References::
%
% - Probabilistic roadmaps for path planning in high dimensional
%   configuration spaces,
%   L. Kavraki, P. Svestka, J. Latombe, and M. Overmars,
%   IEEE Transactions on Robotics and Automation, vol. 12, pp. 566-580,
%   Aug 1996.
%
% - Author: Qianli Ma, qianli.ma622@gmail.com, Johns Hopkins University
%
% Dependencies:
%              GJK_collision_detection/matlab
%              ellipsoidal_containment/matlab
%
% TODO:
% - To be extended to the 3D case
% - 
% -
classdef PRM < handle
    
    properties
        n_v          % No. of total vertices (collision-free)
        n_neighbors  % No. of neighbors to connect to for each vertex
        collision_checker % choose among different collision checkers
        dist_cutoff  % cutoff distance when adding neighboring vertex
        dt           % Step length
        
        % Frames of the robot "moving" along the planned path
        movie_frame = struct('cdata',{},'colormap',{})  
    end
    
    properties (Access = protected)
        graph_prm    % Customized graph which contains an array of vertices
                     % and an adjacency matrix
                      % Note: there is a built-in graph data structure in
                     % Matlab and this is NOT that
        Robot        % Robot Object of class RabbitRobot2D
        Arena        % Arena Obj. of class SuperEllipse
        Obs          % Obstacle Obj. of class SuperEllipse
        paths        % Planned path(s)
        costs        % Cost of the planned path(s)  
    end
    
    methods
        
        %% ---------------------- CONSTRUCTOR --------------------------- %
        % --------------------------------------------------------------- %
        function Obj = PRM(Robot, Arena, Obs, n_v, n_neighbors, ...
                           dist_cutoff, dt, collision_checker)
            
            Obj.Robot = Robot;
            Obj.Arena = Arena;
            Obj.Obs   = Obs;
            
            Obj.n_v         = n_v;
            Obj.n_neighbors = n_neighbors;
            Obj.dist_cutoff = dist_cutoff;
            Obj.dt          = dt;
            Obj.collision_checker = collision_checker;
            
        end
        
        % --------------------------------------------------------------- %
        function Robot = GetRobot(Obj)
            % PRM.GetRobot:
            Robot = Obj.Robot;
        end
        
        % --------------------------------------------------------------- %
        function Arena = GetArena(Obj)
            % PRM.GetArena:
            Arena = Obj.Arena;
        end
        
        % --------------------------------------------------------------- %        
        function Obs = GetObs(Obj)
            % PRM.GetObs:
            Obs = Obj.Obs;
        end
        
        % --------------------------------------------------------------- %        
        function paths = GetPaths(Obj)
            %GetPaths:
            %
            paths = Obj.paths;
        end
        
        %% ----------------- CORE MEMBER FUNCTIONS ---------------------- %
        % ----------------------------------------------------------------%
        function Plan(Obj, start, goal)
            %PRM.PLAN:
            % generate the graph given the start and goal
            
            dim_cspace = length(properties(Obj.Robot));
            Obj.graph_prm.vtx = zeros(dim_cspace, Obj.n_v);
            
            % Use sparse matrix to represent the adjacency matrix
            Obj.graph_prm.adjMat = sparse(Obj.n_v, Obj.n_v, 0);
            
            % Step 01: add Start and Goal to the vertex set
            Obj.graph_prm.vtx(:,       1) = start;
            Obj.graph_prm.vtx(:, Obj.n_v) = goal;
            
            %
            % Collision Checking (CK)
            %   e-e-char: ellipse-ellipse CK based on the characteristic polynomial
            %    e-s-GJK: ellipse-superquadrics CK based on GJK
            % e-s-interp: ellipse-superquadrics CK based on interpolation on ellipse
            %      e-s-M: ellipse-superquadrics CK based on Minkowski sum
            % collisionChecker = 'e-s-GJK';
            
            % --- Step 1: find a total of N_v-2 collision-free samples --%
            for i = 2:Obj.n_v-1
                NewPntInCollision = 1;
                while NewPntInCollision
                    % Create a random sample within the arena
                    new_pnt = Obj.SampleNewPointInArena;
                    NewPntInCollision = Obj.Robot.IsSampleInCollision( ...
                                          new_pnt, Obj.Arena, Obj.Obs, ...
                                          Obj.collision_checker);
                end
                Obj.graph_prm.vtx(:,i) = new_pnt;
                
                %------
                P1 = Obj.graph_prm.vtx(:,i);
                [dist_sorted, inx] = Obj.FindSortedNeighbors(i, P1);
                
                % Connected the newly added vertex to nearest vertices
                count_v = 2; count_edge = 0;
                while count_v <= length(inx) && count_edge <= Obj.n_neighbors ...
                        && dist_sorted(count_v) <= Obj.dist_cutoff
                    j = inx(count_v);
                    P2 = Obj.graph_prm.vtx(:,j);
                    if ~Obj.IsEdgeInCollision(P1, P2)
                        Obj.graph_prm.adjMat(i,j) = 1;
                        Obj.graph_prm.adjMat(j,i) = 1;
                        count_edge = count_edge + 1;
                    end
                    count_v = count_v + 1;
                end
                %-------
                
            end
            
            % --- Step 2: connect the start and goal to the map --- %
            [~, Inx] = Obj.FindSortedNeighbors(Obj.n_v, start);
            for count_v = 2:Obj.n_v
                j = Inx(count_v);
                P2 = Obj.graph_prm.vtx(:,j);
                if ~Obj.IsEdgeInCollision(start, P2)
                    Obj.graph_prm.adjMat(1,j) = 1;
                    Obj.graph_prm.adjMat(j,1) = 1;
                    break;
                end
            end
            
            [~, Inx] = Obj.FindSortedNeighbors(Obj.n_v, goal);
            for count_v = 2:Obj.n_v
                j = Inx(count_v);
                P2 = Obj.graph_prm.vtx(:,j);
                if ~Obj.IsEdgeInCollision(goal, P2)
                    Obj.graph_prm.adjMat(Obj.n_v,j) = 1;
                    Obj.graph_prm.adjMat(j,Obj.n_v) = 1;
                    break;
                end
            end
            
        end
        
        % ----------------------------------------------------------------%
        function [sorted_dists, index_array] = FindSortedNeighbors(Obj,...
                                                          index, sample_c )
            %PRM.FindSortedNeighbors:
            % Input:
            %  - sample_c: C-space sample
            %  - index   : max index for sorting
            
            % define the weights for rotations
            % TODO: To be moved to protected properties?
            w_ang_f = 10;
            w_ang_e1 = 5;
            w_ang_e2 = 5;
            
            % Calculate the distance between the newest sample_c and the
            % previous 1st to indexth samples (vertices of graph)
            node_dists = ((Obj.graph_prm.vtx(1,1:index) - sample_c(1)).*...
                          (Obj.graph_prm.vtx(1,1:index) - sample_c(1))...
                + (Obj.graph_prm.vtx(2,1:index) - sample_c(2)).*...
                  (Obj.graph_prm.vtx(2,1:index) - sample_c(2))...
                + w_ang_f*abs(Obj.graph_prm.vtx(3,1:index) - sample_c(3))...
                + w_ang_e1*abs(Obj.graph_prm.vtx(4,1:index) - sample_c(4))...
                + w_ang_e2*abs(Obj.graph_prm.vtx(5,1:index) - sample_c(5)))...
                .^(1/2);
            
            % Sort the dists
            [sorted_dists, index_array] = sort( node_dists );
            
        end
        
        % ----------------------------------------------------------------%
        function [costs, paths] = FindPath(Obj)
            %FindPath:
            % Find the shortest path using Dijkastra algorithm
            %
            fprintf('Connecting paths \n');
            [costs, paths] = dijkstra(Obj.graph_prm.adjMat, ...
                                      Obj.graph_prm.vtx', 1, Obj.n_v);
            Obj.paths = paths;
            Obj.costs = costs;
        end
        
        
        
        %% ---------------- Edge Collision API -------------------------  %        
        % ----------------------------------------------------------------%
        function in_collision = IsEdgeInCollision(Obj, P1, P2)
            %PRM.IsEdgeInCollision:
            % Perform sample points collision checking based on the 
            % selection of collision checkers
            %
            % TODO:
            % - 'e-s-M': Minkowski based collision checking to be
            % implemented
            
            switch Obj.collision_checker
                case 'e-e-char'
                    in_collision = IsEdgeInCollisionEECP(Obj, P1, P2);
                case 'e-s-GJK'
                    in_collision = IsEdgeInCollisionGJK(Obj, P1, P2);
                case 'e-s-interp'
                    in_collision = IsEdgeInCollisionInterp(Obj, P1, P2);
                case 'e-s-M'
                    disp('e-s-M collision checker to be implemented')
            end
            
        end
        
        %% ------------------- SAMPLING METHOD -------------------------  %
        % ----------------------------------------------------------------%
        function new_pnt = SampleNewPointInArena(Obj)
            %PRM.SampleNewPointInArena:
            % Generate a C-space sample where the body frame origin of the
            % robot is contained in the arena, and the joint angles
            % constrained within the joint limits
            %
            % 
            % TODO:
            % - Current writeup is for the 5 dofs rabbit robot only.
            
            collision = 1;
            
            while collision
                % Step #1
                % Use the boundary of 1st arena to generate sample points
                % with translations (tx_rnd, ty_rnd) only
                ra1 = Obj.Arena(1).ra;
                rb1 = Obj.Arena(1).rb;
                tx_rnd = Obj.RandomSampleInRange(-ra1, ra1);
                ty_rnd = Obj.RandomSampleInRange(-rb1, rb1);
                
                new_pnt = [tx_rnd, ty_rnd];
                
                % Check through all arenas  whether is sample is contained
                for i = 1:length(Obj.Arena)
                    if abs(new_pnt(1)/Obj.Arena(i).ra)^(2/Obj.Arena(i).eps)...
                            + ...
                       abs(new_pnt(2)/Obj.Arena(i).rb)^(2/Obj.Arena(i).eps)...
                            <= 1
                        
                        collision = 0;
                        
                        % Step #2
                        % Generate the C-space sample for rotations
                        
                        [the_r, the1_r, the2_r] = ...
                            Obj.Robot.GetCSpaceRange;
                        
                        the_rnd = ...
                             Obj.RandomSampleInRange(the_r(1),  the_r(2));
                        the1_rnd = ... 
                             Obj.RandomSampleInRange(the1_r(1), the1_r(2));
                        the2_rnd = ...
                             Obj.RandomSampleInRange(the2_r(1), the2_r(2));
                        
                        % Combine with the sample of translations
                        new_pnt(3:5) = [the_rnd, the1_rnd, the2_rnd];
                    else
                        collision = 1;
                        break;
                    end
                end
            end
            
        end
        
        % ----------------------------------------------------------------%
        function sample = RandomSampleInRange(Obj, a, b)
            %RandomSampleInRange:
            % Sample a point in range [a, b] from an uniform distribution
            %
            % TODO: Different sampling methods are known to affect the
            % performance of sampling based methods. All the rest sampling
            % methods are to be implemented.
            %
            %
            sample = (b - a)*rand + a;
        end

        %% -------------------- PATH SMOOTHING -------------------------  %      
        % ----------------------------------------------------------------%
        function SmoothPath(Obj)
            %SmoothPath:
            % TODO:
        end
            
            
            
        %% --------------------- VISUALIZATION -------------------------  %      
        % ----------------------------------------------------------------%
        function PlotMap(Obj)
            %PlotMap:
            % Plot the arena(s) and obstacle(s)
            %
                        
            n_s = length(Obj.Arena);
            n_o = length(Obj.Obs);
            
            for i = 1:n_s
                Obj.Arena(i).PlotShape;
                % axis equal
            end
            
            for i = 1:n_o
                Obj.Obs(i).PlotShape;
                
                % Label the No. of obstacles
                is = num2str(i);
                box on;
                text(Obj.Obs(i).tx, Obj.Obs(i).ty, is, 'Color', [1 1 1]);
                % axis equal
            end
            axis equal
        end
        
        % ----------------------------------------------------------------%
        function PlotSamples(Obj)
            %PlotSamples
            % Plot all of the collision-free samples
            % 
                        
            [r, c, ~] = find(Obj.graph_prm.adjMat);
            for i = 1:length(r)
                plot([Obj.graph_prm.vtx(1,r(i)) Obj.graph_prm.vtx(1, c(i))],...
                     [Obj.graph_prm.vtx(2,r(i)) Obj.graph_prm.vtx(2, c(i))], '-r'); 
                hold on
            end
            
            plot(Obj.graph_prm.vtx(1,:), Obj.graph_prm.vtx(2,:),'xr'); 
            hold on
            gplot(Obj.graph_prm.adjMat,  Obj.graph_prm.vtx(1:2,:)','m')
            axis equal
        end
 
        % ----------------------------------------------------------------%
        function PlotRobotCurrentPose(Obj, index)
            %PlotRobotCurrentPose:
            % Plot robot at the current (index)pose of the planned path
            %
            
            path_copy = Obj.GetPaths;            

            pnt = Obj.graph_prm.vtx(:, path_copy(index));
            Obj.GetRobot.MoveRobot(pnt(1), pnt(2), pnt(3),...
                                   pnt(4), pnt(5));
            Obj.GetRobot.PlotRobot;
            
        end
            
        % ----------------------------------------------------------------%
        function PlotPath(Obj, make_movie)
            %PlotPath:
            % Plot the returned paths (if any)
            % Input:
            % - make_movie: take a screen shot if this is true
            figure('pos',[10 10 1200 800])

            Obj.PlotMap;
            hold on;
            
            Obj.PlotSamples;
            hold on;
            
            if isempty(Obj.GetPaths)
                fprintf('Path is still empty. Please plan first. \n')
                return
            end
            
            if isnan(Obj.GetPaths)
                fprintf('No valid path exist. \n')
                return
            end
            
            if nargin == 1
                make_movie = false;
            end
            
            % Plot the connected path
            plot(Obj.graph_prm.vtx(1, Obj.GetPaths), ...
                 Obj.graph_prm.vtx(2, Obj.GetPaths),'g','Linewidth',2);
            hold on;
                 
            % get rid of the white space around the figure
            ax = gca;
            outerpos = ax.OuterPosition;
            ti = [0.1 0.1 0.1 0.1];
            left      = outerpos(1) + ti(1);
            bottom    = outerpos(2) + ti(2);
            ax_width  = outerpos(3) - ti(1) - ti(3);
            ax_height = outerpos(4) - ti(2) - ti(4);
            ax.Position = [left bottom ax_width ax_height];
                
            % Plot the start pose
            Obj.PlotRobotCurrentPose(1);
            hold on;
            
            % Plot the goal pose
            Obj.PlotRobotCurrentPose(length(Obj.GetPaths));
            hold on;
            axis off
            
            Obj.movie_frame(1) = getframe(gcf);
            
            for i = 2:length(Obj.GetPaths)-1
                
                Obj.PlotRobotCurrentPose(i);
                axis equal
                
                % Start from the 2nd frame to avoid frame size changing
                if make_movie
                    Obj.movie_frame(i-1) = getframe(gcf);
                    fprintf('Record the %d th frame ... \n', i);
                end
                hold on;
            end
        end
        
        % ----------------------------------------------------------------%
        function MakeMovie(Obj)
            %MakeMovie:
            %
            
            if isempty(Obj.GetPaths)
                fprintf('Path is still empty. Please plan first. \n')
                return
            end
            
            if isnan(Obj.GetPaths)
                fprintf('No valid path exist. \n')
                return
            end
            
            fprintf('Making the movie into /video folder \n');
            if ~exist('video', 'dir')
               mkdir('video') 
            end
            ObjVid = VideoWriter('../video/ES_PRM_GJK_5dofs.avi');
            open(ObjVid);
            writeVideo(ObjVid, Obj.movie_frame);
            close(ObjVid);
        end
    end
end