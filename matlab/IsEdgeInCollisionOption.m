function collision = IsEdgeInCollisionOption(P1, P2, dt, Robot, ...
                                             Arena, Obs, collision_checker)
% perform sample points collision checking based on the selection of
% collision checkers

switch collision_checker
    case 'e-e-char'
        collision = IsEdgeInCollisionEECP(P1, P2, dt, Robot, Arena, Obs);
    case 'e-s-GJK'
        collision = IsEdgeInCollisionGJK(P1, P2, dt, Robot, Arena, Obs);
    case 'e-s-interp'
        collision = IsEdgeInCollisionInterp(P1, P2, dt, Robot, Arena, Obs);
    case 'e-s-M'
        disp('e-s-M collision checker to be implemented')
end

end