function Collision_NewPnt = IsSampleInCollisionOption(new_pnt, collision_checker)
% perform sample points collision checking based on the selection of
% collision checkers

switch collision_checker
    case 'e-e-char'
        Collision_NewPnt = IsSampleInCollisionNew(new_pnt);
    case 'e-s-GJK'
        Collision_NewPnt = IsSampleInCollisionES_GJK(new_pnt);
    case 'e-s-interp'
        Collision_NewPnt = IsSampleInCollision_Interp(new_pnt);
    case 'e-s-M'
        disp('e-s-M collision checker to be implemented')
end

end