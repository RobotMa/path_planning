function NewPntInCollision = IsSampleInCollisionGJK(Robot, Pnt, Arena, Obs)

iteration = 10;
Robot.MoveRobot(Pnt(1), Pnt(2), Pnt(3), Pnt(4), Pnt(5));

% internal collision checking (containment checking)
for j = 1:length(Arena)
    % 0: True  - contained, no collision 
    % 1: False - not contained, in collision
    containmentStatus = Robot.IsContainedES(Arena(j));
    if containmentStatus
        NewPntInCollision = 1;
        return;
    end
end

[f, eL, eR] = Robot.GetLink;
shape_f  = f.GetPoints;
shape_e1 = eL.GetPoints;
shape_e2 = eR.GetPoints;

% change data type accoriding to GJK2D function
f_obj.Vertices  = shape_f';
e1_obj.Vertices = shape_e1';
e2_obj.Vertices = shape_e2';

% external collision checking
for j = 1:length(Obs)    
    shape_oj = Obs(j).GetPoints;
    oj_obj.Vertices = shape_oj';
    NewPntInCollision = GJK2D(oj_obj, f_obj,  iteration) || ...
                        GJK2D(oj_obj, e1_obj, iteration) ||...
                        GJK2D(oj_obj, e2_obj, iteration);
    if NewPntInCollision
        return;
    end
end

end