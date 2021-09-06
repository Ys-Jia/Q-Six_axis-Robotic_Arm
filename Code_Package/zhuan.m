function real_rot = zhuan(A)
real_rot = [0 0 0 0;
            0 0 0 0;
            0 0 0 0;
            0 0 0 1;];
    for i = 1:1:3
        for j = 1:1:3
           real_rot(i,j) = A(i,j);
        end        
    end     
end
        