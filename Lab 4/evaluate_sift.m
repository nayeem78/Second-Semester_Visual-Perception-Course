function loc = evaluate_sift(I1, I2, H, is_one_scale)    
    [f1, d1] = vl_sift(I1);
    [f2, d2] = vl_sift(I2);
    
    if is_one_scale
        f1(3, :) = 1;
        f2(3, :) = 1;
        [f1, d1] = vl_sift(I1, 'Octaves', 1, 'Frames', f1);
        [f2, d2] = vl_sift(I2, 'Octaves', 1, 'Frames', f2);
    end
    
    matches = vl_ubcmatch(d1, d2);
    
    loc = 0;
    location_threshold = 2;
    
    for i = 1 : size(matches, 2)
        p1 = H * [f1(1:2, matches(1, i)); 1];
        p1 = p1 / p1(end);
        p2 = [f2(1:2, matches(2, i)); 1];

        if norm(p1 - p2) < location_threshold
            loc = loc + 1;
        end
    end
 
    loc = loc / size(matches, 2);
end