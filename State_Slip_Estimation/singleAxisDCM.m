function C = singleAxisDCM(axis, angle)
%SINGLEAXISDCM  Gives DCM corresponding to rotation about given coordinate axis by given angle

    if axis == 1
        C = [1 0 0;
             0 cos(angle) sin(angle);
             0 -sin(angle) cos(angle)];
    elseif axis == 2
            C = [cos(angle) 0 -sin(angle);
                 0 1 0;
                 sin(angle) 0 cos(angle)];
    elseif axis == 3
            C = [cos(angle) sin(angle) 0;
                 -sin(angle) cos(angle) 0;
                 0 0 1];
    else 
        error('Invalid axis specified. Specify 1 for X-axis, 2 for Y-axis, and 3 for Z-axis')
    end
end