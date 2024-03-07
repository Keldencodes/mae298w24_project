function [q] = DCM_to_quat_v2(A)
%Input: direction cosigne matrix 
%Ouput: quaternion, scalar last
%NOTE: this only works when the trace is nonzero
%Fixed, July 30 2020 for zero (or negative) trace
tr = 1+A(1,1)+A(2,2)+A(3,3);  %I think this is wrong? not +1 at beginning

if(tr > 0)
%     qw = 0.5*sqrt(1+A(1,1)+A(2,2)+A(3,3));
%     qx = (A(2,3)-A(3,2))/(4*qw);  %TODO: check that indexing isn't backwards
%     qy = (A(3,1)-A(1,3))/(4*qw);
%     qz = (A(1,2)-A(2,1))/(4*qw);    
    S = 2*sqrt(tr+1);
    qw = 0.25*S;
    qx = (A(2,3)-A(3,2))/S;
    qy = (A(3,1)-A(1,3))/S;
    qz = (A(1,2)-A(2,1))/S;
elseif A(1,1) > A(2,2) && A(1,1) > A(3,3)   %first term is largest
    S = 2*sqrt(1+A(1,1)-A(2,2)-A(3,3));
    qw = (A(2,3)-A(3,2))/S;
    qx = 0.25 * S;
    qy = (A(2,1)+A(1,2))/S;
    qz = (A(3,1)+A(1,3))/S;
elseif A(2,2) > A(3,3)                      %second term is largest
    S = 2*sqrt(1+A(2,2)-A(1,1)-A(3,3));
    qw = (A(3,1)-A(1,3))/S;
    qx = (A(2,1)+A(1,2))/S;
    qy = 0.25*S;
    qz = (A(3,2)+A(2,3))/S;
else                                        %third term is the larget
    S = 2*sqrt(1+A(3,3)-A(1,1)-A(2,2));
    qw = (A(1,2)-A(2,1))/S;
    qx = (A(3,1)+A(1,3))/S;
    qy = (A(3,2)+A(2,3))/S;
    qz = 0.25*S;
end
q = [qx; qy; qz; qw];
q = q/norm(q);

end

