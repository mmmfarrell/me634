function [ out ] = twelve_to_thirteen( x )

    out(1)    = x(1);
    out(2)    = x(2);
    out(3)    = x(3);
    out(4)     = x(4);
    out(5)     = x(5);
    out(6)     = x(6);
    r    = x(7);
    p    = x(8);
    y    = x(9);

    quat = euler_to_quat([r,p,y]);
    out(7) = quat(1);
    out(8) = quat(2);
    out(9) = quat(3);
    out(10) = quat(4);
    
    out(11)     = x(10);
    out(12)     = x(11);
    out(13)     = x(12);


end

