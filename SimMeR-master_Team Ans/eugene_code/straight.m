function[a] = straight(u,u_past)

error1 = abs(u(2)/1.08 - u_past(2));
error2 = abs(u(4)/1.08 - u_past(4));
error3 = abs(u(5)/1.08 - u_past(5));
tol = 0.5;
if (average(error1) <= tol)
    a = 1;

elseif (average(error2) <= tol)||(average(error3) <= tol)
    a = 2;
else
    a =0;
end
end
