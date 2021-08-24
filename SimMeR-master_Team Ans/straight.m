function[a] = straight(u,u_past,tol)

error = abs(u - u_past);

if (average(error) <= tol)
    a = 1;

else
    a =0;
end
end
