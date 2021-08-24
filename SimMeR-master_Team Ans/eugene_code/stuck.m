function[a] = stuck(u,u_past)

randpick = randperm(5,4);
randpick2 = randperm(5,4);
error = abs(u(randpick)/1.08 - u_past(randpick));
error2 = abs(u(randpick2) - u_past(randpick2)/(1.08));

if (average(error) <= 2)||(average(error2) <= 2)
    a=1;
    
else
    a=0;
end



end