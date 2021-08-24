function [check]=intersection_check(F,L,R)

    if (F>7)&&((L>12)||(R>12))
        check = 'L'
    elseif (F>7)&&(R>12) &&(L>12)
        check = '+'
    else
        check = 'F'
        
    end 


end 