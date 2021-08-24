function[] = pls_makesameu4u5(s_cmd,s_rply)
u = [0,0,0,0,0];  % Ultrasonic measurements
rot_stuck = 90;
stepcount = 0;

a = 1;

    while a
        
        for m = 1:5
        cmdstring = [strcat('u',num2str(m)) newline];
        u(m) = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
        
        if (abs(u(4) - u(5)) <=0.2) 
            a = 0

        elseif (abs(u(4) - u(5)) > 0.2) && (abs(u(4) - u(5)) <= 7)
            %abs(u(4) - u(5))>1
            % If there is a large discrepancy between ultra 4 and 5, then
            % adjust until they produce similiar readings
            % adjust its direction
            if u(4)>u(5)
                beta = 2.2 % The spacing between u4 and u5 is ~2 inches
                cmdstring = [strcat('r1-',num2str(beta)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            else 
                beta = -2.2 % The spacing between u4 and u5 is ~2 inches
                cmdstring = [strcat('r1-',num2str(beta)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        
            end
        else 
            a = 0
    end


    end
 

    