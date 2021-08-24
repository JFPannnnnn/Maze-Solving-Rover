function[] = pls_go_straight(s_cmd,s_rply)
u = 0;  % Ultrasonic measurements
u_past = u;
count =0;
error = 1;
incre = 5;
while error
    if (count == 0)
        u1 = read_five_times(s_cmd,s_rply)
        cmdstring = [strcat('r1-',num2str(1)) newline];    
        reply = tcpclient_write(cmdstring, s_cmd, s_rply)
        u2  = read_five_times(s_cmd,s_rply)
        u = u2;
        if straight(u1,u2)
            error = 0;
        else
            if u2>u1
                incre = -incre;
                inv = 1;
            end
                
        end
        cmdstring = [strcat('r1-',num2str(-1)) newline];    
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
      
    else
            
            u = read_five_times(s_cmd,s_rply)
            if ~straight(u(1),u_past(1))
%                 if u<=u_past
%                     if inv
%                     incre = incre + 0.2;
%                     else
%                         incre = incre - 0.2;
%                     end
%                 end
                  cmdstring = [strcat('r1-',num2str(incre)) newline];    
                  reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            else
            u1 = read_five_times(s_cmd,s_rply)
            cmdstring = [strcat('r1-',num2str(-1)) newline];    
            reply = tcpclient_write(cmdstring, s_cmd, s_rply)
            u2  = read_five_times(s_cmd,s_rply)
            if straight(u1,u2)
                error = 0;
            end
            cmdstring = [strcat('r1-',num2str(2)) newline];    
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            
        end
            
    end
    u_past = u
    count = count+1

end