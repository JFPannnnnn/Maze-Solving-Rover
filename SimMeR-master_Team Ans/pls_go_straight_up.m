function[] = pls_go_straight_up(s_cmd,s_rply)
u = 0;  % Ultrasonic measurements
u_past = u;
count =0;
error = 1;
incre = 10;
tol = 0.1;
num = 1;
angle = 0;
ua = read_five_times(s_cmd,s_rply,1);
ub = read_five_times(s_cmd,s_rply,2);
uc = read_five_times(s_cmd,s_rply,3);
ud = read_five_times(s_cmd,s_rply,4);
ue = read_five_times(s_cmd,s_rply,5);

all = [ua ub uc ud ue];
[val,num] = min(all);

while all(num) <= 1.7
    all(num) = 10000;
    [val,num] = min(all);
end

while error
    if (count == 0)
        u1 = read_five_times(s_cmd,s_rply,num)
        cmdstring = [strcat('r1-',num2str(5)) newline];    
        reply = tcpclient_write(cmdstring, s_cmd, s_rply)
        u2  = read_five_times(s_cmd,s_rply,num)
        u = u2;
        if straight(u1,u2,tol)
            error = 0;
        else
            if u2>u1
                incre = -incre;
                inv = 1;
            end
               
        end
        cmdstring = [strcat('r1-',num2str(-5)) newline];    
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
     
    else
            cmdstring = [strcat('u',num2str(num)),newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
%             u = read_five_times(s_cmd,s_rply,num)
            if ~straight(u(1),u_past(1),tol)
%                 if u<=u_past
%                     if inv
%                     incre = incre + 0.2;
%                     else
%                         incre = incre - 0.2;
%                     end
%                 end
if (angle<=720 &&angle>0) || (angle<=0 && (angle>=-720))
    cmdstring = [strcat('r1-',num2str(incre)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    angle = angle+incre;
else
    um = read_five_times(s_cmd,s_rply,1);
    un = read_five_times(s_cmd,s_rply,3);
    if um >= un
        cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    else
        cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    angle = 0;
end
            else
                u1 = read_five_times(s_cmd,s_rply,num)
                cmdstring = [strcat('r1-',num2str(-1)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply)
                u2  = read_five_times(s_cmd,s_rply,num)
                if straight(u1,u2,tol)
                    error = 0;
                end
                cmdstring = [strcat('r1-',num2str(2)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
               
            end
           
    end
    u_past = u
    count = count+1

end