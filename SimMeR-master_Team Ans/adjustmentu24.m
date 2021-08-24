function[] = adjustmentu24(u,u_past,speed,s_cmd, s_rply)
    u2 = read_five_times(s_cmd,s_rply,2);
    u4 = read_five_times(s_cmd,s_rply,4);
    u5 = read_five_times(s_cmd,s_rply,5);
    if (u2<u4 && u2>3.5) || (u2>u4 && (u4<2)||u5<2)
       cmdstring = [strcat('r1-',num2str(20)) newline];  % Build command string to rotate bot
       reply = tcpclient_write(cmdstring, s_cmd, s_rply);
       cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
       reply = tcpclient_write(cmdstring, s_cmd, s_rply);
       cmdstring = [strcat('r1-',num2str(-13)) newline];  % Build command string to rotate bot
       reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    elseif (u2>u4 && u4>3.5) || (u2<u4 && u2<2)
       cmdstring = [strcat('r1-',num2str(-20)) newline];  % Build command string to rotate bot
       reply = tcpclient_write(cmdstring, s_cmd, s_rply);
       cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
       reply = tcpclient_write(cmdstring, s_cmd, s_rply);
       cmdstring = [strcat('r1-',num2str(13)) newline];  % Build command string to rotate bot
       reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    end

    toler = 0.2;
    u2diff = straight (u(4),u_past(2),toler);
    u4diff = straight (u(4),u_past(4),toler);
    if (u2diff==1) || (u4diff==1)
        cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    else
        if (u(2)-u_past(2))<0
          cmdstring = [strcat('r1-',num2str(-5)) newline];  % Build command string to rotate bot
          reply = tcpclient_write(cmdstring, s_cmd, s_rply);

        else
          cmdstring = [strcat('r1-',num2str(5)) newline];  % Build command string to rotate bot
          reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end  
    end

end