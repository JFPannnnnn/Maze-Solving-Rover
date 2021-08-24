function[u_avg] = read_five_times(s_cmd,s_rply,num)
u = [0,0,0,0,0];
for i = 1:5
    cmdstring = [strcat('u',num2str(num)),newline];
    u(i) = tcpclient_write(cmdstring, s_cmd, s_rply);
end
u_avg= average(u);
end
    