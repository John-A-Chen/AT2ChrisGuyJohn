list = serialportlist;
for i = 1:size(list,2)
    try
        device = serialport(list(i),9600);
    catch
        fprintf('%d does not work\n',i);
    end
end