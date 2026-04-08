N = 1024;
w = hann(N);
w_q15 = round(w * 32767); 
%%convers this to 16 bit hex

fid = fopen('hann_lut.mem', 'wt');
for i = 1:N
    fprintf(fid, '%04x\n', w_q15(i));
end
fclose(fid);