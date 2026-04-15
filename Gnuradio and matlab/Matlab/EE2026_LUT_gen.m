% Parameters
N = 1024;               % FFT Size

% The theoretical max magnitude squared after a 1024-point FFT
%%MaxMagSq = (InputMax * N)^2; %%unscaled 22bit
InputMax = 2048; % Full 12-bit signed peak
%scaled output
MaxMagSq = (InputMax)^2; % No "* N" because the core scaled it down!

% LUT Settings
LutEntries = 2048;      % Number of rows in your .mem file
DbRange = 60;           % We will map the top 60dB of signal
OutputWidth = 8;        % 8-bit output for display (0-255)

% Generate the table
lut = zeros(LutEntries, 1);
for i = 0:LutEntries-1
    % Use the index as a proxy for Magnitude Squared
    % Scaling index to represent the full range of MagSq
    currentMagSq = (i / (LutEntries-1)) * MaxMagSq;
    
    if currentMagSq > 0
        % Calculate dB relative to full scale (0dB is Max)
        db = 10 * log10(currentMagSq / MaxMagSq);
    else
        db = -DbRange; % Floor for log(0)
    end
    
    % Scale dB to 0-255 range
    % Points closer to 0dB get higher values (taller bars)
    val = (db + DbRange) * ( (2^OutputWidth - 1) / DbRange );
    lut(i+1) = round(max(0, val));
end

% Write to Verilog .mem file (Hex format)
fileID = fopen('db_lut.mem', 'w');
fprintf(fileID, '// dB Look-Up Table: 1024 points, 12-bit input\n');
fprintf(fileID, '// Index: Magnitude Squared (Normalized)\n');
fprintf(fileID, '// Value: 8-bit Scaled dB\n');
for i = 1:length(lut)
    fprintf(fileID, '%02x\n', lut(i));
end
fclose(fileID);

disp('db_lut.mem generated successfully.');