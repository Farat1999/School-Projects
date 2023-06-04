% Load the "train" signal
load train;

% Here the Vp and N can be changed;
Vp = 1;
N = 2;

% Call function "MyDAconverter" 
[quantizedSignal,varLin,varSat,SNqR] = MyQuantizer(y,Vp,N);

% Convert the quantized signal to gray coded bitstreams 
[bitStream] = MyGraycode(quantizedSignal,Vp,N);

% Convert the bitsream back to quantified signal 
[estimatedSignal] = MyDAconverter(bitStream,Vp,N);

%% Task 1-6

% Task 1
figure
subplot(2,1,1);
plot(y)
xlabel = ("Number of signals");
ylabel = ("Amplitude (V)");
title('Unquantized signal')
subplot(2,1,2); 
plot(quantizedSignal)
xlabel = ("Number of signals");
ylabel = ("Amplitude (V)");
title('Quantized signal')

%%
% Question 4a
lin_var = [];
for i = 1:6
    [var_sat, var_lin,quantizedSignal, SNqR] = MyQuantizer(y,i,4);
    lin_var = [lin_var, var_lin];
    
end

%Task 4a
lin_var_es = [];
lin_var_th = [];
x = [];
for i = 1:6
    L = 2.^4; 
    delta =(i-(-i))/L; 
    [var_sat, var_lin,quantizedSignal, SNqR] = MyQuantizer(y,i,4);
    lin_var_es = [lin_var_es, var_lin];
    lin_var_th = [lin_var_th, (delta.^2)./12];
     x =[x, i];
    plot(x,lin_var_th)
    title('Linear variance for different Vp-values')
    
    hold on
    
    plot(x,lin_var_es)
    
    hold off
    
end

%%
% Question 5a
SNqR_= [];
for i = 1:20
    [var_sat, var_lin,~, SNqR] = MyQuantizer(y,1,i);
    SNqR_ = [SNqR_, SNqR];
end
%%
% Question 6
SNqR_values = [];
for i = 1:10
    [~, ~ ,~, SNqR] = MyQuantizer(y,1,i);
    SNqR_values = [SNqR_values, SNqR];
    %sound(quantizedSignal)
end

SNqR_th = [];
SNqR_es = [];
x = [];
for i = 1:10
    L = 2.^i;
    [~, ~ ,~, SNqR] = MyQuantizer(y,1,i);
    SNqR_es = [SNqR_es, SNqR];
    SNqR_th = [SNqR_th, 20*log10(L.^2)];
    x =[x, i];
    
    plot(x,SNqR_th)
    title('SNqR for different N-values')
    
    hold on
    
    plot(x,SNqR_es)
    
    hold off
end

