% Generate the curve necessary for valve PWM

img = imread('./images/curve.png');

VALVE__MAX_THRUST=0.5; % [N]

% Flip the image upside down before showing it
imagesc([0 127], [0 VALVE__MAX_THRUST], flipud(img));
 
% NOTE: if your image is RGB, you should use flipdim(img, 1) instead of flipud.
 
% set the y-axis back to normal.
set(gca,'ydir','normal');

[x,y] = ginput;

y(1)=0;
y(end)=VALVE__MAX_THRUST;

x(end)=127;

b=x(end);
a=x(1);
x=(x-a)./(b-a)*b; % Rescale the PWM values to a 0-127 scale (7 bits)

% Now display the C code to copy-paste into GNC software

fprintf('# define VALVE_CHARAC_RESOLUTION %d\n',length(x));
fprintf('unsigned int PWM_valve_charac[VALVE_CHARAC_RESOLUTION] = {');

for ii=1:length(x)
    if ii==1
        fprintf('%u',round(x(ii)));
    else
        fprintf(',%u',round(x(ii)));
    end
end

fprintf('};\n');

fprintf('double R_valve_charac[VALVE_CHARAC_RESOLUTION] = {');

for ii=1:length(y)
    if ii==1
        fprintf('%.4f',y(ii));
    else
        fprintf(',%.4f',y(ii));
    end
end

fprintf('};\n');
