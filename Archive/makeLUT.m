% temp = [...
%     42*ones(1,40) ...   %37 degrees for 35 min
%     30*ones(1,35)    %30 degrees for 25 min
%     ];
% Convert to uint8
%temp = uint8(temp);

t = linspace(-pi/2,pi/2,30);
temp = [ones(1,110)*30, sin(t)*(38-30)/2+34 , ones(1,40)*38];

% Format the string
formatted_string = 'const PROGMEM double lookupTable[] = {\n';
disp('const PROGMEM double lookupTable[] = {')
for i = 1:10:length(temp)
    chunk = temp(i:min(i+9, end));
    line = sprintf('    %.2f', chunk(1));
    for j = 2:length(chunk)
        line = [line, sprintf(', %.2f', chunk(j))];
    end
    if i+9 < length(temp)
        line = [line, ','];
    end
    disp(line)
    formatted_string = [formatted_string, line, '\n'];
end
disp('};')
formatted_string = [formatted_string, '};'];

% Display the result
%disp(formatted_string)