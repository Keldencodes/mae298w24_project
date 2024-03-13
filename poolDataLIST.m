function yout = poolDataLIST(yin,ahat,nVars,polyorder, usesine)
n = size(yin,1);

ind = 1;
% poly order 0
yout{ind,1} = '1';
ind = ind+1;

% poly order 1
for i=1:nVars
    yout(ind,1) = yin(i);
    ind = ind+1;
end

if(polyorder>=2)
    % poly order 2
    for i=1:nVars
        for j=i:nVars
            yout{ind,1} = [yin{i},yin{j}];
            ind = ind+1;
        end
    end
end

if(polyorder>=3)
    % poly order 3
    for i=1:nVars
        for j=i:nVars
            for k=j:nVars
                yout{ind,1} = [yin{i},yin{j},yin{k}];
                ind = ind+1;
            end
        end
    end
end
numsin = 1; % number of sines
if(usesine)
    for k=1:numsin
        for j=1:nVars
            yout{ind,1} = ['sin(',num2str(k),num2str(yin{j}),')'];
            ind = ind + 1;
            yout{ind,1} = ['cos(',num2str(k),num2str(yin{j}),')'];
            ind = ind + 1;
        end
    end
end

output = yout;
newout(1) = {''};
for k=1:length(yin)
    newout{1,1+k} = [yin{k},'dot'];
end
newout = {'','xdot','ydot','zdot'};
for k=1:size(ahat,1)
    newout(k+1,1) = output(k);
    for j=1:length(yin)
        newout{k+1,1+j} = ahat(k,j);
    end
end
newout