%%  Created by Luis Alejandro (alejand@umich.edu)
%   Approximates mutual infomation using histograms (counts) to estimate
%   the density functions px, py and joint pxy
function [ info ] = mutual(X,y,bins)
    d = size(X,2);
    c = numel(unique(y));
    info = zeros(d,1);
    
    % aprox py using histogram
    ny = histcounts(y,c); 
    py = ny/sum(ny);
        
    for i = 1:d
        % aprox px using histogram
        nx = histcounts(X(:,i),bins); 
        px = nx/sum(nx);
        % aprox joint probability pxy using histogram
        nxy = histcounts2(X(:,i),y,[bins c]); 
        pxy = nxy/sum(nxy(:));
        % build all possible combinations px*py for mutual info computation
        pxpy = px'*py;
        % find non-zero elements
        j = find(pxpy ~= 0 & pxy ~= 0);
        % computes mutual info
        info(i) = sum(pxy(j).*log2(pxy(j)./pxpy(j))); 
    end
end