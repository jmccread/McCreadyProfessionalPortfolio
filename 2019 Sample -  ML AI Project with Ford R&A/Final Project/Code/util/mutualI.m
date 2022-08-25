%%  Created by Luis Alejandro (alejand@umich.edu)
%   Approximates mutual infomation using histograms (counts) to estimate
%   the density functions px, py and joint pxy
function [ info ] = mutualI(X,y,bins)
    d = size(X,2);
    c = numel(unique(y));
    info = zeros(d,1);
    for j = 1:d
        % aprox px using histogram
        nx = histcounts(X(:,j),bins); 
        px = nx/sum(nx);
        % aprox py using histogram
        ny = histcounts(y,c); 
        py = ny/sum(ny);
        % aprox joint probability pxy using histogram
        nxy = histcounts2(X(:,j),y,[bins c]); 
        pxy = nxy/sum(nxy(:));
        % build all possible combinations px*py for mutual info computation
        pxpy = px'*py;
        % find non-zero elements
        i = find(pxpy ~= 0 & pxy ~= 0);
        % computes mutual info
        info(j) = sum(pxy(i).*log2(pxy(i)./pxpy(i))); 
    end
    
    term = zeros(d,1);
    for j = 1:d
        nj = histcounts(X(:,j),bins); 
        pxj = nj/sum(nj);
        for i = 1:d
            if (i == j), continue; end
            ni = histcounts(X(:,i),bins); 
            pxi = ni/sum(ni);            
            nxixj = histcounts2(X(:,j),X(:,i),[bins bins]); 
            pxixj = nxixj/sum(nxixj(:));
            pxipxj = pxi'*pxj;
            s = find(pxipxj ~= 0 & pxixj ~= 0);
            term(j) = term(j) + (sum(pxixj(s).*log2(pxixj(s)./pxipxj(s)))); 
        end
        term(j) = term(j)/(d-1);
    end
    
    info = info - term;    
end