function colors = customColorMap(n)
    colors = zeros(n, 3, 'double');

    if n == 1
        colors = [1 1 1];
        return
    end

    % Red Cyan Green Yellow Purple Blue ... then lines() colormap
    definedColors = [ 1 0 0 ; 0.25 1 1 ; 0 1 0 ; 1 1 0 ; 1 0 1 ;  0 1 0 ];
    nDefinedColors = size(definedColors, 1);
    if n <= nDefinedColors
        colors = definedColors(1:n, :);
        return
    end
    colors(1 : nDefinedColors, :) = definedColors;
    colors(nDefinedColors + 1 : n, :) = lines(n-nDefinedColors);
end
