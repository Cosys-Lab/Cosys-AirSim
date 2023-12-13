% This takes forever so just load in the colormap.csv provided

colorMap = generate_colormap();
 
function values = get_colormap_channel_values()
    values = zeros(258, 1, 'int32');
    step = 256;
    iter = 0;
    values(1) = 0;
    stop = false;
    while ~stop
        val = step - 1;
        
        while val <= 256
            iter = iter + 1;
            values(iter + 1) = int32(val);
            val = val + (step * 2);
        end
        if single(step) / 2 < 1
            stop = true;
        end
        step = int32(step / 2);
    end
end

function colormapOut = get_colormap_colors(maxVal, enable1, enable2, enable3, colormapIn, channelValues, okValues, gammaCorrectionTable)
    colormapOut = colormapIn;
    if ~enable1
        max1 = maxVal - 1;
    else
        max1 = 0;
    end
    
    if ~enable2
        max2 = maxVal - 1;
    else
        max2 = 0;
    end
    
    if ~enable3
        max3 = maxVal - 1;
    else
        max3 = 0;
    end
    
    i = 0;
    j = 0;
    k = 0;
    
    while i <= max1
        while j <= max2
            while k <= max3
                if enable1
                    r = channelValues(maxVal + 1);
                else
                    r = channelValues(i + 1);
                end
                
                if enable2
                    g = channelValues(maxVal + 1);
                else
                    g = channelValues(j + 1);
                end
                
                if enable3
                    b = channelValues(maxVal + 1);
                else
                    b = channelValues(k + 1);
                end
                
                if ismember(r, okValues) && ismember(g, okValues) && ismember(b, okValues) && r ~= 149 && g ~= 149 && b ~= 149
                    colormapOut = [colormapOut; [gammaCorrectionTable(r+1), gammaCorrectionTable(g+1), gammaCorrectionTable(b+1)]];
                end
                
                k = k + 1;
            end
            
            j = j + 1;
            k = 0;
        end
        
        i = i + 1;
        j = 0;
    end
end

function colorMap = generate_colormap()
    channelValues = get_colormap_channel_values();
    numPerChannel = 256;
    colorMap = [];
    okValues = [];
    uneven_start = 79;
    full_start = 149;

    gammaCorrectionTable = [0,...
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,...
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,...
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,...
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,...
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,...
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,...
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,...
                            0, 0, 0, 0, 0, 0, 0, 0, 79, 0,...
                            81, 0, 83, 0, 85, 0, 86, 0, 88, 0,...
                            90, 0, 93, 0, 95, 0, 96, 0, 98, 0,...
                            101, 0, 102, 0, 105, 0, 106, 0, 109, 0,...
                            110, 0, 113, 0, 114, 0, 117, 0, 119, 0,...
                            120, 0, 122, 0, 125, 0, 127, 0, 129, 0,...
                            131, 0, 133, 0, 135, 0, 137, 0, 139, 0,...
                            141, 0, 143, 0, 145, 0, 147, 147, 148, 150,...
                            151, 152, 153, 154, 155, 156, 157, 158, 159, 160,...
                            161, 162, 163, 164, 165, 166, 167, 168, 169, 170,...
                            171, 172, 173, 174, 175, 176, 177, 178, 179, 180,...
                            181, 182, 183, 184, 185, 186, 187, 188, 189, 190,...
                            191, 192, 193, 194, 195, 196, 197, 198, 199, 200,...
                            201, 202, 203, 204, 205, 206, 207, 208, 209, 210,...
                            211, 212, 213, 214, 215, 216, 217, 218, 219, 220,...
                            221, 222, 223, 224, 225, 226, 227, 228, 229, 230,...
                            231, 232, 233, 234, 235, 236, 237, 238, 239, 240,...
                            241, 242, 243, 244, 245, 246, 247, 248, 249, 250,...
                            251, 252, 253, 254, 255];
    
    okValues = [uneven_start:2:full_start, full_start + 1:numPerChannel - 1];
    
    for maxChannelIndex = 0:numPerChannel-1
        colorMap = get_colormap_colors(maxChannelIndex, false, false, true, colorMap, channelValues, okValues, gammaCorrectionTable);
        colorMap = get_colormap_colors(maxChannelIndex, false, true, false, colorMap, channelValues, okValues, gammaCorrectionTable);
        colorMap = get_colormap_colors(maxChannelIndex, false, true, true, colorMap, channelValues, okValues, gammaCorrectionTable);
        colorMap = get_colormap_colors(maxChannelIndex, true, false, false, colorMap, channelValues, okValues, gammaCorrectionTable);
        colorMap = get_colormap_colors(maxChannelIndex, true, false, true, colorMap, channelValues, okValues, gammaCorrectionTable);
        colorMap = get_colormap_colors(maxChannelIndex, true, true, false, colorMap, channelValues, okValues, gammaCorrectionTable);
        colorMap = get_colormap_colors(maxChannelIndex, true, true, true, colorMap, channelValues, okValues, gammaCorrectionTable);
    end
    
    colorMap = int32(colorMap);
end