classdef AirSimWeather < uint32
    % AIRSIMWEATHER Types of weather.
    %
    % Description:
    %   Enumerator for types of weather.
    
    enumeration
        Rain        (0)
        Roadwetness (1)
        Snow        (2)
        RoadSnow    (3)
        MapleLeaf   (4)
        RoadLeaf    (5)
        Dust        (6)
        Fog         (7)
        Enabled     (8)
    end

end

