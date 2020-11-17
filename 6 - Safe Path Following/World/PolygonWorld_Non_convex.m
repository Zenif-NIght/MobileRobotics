classdef PolygonWorld_Non_convex < PolygonWorld
    %PolygonWorld1 Is an instantiation of Polygon World with a three polygons
    
    
    methods
        function obj = PolygonWorld_Non_convex()
            % Define the vertices for various polygons
            V1 = [6, 5, 1; 1, 6, 4];
            V2 = [5, 3, 7; 2, -3, -3];
            V3 = [10, 10, 12, 12; 12, 0, 0, 12];
            
            % Create the polygon world
            obj = obj@PolygonWorld(V1, V2, V3);
        end
    end
end

