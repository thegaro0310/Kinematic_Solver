classdef Colors
    %enum class for different colors
    
    properties
      R
      G
      B
   end
   methods
      function c = Colors(r, g, b)
         c.R = r; c.G = g; c.B = b;
      end
      
      function rgb=getColor(obj)
        rgb=[obj.R,obj.G,obj.B];
      end
   end
   enumeration
      NormalLink (0.3451   , 0.3451   , 0.3451)
      Inactive (0.7   , 0.7   , 0.7)
      NormalNode (0.75  , 0.6   , 0)
      Incomplete   (0.8000  ,  0.0824   , 0.3490)
      Selected (0.3098   , 0.8353   , 0.8392)
      Constrained ( 0.8000,0.0824,0.3490)
      Annotation  (0.4000 , 0 ,   0.8000)
      Point1  (0.87, 0.49, 0)
      Point2  (0.6, 0.0 ,0.8)
      StatusComplete (0.165, 0.384, 0.275)
      StatusIncomplete  (0.757, 0.188, 0)
      Button (0.702,0.78,1)
      BackGround (0.9608  ,  0.9804  ,  0.9804)
      PanelBackground (0.8706  ,  0.9059  ,  0.9373)    
      UpDownPanel (0.9137   , 0.9373  ,  0.9608)
      PanelBorder (0.8078  ,       0      ,   0)
      PanelText (0.3333     ,    0 ,   0.5020)
      Text (0    ,     0  ,  0.3882)
      Blue (0    ,     0.6 ,  0.8)
   end
    
end

