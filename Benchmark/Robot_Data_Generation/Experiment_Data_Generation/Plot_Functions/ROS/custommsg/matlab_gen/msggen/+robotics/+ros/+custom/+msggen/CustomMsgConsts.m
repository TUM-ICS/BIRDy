classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    properties (Constant)
        visualization_msgs_ImageMarker = 'visualization_msgs/ImageMarker'
        visualization_msgs_InteractiveMarker = 'visualization_msgs/InteractiveMarker'
        visualization_msgs_InteractiveMarkerControl = 'visualization_msgs/InteractiveMarkerControl'
        visualization_msgs_InteractiveMarkerFeedback = 'visualization_msgs/InteractiveMarkerFeedback'
        visualization_msgs_InteractiveMarkerInit = 'visualization_msgs/InteractiveMarkerInit'
        visualization_msgs_InteractiveMarkerPose = 'visualization_msgs/InteractiveMarkerPose'
        visualization_msgs_InteractiveMarkerUpdate = 'visualization_msgs/InteractiveMarkerUpdate'
        visualization_msgs_Marker = 'visualization_msgs/Marker'
        visualization_msgs_MarkerArray = 'visualization_msgs/MarkerArray'
        visualization_msgs_MenuEntry = 'visualization_msgs/MenuEntry'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(10, 1);
                msgList{1} = 'visualization_msgs/ImageMarker';
                msgList{2} = 'visualization_msgs/InteractiveMarker';
                msgList{3} = 'visualization_msgs/InteractiveMarkerControl';
                msgList{4} = 'visualization_msgs/InteractiveMarkerFeedback';
                msgList{5} = 'visualization_msgs/InteractiveMarkerInit';
                msgList{6} = 'visualization_msgs/InteractiveMarkerPose';
                msgList{7} = 'visualization_msgs/InteractiveMarkerUpdate';
                msgList{8} = 'visualization_msgs/Marker';
                msgList{9} = 'visualization_msgs/MarkerArray';
                msgList{10} = 'visualization_msgs/MenuEntry';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
