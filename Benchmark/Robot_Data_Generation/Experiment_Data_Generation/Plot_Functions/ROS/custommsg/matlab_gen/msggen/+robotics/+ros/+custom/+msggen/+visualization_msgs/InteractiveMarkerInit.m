classdef InteractiveMarkerInit < robotics.ros.Message
    %InteractiveMarkerInit MATLAB implementation of visualization_msgs/InteractiveMarkerInit
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'visualization_msgs/InteractiveMarkerInit' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'd5f2c5045a72456d228676ab91048734' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        VisualizationMsgsInteractiveMarkerClass = robotics.ros.msg.internal.MessageFactory.getClassForType('visualization_msgs/InteractiveMarker') % Dispatch to MATLAB class for message type visualization_msgs/InteractiveMarker
    end
    
    properties (Dependent)
        ServerId
        SeqNum
        Markers
    end
    
    properties (Access = protected)
        Cache = struct('Markers', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Markers', 'SeqNum', 'ServerId'} % List of non-constant message properties
        ROSPropertyList = {'markers', 'seq_num', 'server_id'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = InteractiveMarkerInit(msg)
            %InteractiveMarkerInit Construct the message object InteractiveMarkerInit
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function serverid = get.ServerId(obj)
            %get.ServerId Get the value for property ServerId
            serverid = char(obj.JavaMessage.getServerId);
        end
        
        function set.ServerId(obj, serverid)
            %set.ServerId Set the value for property ServerId
            serverid = convertStringsToChars(serverid);
            
            validateattributes(serverid, {'char', 'string'}, {}, 'InteractiveMarkerInit', 'ServerId');
            
            obj.JavaMessage.setServerId(serverid);
        end
        
        function seqnum = get.SeqNum(obj)
            %get.SeqNum Get the value for property SeqNum
            seqnum = typecast(int64(obj.JavaMessage.getSeqNum), 'uint64');
        end
        
        function set.SeqNum(obj, seqnum)
            %set.SeqNum Set the value for property SeqNum
            validateattributes(seqnum, {'numeric'}, {'nonempty', 'scalar'}, 'InteractiveMarkerInit', 'SeqNum');
            
            obj.JavaMessage.setSeqNum(seqnum);
        end
        
        function markers = get.Markers(obj)
            %get.Markers Get the value for property Markers
            if isempty(obj.Cache.Markers)
                javaArray = obj.JavaMessage.getMarkers;
                array = obj.readJavaArray(javaArray, obj.VisualizationMsgsInteractiveMarkerClass);
                obj.Cache.Markers = feval(obj.VisualizationMsgsInteractiveMarkerClass, array);
            end
            markers = obj.Cache.Markers;
        end
        
        function set.Markers(obj, markers)
            %set.Markers Set the value for property Markers
            if ~isvector(markers) && isempty(markers)
                % Allow empty [] input
                markers = feval([obj.VisualizationMsgsInteractiveMarkerClass '.empty'], 0, 1);
            end
            
            validateattributes(markers, {obj.VisualizationMsgsInteractiveMarkerClass}, {'vector'}, 'InteractiveMarkerInit', 'Markers');
            
            javaArray = obj.JavaMessage.getMarkers;
            array = obj.writeJavaArray(markers, javaArray, obj.VisualizationMsgsInteractiveMarkerClass);
            obj.JavaMessage.setMarkers(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Markers)
                obj.Cache.Markers = [];
                obj.Cache.Markers = obj.Markers;
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Markers = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.ServerId = obj.ServerId;
            cpObj.SeqNum = obj.SeqNum;
            
            % Recursively copy compound properties
            cpObj.Markers = copy(obj.Markers);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.ServerId = strObj.ServerId;
            obj.SeqNum = strObj.SeqNum;
            MarkersCell = arrayfun(@(x) feval([obj.VisualizationMsgsInteractiveMarkerClass '.loadobj'], x), strObj.Markers, 'UniformOutput', false);
            obj.Markers = vertcat(MarkersCell{:});
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.ServerId = obj.ServerId;
            strObj.SeqNum = obj.SeqNum;
            strObj.Markers = arrayfun(@(x) saveobj(x), obj.Markers);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.visualization_msgs.InteractiveMarkerInit.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.visualization_msgs.InteractiveMarkerInit;
            obj.reload(strObj);
        end
    end
end