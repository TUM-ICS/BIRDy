classdef InteractiveMarkerControl < robotics.ros.Message
    %InteractiveMarkerControl MATLAB implementation of visualization_msgs/InteractiveMarkerControl
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'visualization_msgs/InteractiveMarkerControl' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'b3c81e785788195d1840b86c28da1aac' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant)
        INHERIT = uint8(0)
        FIXED = uint8(1)
        VIEWFACING = uint8(2)
        NONE = uint8(0)
        MENU = uint8(1)
        BUTTON = uint8(2)
        MOVEAXIS = uint8(3)
        MOVEPLANE = uint8(4)
        ROTATEAXIS = uint8(5)
        MOVEROTATE = uint8(6)
        MOVE3D = uint8(7)
        ROTATE3D = uint8(8)
        MOVEROTATE3D = uint8(9)
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsQuaternionClass = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Quaternion') % Dispatch to MATLAB class for message type geometry_msgs/Quaternion
        VisualizationMsgsMarkerClass = robotics.ros.msg.internal.MessageFactory.getClassForType('visualization_msgs/Marker') % Dispatch to MATLAB class for message type visualization_msgs/Marker
    end
    
    properties (Dependent)
        Name
        Orientation
        OrientationMode
        InteractionMode
        AlwaysVisible
        IndependentMarkerOrientation
        Description
        Markers
    end
    
    properties (Access = protected)
        Cache = struct('Orientation', [], 'Markers', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'AlwaysVisible', 'Description', 'IndependentMarkerOrientation', 'InteractionMode', 'Markers', 'Name', 'Orientation', 'OrientationMode'} % List of non-constant message properties
        ROSPropertyList = {'always_visible', 'description', 'independent_marker_orientation', 'interaction_mode', 'markers', 'name', 'orientation', 'orientation_mode'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = InteractiveMarkerControl(msg)
            %InteractiveMarkerControl Construct the message object InteractiveMarkerControl
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
        
        function name = get.Name(obj)
            %get.Name Get the value for property Name
            name = char(obj.JavaMessage.getName);
        end
        
        function set.Name(obj, name)
            %set.Name Set the value for property Name
            name = convertStringsToChars(name);
            
            validateattributes(name, {'char', 'string'}, {}, 'InteractiveMarkerControl', 'Name');
            
            obj.JavaMessage.setName(name);
        end
        
        function orientation = get.Orientation(obj)
            %get.Orientation Get the value for property Orientation
            if isempty(obj.Cache.Orientation)
                obj.Cache.Orientation = feval(obj.GeometryMsgsQuaternionClass, obj.JavaMessage.getOrientation);
            end
            orientation = obj.Cache.Orientation;
        end
        
        function set.Orientation(obj, orientation)
            %set.Orientation Set the value for property Orientation
            validateattributes(orientation, {obj.GeometryMsgsQuaternionClass}, {'nonempty', 'scalar'}, 'InteractiveMarkerControl', 'Orientation');
            
            obj.JavaMessage.setOrientation(orientation.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Orientation)
                obj.Cache.Orientation.setJavaObject(orientation.getJavaObject);
            end
        end
        
        function orientationmode = get.OrientationMode(obj)
            %get.OrientationMode Get the value for property OrientationMode
            orientationmode = typecast(int8(obj.JavaMessage.getOrientationMode), 'uint8');
        end
        
        function set.OrientationMode(obj, orientationmode)
            %set.OrientationMode Set the value for property OrientationMode
            validateattributes(orientationmode, {'numeric'}, {'nonempty', 'scalar'}, 'InteractiveMarkerControl', 'OrientationMode');
            
            obj.JavaMessage.setOrientationMode(orientationmode);
        end
        
        function interactionmode = get.InteractionMode(obj)
            %get.InteractionMode Get the value for property InteractionMode
            interactionmode = typecast(int8(obj.JavaMessage.getInteractionMode), 'uint8');
        end
        
        function set.InteractionMode(obj, interactionmode)
            %set.InteractionMode Set the value for property InteractionMode
            validateattributes(interactionmode, {'numeric'}, {'nonempty', 'scalar'}, 'InteractiveMarkerControl', 'InteractionMode');
            
            obj.JavaMessage.setInteractionMode(interactionmode);
        end
        
        function alwaysvisible = get.AlwaysVisible(obj)
            %get.AlwaysVisible Get the value for property AlwaysVisible
            alwaysvisible = logical(obj.JavaMessage.getAlwaysVisible);
        end
        
        function set.AlwaysVisible(obj, alwaysvisible)
            %set.AlwaysVisible Set the value for property AlwaysVisible
            validateattributes(alwaysvisible, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'InteractiveMarkerControl', 'AlwaysVisible');
            
            obj.JavaMessage.setAlwaysVisible(alwaysvisible);
        end
        
        function independentmarkerorientation = get.IndependentMarkerOrientation(obj)
            %get.IndependentMarkerOrientation Get the value for property IndependentMarkerOrientation
            independentmarkerorientation = logical(obj.JavaMessage.getIndependentMarkerOrientation);
        end
        
        function set.IndependentMarkerOrientation(obj, independentmarkerorientation)
            %set.IndependentMarkerOrientation Set the value for property IndependentMarkerOrientation
            validateattributes(independentmarkerorientation, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'InteractiveMarkerControl', 'IndependentMarkerOrientation');
            
            obj.JavaMessage.setIndependentMarkerOrientation(independentmarkerorientation);
        end
        
        function description = get.Description(obj)
            %get.Description Get the value for property Description
            description = char(obj.JavaMessage.getDescription);
        end
        
        function set.Description(obj, description)
            %set.Description Set the value for property Description
            description = convertStringsToChars(description);
            
            validateattributes(description, {'char', 'string'}, {}, 'InteractiveMarkerControl', 'Description');
            
            obj.JavaMessage.setDescription(description);
        end
        
        function markers = get.Markers(obj)
            %get.Markers Get the value for property Markers
            if isempty(obj.Cache.Markers)
                javaArray = obj.JavaMessage.getMarkers;
                array = obj.readJavaArray(javaArray, obj.VisualizationMsgsMarkerClass);
                obj.Cache.Markers = feval(obj.VisualizationMsgsMarkerClass, array);
            end
            markers = obj.Cache.Markers;
        end
        
        function set.Markers(obj, markers)
            %set.Markers Set the value for property Markers
            if ~isvector(markers) && isempty(markers)
                % Allow empty [] input
                markers = feval([obj.VisualizationMsgsMarkerClass '.empty'], 0, 1);
            end
            
            validateattributes(markers, {obj.VisualizationMsgsMarkerClass}, {'vector'}, 'InteractiveMarkerControl', 'Markers');
            
            javaArray = obj.JavaMessage.getMarkers;
            array = obj.writeJavaArray(markers, javaArray, obj.VisualizationMsgsMarkerClass);
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
            obj.Cache.Orientation = [];
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
            cpObj.Name = obj.Name;
            cpObj.OrientationMode = obj.OrientationMode;
            cpObj.InteractionMode = obj.InteractionMode;
            cpObj.AlwaysVisible = obj.AlwaysVisible;
            cpObj.IndependentMarkerOrientation = obj.IndependentMarkerOrientation;
            cpObj.Description = obj.Description;
            
            % Recursively copy compound properties
            cpObj.Orientation = copy(obj.Orientation);
            cpObj.Markers = copy(obj.Markers);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Name = strObj.Name;
            obj.OrientationMode = strObj.OrientationMode;
            obj.InteractionMode = strObj.InteractionMode;
            obj.AlwaysVisible = strObj.AlwaysVisible;
            obj.IndependentMarkerOrientation = strObj.IndependentMarkerOrientation;
            obj.Description = strObj.Description;
            obj.Orientation = feval([obj.GeometryMsgsQuaternionClass '.loadobj'], strObj.Orientation);
            MarkersCell = arrayfun(@(x) feval([obj.VisualizationMsgsMarkerClass '.loadobj'], x), strObj.Markers, 'UniformOutput', false);
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
            
            strObj.Name = obj.Name;
            strObj.OrientationMode = obj.OrientationMode;
            strObj.InteractionMode = obj.InteractionMode;
            strObj.AlwaysVisible = obj.AlwaysVisible;
            strObj.IndependentMarkerOrientation = obj.IndependentMarkerOrientation;
            strObj.Description = obj.Description;
            strObj.Orientation = saveobj(obj.Orientation);
            strObj.Markers = arrayfun(@(x) saveobj(x), obj.Markers);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.visualization_msgs.InteractiveMarkerControl.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.visualization_msgs.InteractiveMarkerControl;
            obj.reload(strObj);
        end
    end
end
