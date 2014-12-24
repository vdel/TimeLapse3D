classdef RoomManager < matlab.mixin.Copyable % to get the copy method
    % Wrapper for Java 3D visualization
        
    properties %(GetAccess = private, SetAccess = private)
        visu3D
        
        vid
        room
        frame
        opened
        params        
        
        classes       
        
        fx
        fy
        g
        
        depth
        width
        height
        
        K
        invK        
        R
        invR
        KR
        invKR        
        t    
     
        switch_xy
        vp
        p0
        p1
        pH
        
        cameraPosition
        cameraAngle
        cameraTopAngle
        
        imgW
        imgH
        img
        imgmaxsize
    end
    
    properties %(GetAccess = public)
        
    end

    properties (Constant)
        FRONT = 0;
        BACK = 1;
        LEFT = 2;
        RIGHT = 3;
        TOP = 4;
        BOTTOM = 5;
        
        TOPLEFT = int32(0);
        TOPRIGHT = int32(1);
        BOTTOMRIGHT = int32(2);
        BOTTOMLEFT = int32(3);
        DEPTHVP = int32(4);
        PH = int32(5);
    end    
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function obj = RoomManager(params, visu3D)
            if ~exist('visu3D', 'var')
                visu3D = 0;
            end
            obj.visu3D = visu3D;
            obj.params = params;
            obj = RoomManager.loadobj(obj);
            obj.makeroom();            
        end                       
        
        function o = saveobj(obj)
            o = obj.copy();
            o.room = [];
        end
        
        function delete(obj)
            obj.free();
                    
            obj.room = [];
            
            try
                java.lang.System.gc();            
            catch % might fail if Matlab is ran without java
            end
        end
        
        function free(obj) 
            if ~isempty(obj.frame)
                obj.hide();
                obj.frame.close();
            end
            obj.frame = [];
            
            if ~isempty(obj.room)
                obj.room.clear(); 
                obj.room = [];
            end
                        
            obj.opened = false;            
            
            try
                java.lang.System.gc();
            catch % might fail if Matlab is ran without java
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function obj = makeroom(obj)   
            if obj.visu3D
                obj.room = com.annot.gui.Room3D(java.io.File(obj.params.objectLib), com.annot.room.Room.READ_ONLY);
            else
                obj.room = com.annot.room.Room(java.io.File(obj.params.objectLib), com.annot.room.Room.READ_ONLY);
            end            
            obj.imgmaxsize = obj.room.getImageMaximumSize();
            
            obj.classes = obj.room.getClasses();
            for i = 1 : length(obj.classes)
                obj.classes{i} = obj.classes{i}';
            end
        end
                
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function openEmptyRoom(obj, depth, width, height, camPos) 
            if obj.opened
                warning('A room is already opened. Please use close().');
                obj.close();
            end
            
            if isempty(obj.room)
                obj.makeroom();
            end
            
            if exist('camPos', 'var')
                obj.room.loadEmptyRoom(depth, width, height, depth / 2 + camPos(1), width / 2 + camPos(2), camPos(3));
            else
                obj.room.loadEmptyRoom(depth, width, height);
            end
            obj.opened = true;
            
            obj.setupParameters();          
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function open(obj, vid) 
            fprintf('In %s\n', vid);
            if obj.opened
                warning('A room is already opened. Please use close().');
                obj.close();
            end
            
            if isempty(obj.room)
                obj.makeroom();
            end
            
            imgPath = fullfile(obj.params.root, 'annots', 'back', [vid '.jpg']);
            xmlPath = fullfile(obj.params.root, 'annots', 'geometry', [vid '.xml']);

            obj.vid = vid;

            obj.room.load(java.io.File(imgPath), java.io.File(xmlPath));
            obj.opened = true;
            obj.setupParameters();   
        end   
        
        function setupParameters(obj)      
            obj.imgW = obj.room.getImage().getWidth();
            obj.imgH = obj.room.getImage().getHeight();
            obj.img = reshape(uint8(obj.room.getImage().toArray()), [obj.imgH obj.imgW 3]);      
            
            obj.fx = obj.room.getParams().fx;
            obj.fy = obj.room.getParams().fy;
            obj.g = obj.room.getParams().g;
                    
            obj.K = reshape(obj.room.getParams().K.toArray(), 3, 3);
            obj.invK = obj.K ^ -1;
            obj.R = reshape(obj.room.getParams().R.toArray(), 3, 3);
            obj.invR = obj.R ^ -1;
            obj.KR = reshape(obj.room.getParams().KR.toArray(), 3, 3);
            obj.invKR = obj.KR ^ -1;            
            obj.t = obj.room.getParams().t.toArray();
            
            obj.pH = obj.room.getCorner(obj.PH).toArray();
            obj.switch_xy = ~obj.room.isCornerVisible(obj.BOTTOMLEFT);
            if ~obj.switch_xy
                obj.p0 = obj.room.getCorner(obj.BOTTOMLEFT).toArray();
                obj.p1 = obj.room.getCorner(obj.BOTTOMRIGHT).toArray();
            else
                obj.p0 = obj.room.getCorner(obj.BOTTOMRIGHT).toArray();
                obj.p1 = obj.room.getCorner(obj.BOTTOMLEFT).toArray();                
            end
            obj.p0 = obj.p0(1:2);
            obj.p1 = obj.p1(1:2);
            obj.pH = obj.pH(1:2);
            obj.vp = zeros(3, 2);
            for i = 1 : 3  % coordinate system are weird.
                vp = obj.room.getVP(i - 1);
                if ~isempty(vp)
                    tmpVP = obj.room.getVP(i - 1).toArray();
                    obj.vp(i, :) = [obj.imgW / 2 + tmpVP(1),
                                    obj.imgH / 2 - tmpVP(2)];
                end
            end
            
            obj.cameraPosition = obj.room.getParams().cameraPosition.toArray();
            obj.cameraAngle = obj.room.getParams().cameraAngle.toArray();
            obj.cameraTopAngle = atan2(obj.invR(2, 3), obj.invR(1, 3));   % facing vector x: 0, facing y: pi/2
            
            obj.depth = max(obj.cameraPosition(1), obj.room.getParams().depth);
            obj.width = max(obj.cameraPosition(2), obj.room.getParams().width);
            obj.height = max(obj.cameraPosition(3), obj.room.getParams().height);            
        end
        
        function close(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            obj.free();
        end           
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function show(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            if isempty(obj.frame)
                obj.frame = com.annot.gui.VisuFrame(100, 100, obj.room);
            end
            if ~obj.frame.isVisible()
                obj.frame.setVisible(true);
            end            
        end
        
        function hide(obj)
            if ~isempty(obj.frame)
                obj.frame.setVisible(false);
            end
        end
        
        function setCameraPosition(obj, pos)
            if ~obj.opened
                error('No room is opened.');
            end
            
            pos = com.common.MyVect(pos(1), pos(2), pos(3));
            obj.frame.getCanvas().getBehavior().setCameraPos(pos);
        end
        
        function img = screenCopy(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            iw = obj.frame.screenCopy();
            img = reshape(uint8(iw.pixels), [iw.height, iw.width, 3]);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function classes = getClasses(obj)
            classes = obj.classes;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function coord = getDistordedImageCoord(obj, coord, srcmaxsize) % (2 or 3)xnxm matrix [X, Y]
            if exist('srcmaxsize', 'var')
                if srcmaxsize < obj.imgmaxsize
                    scale = 1;
                else
                    scale = obj.imgmaxsize / srcmaxsize;
                end
            else
                scale = obj.getImageScale();
            end
            coord = obj.getDistordedImageCoordStatic(scale, ...
                                                     obj.getImageDistortion(), ...
                                                     obj.imgW, obj.imgH, coord);
        end
        
        function coord = getOriginalImageCoord(obj, coord, srcmaxsize) % (2 or 3)xnxm matrix [X, Y]
            coord = obj.getOriginalImageCoordStaticSub(obj.getImageDistortion(), coord);            
            coord = obj.directCoord2img(coord);
            
            if exist('srcmaxsize', 'var')
                if srcmaxsize < obj.imgmaxsize
                    scale = 1;
                else
                    scale = obj.imgmaxsize / srcmaxsize;
                end
            else
                scale = obj.getImageScale();
            end
            coord = coord / scale;
        end
     
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [bump param] = liftPoses(obj, poses)
            if size(poses, 2) == 14                
                nposes = size(poses, 3);
                if size(poses, 1) == 2
                    poses = cat(1, poses, ones(1, 14, nposes));
                end                
            elseif size(poses, 2) == 28 || size(poses, 2) == 29
                if size(poses, 2) == 29
                    poses = poses(:, 1 : (end - 1));
                end
                nposes = size(poses, 1);
                poses = reshape(poses', [2 14 nposes]);
                poses = cat(1, poses, ones(1, 14, nposes));
            else
                error('Unknown pose format.')
            end
            bump = cell(1, nposes);
            param = cell(1, nposes);
            if obj.opened
                v = obj.R * [0; 0; 1];
                topDir = com.common.MyVect(v(1), v(2), v(3));
            else
                topDir = com.common.MyVect();
            end
            human3DPose = com.pose.Pose3D();
            for i = 1 : nposes
                p = poses(:, :, i);                    
                human3DPose.loadFrom2D(reshape(p, 1, 3 * 14));                
                p = human3DPose.lift3D(topDir);
                bump{i} = reshape(p, 3, 14);
                p = human3DPose.getParameters();
                param{i} = reshape(p, 1, length(p));
            end
            bump = cat(3, bump{:});
            param = cat(1, param{:});
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function addPose(obj, pose) % 3x14 matrix: x (pixel), y (pixel), z (meters)
            if ~obj.opened
                error('No room is opened.');
            end
            if size(pose, 1) ~= 3 || size(pose, 2) ~= 14
                error('Pose should be a 3x14 matrix.');
            end
            
            if isempty(obj.frame)
                obj.frame = com.annot.gui.room.VisuFrame(100, 100, obj.room);
            end
            
            %pose(1, : ) = pose(1, : ) + obj.depth / 2;
            %pose(2, : ) = pose(2, : ) + obj.width / 2;
            obj.frame.addPose(reshape(pose, 3 * 14, 1));
        end
        
        function clearPoses(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            if ~isempty(obj.frame)
                obj.frame.clearPoses();
            end
        end      
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function floor = getFloor(obj) 
            if ~obj.opened
                error('No room is opened.');
            end
            floor = obj.room.getFloor();
        end
        
        function ceiling = getCeiling(obj) 
            if ~obj.opened
                error('No room is opened.');
            end
            ceiling = obj.room.getCeiling();
        end
        
        function wall = getWall(obj, i) 
            if ~obj.opened
                error('No room is opened.');
            end
            wall = obj.room.getWall(i - 1);
        end               
                
        function o = getAttachedObjects(obj, parent, part, face) 
            if ~obj.opened
                error('No room is opened.');
            end
            if face < 0 || face > 5
                error('Face should be between 0 and 5');                
            end
            facetype = com.annot.room.ObjectManager.getFaceType(face);
            o = parent.getPart(part).getFace(facetype).getChildrenArray();
        end
        
        function newObject = addObjectTo(obj, parent, part, face, name, x, y, angle) 
            if ~obj.opened
                error('No room is opened.');
            end
            
            % parent: parent object
            % face: face type: obj.FRONT, obj.BACK, obj.LEFT, obj.RIGHT, obj.TOP, obj.BOTTOM};            
            % x, y in meters, angle in degree
%             if angle ~= 0 && angle ~= 90 && angle ~= 180 && angle ~= 270
%                 error('Angle should be 0, 90, 180 or 270 degrees');                
%             end
            if face < 0 || face > 5
                error('Face should be between 0 and 5');                
            end
            newObject = obj.room.newObject(name);
            facetype = com.annot.room.ObjectManager.getFaceType(face);
            newObject.attachTo(parent.getPart(part), facetype);
            newObject.setFacePosition(x, y, angle * pi / 180);
        end
        
        function newObject = addObjectToFloor(obj, name, x, y, angle) 
            % invert x and y because we attach in the face reference
            % coordinates.
            newObject = addObjectTo(obj, obj.getFloor(), 0, obj.TOP, name, y - obj.width / 2, x - obj.depth / 2, angle);
        end
        
        function clearObject(obj, object) 
            if ~obj.opened
                error('No room is opened.');
            end
            
            if ~object.isPrivate()
                object.detach();
            end
        end
        
        function clearObjects(obj)
            obj.room.clearObjects()
        end
        
        function objects = getObjects(obj, parent, part, face)              
            objs = obj.getAttachedObjects(parent, part, face);
            nobjs = length(objs);
            
            attach = 0;
            if parent.isFloor()
                attach = 1;
            elseif parent.isWall()
                attach = 2;
            elseif parent.isCeiling()
                attach = 3;
            end
            
            objects = struct('name', cell(nobjs, 1), 'attach', attach, ... % attach: 0 = other, 1 = floor, 2 = wall, 3 = ceiling
                             'className', [], 'classID', [], ... 
                             'object', [], 'angle', [], 'toCam', [], ...
                             'boundingbox', [], 'bound3D', [], 'center3D', [], ...
                             'attachPoint3D', [], 'attachCoordSystem', [], ...
                             'attachPoint2D', [], ...
                             'corners', [], ... % corners order is frontLeft; backRight; topLeft
                             'dims', [], 'color', [], 'poly', []);
            keep = true(nobjs, 1);
            
            for i = 1 : nobjs
                objects(i).name = objs(i).getNameArray()';
                objects(i).className = objs(i).getClassNameArray()';
                objects(i).classID = objs(i).getClassID() + 1;
                
                objects(i).object = objs(i);
                
                b = objs(i).getBoundingBox(obj.room);
                                              
                dist = obj.getImageDistortion();
                p1 = obj.getOriginalImageCoordStaticSub(dist, [b.xmin; b.ymin])';
                p2 = obj.getOriginalImageCoordStaticSub(dist, [b.xmin; b.ymax])';                
                p3 = obj.getOriginalImageCoordStaticSub(dist, [b.xmax; b.ymin])';
                p4 = obj.getOriginalImageCoordStaticSub(dist, [b.xmax; b.ymax])';         
                p = [p1; p2; p3; p4];
                 
                bmin = min(p, [], 1);
                bmax = max(p, [], 1);
                
                objects(i).boundingbox = [max(bmin, -1.5) min(bmax, 1.5)]; 
                
                if objects(i).boundingbox(3) <= objects(i).boundingbox(1) || ...
                   objects(i).boundingbox(4) <= objects(i).boundingbox(2)
                    keep(i) = false;
                    continue;
                end
                
                objects(i).angle = objs(i).getAngle();
                c = cos(-objects(i).angle);
                s = sin(-objects(i).angle);

                b = objs(i).getBound3D();
                b = [b.min.x b.max.x; b.min.y b.max.y; b.min.z b.max.z];
                objects(i).bound3D = b;
                objects(i).center3D = (b(:, 1) + b(:, 2)) / 2;        
                
                objects(i).attachCoordSystem = [objs(i).getAttachedFaceX().toArray(), ...
                                                objs(i).getAttachedFaceY().toArray(), ...
                                                objs(i).getAttachedFaceNormal().toArray()];
                attachBB = objects(i).attachCoordSystem' * b; 
                objects(i).attachPoint3D = objects(i).attachCoordSystem * [mean(attachBB(1 : 2, :), 2); min(attachBB(3, :))];                
                p = obj.K * (obj.R * objects(i).attachPoint3D + obj.t);
                objects(i).attachPoint2D = [p(1); p(2)] / p(3);

                rot = [c -s 0; s c 0; 0 0 1];                
                d = abs(rot * (b(:, 2) - b(:, 1)));
                objects(i).dims = d;
                
                objects(i).toCam = rot * (obj.cameraPosition - objects(i).center3D);
                
                d = d / 2;
                points = [1  1 -1;  ... % bottom front left
                          1 -1 -1;  ... % bottom front right
                         -1 -1 -1;  ... % bottom back right
                         -1  1 -1;  ... % bottom back left
                          1  1  1;  ... % top front left
                          1 -1  1;  ... % top front right
                         -1 -1  1;  ... % top back right
                         -1  1  1]';  ... % top back left
                                         
                c = zeros(2, 8);
                for v = 1 : 8
                    p = objects(i).center3D + rot' * (d .* points(:, v));
                    p = obj.K * (obj.R * p + obj.t);
                    c(:, v) = [p(1); p(2)] / p(3);
                end
                objects(i).corners = obj.getOriginalImageCoordStaticSub(dist, c)';
                
                objects(i).color = objs(i).getColor(); 
                
                poly = objs(i).getPolygon(obj.room).toArray();
                objects(i).poly = reshape(poly, 2, numel(poly) / 2)'; 
            end 
            objects = objects(keep);
        end          
        
        function bounds = getObjectsOnFloor(obj)
            bounds = obj.getObjects(obj.getFloor(), 0, obj.TOP);            
        end
        
        function objects = getAllObjects(obj)
            objects = cell(6, 1);
            for i = 1 : 4
                objects{i} = obj.getObjects(obj.getWall(i), 0, obj.FRONT);
            end
            objects{5} = obj.getObjects(obj.getFloor(), 0, obj.TOP);
            objects{6} = obj.getObjects(obj.getCeiling(), 0, obj.BOTTOM);
            objects = cat(1, objects{:});
        end
        
%         function bounds = getFloorBounds(obj, classID, frontLeft, backRight) 
%             % Given an object hypothesis with given corners frontLeft and
%             % and backRight [x y] image coordinates, it returns the 
%             % coresponding 2D bounding box on the floor. (the back is 
%             % defined by object usage: it's clear for sofa, table is 
%             % symetric so any opposed corners will do)
%             
%             if ~obj.opened
%                 error('No room is opened.');
%             end
%             
%             bounds = struct('name', obj.classes{classID}, 'className', obj.classes{classID}, 'classID', classID, 'object', [], ...
%                             'bound', [], 'color', [1 1 1], 'center', [], 'angle', [], 'depth', [], 'width', []);
%                         
%             frontLeft = reshape([frontLeft 1], 3, 1);
%             backRight = reshape([backRight 1], 3, 1);
%             frontLeft = obj.getDistordedImageCoord(frontLeft);
%             backRight = obj.getDistordedImageCoord(backRight);
%             
%             rayfront = obj.invK * frontLeft;
%             rayback = obj.invK * backRight;
%             
%             pointfront = obj.cameraPosition + rayfront * (-obj.cameraPosition(3) / rayfront(3));
%             pointback = obj.cameraPosition + rayback * (-obj.cameraPosition(3) / rayback(3));
%             pointfront = pointfront - [obj.depth; obj.width; 0] / 2;
%             pointback = pointback - [obj.depth; obj.width; 0] / 2;
%             
%             if pointfront(1) > pointback(1)
%                 if pointfront(2) > pointback(2)
%                     angle = 0;
%                 else
%                     angle = -pi / 2;
%                 end
%             else
%                 if pointfront(2) > pointback(2)
%                     angle = pi / 2;
%                 else
%                     angle = pi;
%                 end
%             end
%             
%             % remember that system coordinate of a face does not correspond to world's coordinate: xfloor = yworld            
%             xmin = min(pointfront(2), pointback(2)); 
%             xmax = max(pointfront(2), pointback(2));
%             ymin = min(pointfront(1), pointback(1)); 
%             ymax = max(pointfront(1), pointback(1));
%                         
%             bounds.bound = [xmin ymin xmax ymax];
%             bounds.center = (bounds.bound([1 2]) + bounds.bound([3 4])) / 2;
%             bounds.angle = angle;
%             c = cos(bounds.angle);
%             s = sin(bounds.angle);
%             d = [c s; -s c] * [(xmax - xmin); (ymax - ymin)];
%             bounds.depth = abs(d(2));
%             bounds.width = abs(d(1));            
%         end   
        
        function img = getRoomTopView(obj)
            bounds = getObjects(obj, obj.getFloor(), 0, obj.TOP);
            h = round(obj.depth * 100);
            w = round(obj.width * 100);
            img = zeros([h, w, 3]);
            img(:, :, :) = 0.5;
            
            offset = [w h w h] / 2;
            for i = 1 : length(bounds)
                b = min(max(round(bounds(i).bound3D([1 2 4 5]) * 100 + offset), [1 1 -Inf -Inf]), [Inf Inf w h]);                                
                img(b(2) : b(4), b(1) : b(3), :) = repmat(reshape(bounds(i).color, [1, 1, 3]), [(b(4) - b(2) + 1) (b(3) - b(1) + 1)]);
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [mfx, mfy, mg] = getFocal(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            mfx = obj.fx;
            mfy = obj.fy;
            mg = obj.g;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [mdepth, mwidth, mheight] = getDimensions(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            mdepth = obj.depth;
            mwidth = obj.width;
            mheight = obj.height;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function mimg = getImage(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            mimg = obj.img;
        end
        
        function bb = getImageClip(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            bb = [obj.room.getImage().getXmin() ...
                  obj.room.getImage().getYmin() ...
                  obj.room.getImage().getXmax() ...
                  obj.room.getImage().getYmax() ...
                 ];
        end
        
        function scale = getImageScale(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            scale = obj.room.getImage().getScale();            
        end
        
        function distortion = getImageDistortion(obj)
            if ~obj.opened
                error('No room is opened.');
            end
            
            distortion = obj.room.getImage().getDistortion();            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [poses poseparams] = localize3DPosesByRoomBoundaries(obj, poses)
            nposes = size(poses, 3);
            poses(3, :, :) = 1;
            
            vmin = [0, 0, 0]';
            vmax = [obj.depth, obj.width, obj.height]';
            
            poseparams = cell(nposes, 1);
            for i = 1 : nposes
                rays = obj.invKR * poses(:, :, i);                
                [p poseparam] = obj.liftPoses(rays);
                poseparams{i} = poseparam;
                
                lambda = obj.findBoundingLambda(obj.cameraPosition, rays, vmin, vmax);                
                [lambda k] = min(lambda);
                zref = obj.R(3, :) * rays(:, k) * lambda;
                p = p * lambda;
                p(3, :) = p(3, :) + (zref - p(3, k));
                poses(:, :, i) = bsxfun(@plus, obj.invR * p, obj.cameraPosition);
            end
            poseparams = cat(1, poseparams{:});
        end
        
        function poses = localize2DPosesByRoomBoundaries(obj, poses, depthmap)
            assert(size(poses, 2) == 14);
            nposes = size(poses, 3);
            poses(3, :, :) = 1;
            
            vmin = min([0; 0; 0], obj.cameraPosition);
            vmax = max([obj.depth; obj.width; obj.height], obj.cameraPosition);
            
            for i = 1 : nposes                 
                rays = obj.invKR * poses(:, [1 6 7 12], i); % foot and hands in the room
                lambda = obj.findBoundingLambda(obj.cameraPosition, rays, vmin, vmax);
                raydepth = -max(obj.R(3, :) * rays) * min(lambda);                
                
%                 if ~isempty(depthmap)
%                     hip = mean(poses(1 : 2, [3 4], i), 2);
%                     head = poses(1 : 2, 13, i);
%                     joints = [hip head];
%                     points = round(diag([1 -1]) * joints * obj.imgW / 2 + repmat([obj.imgW; obj.imgH] / 2, 1, size(joints, 2))); % point in image coord
%                     points(1, :) = min(max(points(1, :), 1), obj.imgW);
%                     points(2, :) = min(max(points(2, :), 1), obj.imgH);
%                     jointdepth = depthmap((points(1, :) - 1) * obj.imgH + points(2, :));
% 
%                     closest = min(raydepth, min(jointdepth));
%                 end
                                
                rays = obj.invK * poses(:, :, i);
                rays = obj.invR * bsxfun(@times, rays, -raydepth ./ rays(3, :));
                
                poses(:, :, i) = bsxfun(@plus, rays, obj.cameraPosition);
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function points = intersectRay(obj, points, depthmap, offsetRoomCoordinate) % points 2xn matrix, in image coordinate [1 obj.imgW]            
            if ~exist('offsetRoomCoordinate', 'var')
                offsetRoomCoordinate = [0; 0; 0];
            end
            
            vmin = min([0; 0; 0], obj.cameraPosition);
            vmax = max([obj.depth; obj.width; obj.height], obj.cameraPosition);
            
            points = bsxfun(@times, bsxfun(@minus, points, [obj.imgW; obj.imgH] / 2), [1; -1] / (obj.imgW / 2));            
            points = [points; ones(1, size(points, 2))];
            rays = obj.invK * points;
            
            r = obj.invR * rays;
            lambda = obj.findBoundingLambda(obj.cameraPosition, r, vmin, vmax);
            raydepth = -max(obj.R(3, :) * r, [], 1) .* lambda;
            
            if exist('depthmap', 'var') && ~isempty(depthmap)
                points = round(points);
                points(1, :) = min(max(points(1, :), 1), obj.imgW);
                points(2, :) = min(max(points(2, :), 1), obj.imgH);
                raydepth = min(raydepth, depthmap((points(1, :) - 1) * obj.imgH + points(2, :)));
            end
            
            rays = obj.invR * (bsxfun(@plus, bsxfun(@times, rays, -raydepth ./ rays(3, :)), offsetRoomCoordinate));

            points = bsxfun(@plus, rays, obj.cameraPosition);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function depth = getDepthMap(obj)
            if ~obj.opened
                error('No room is opened.');
            end
  
            iw = obj.room.getDepthMap(true);            
            depth = reshape(iw.pixels, [iw.height, iw.width]);
        end
        
        function labels = getLabelMap(obj, allClasses)
            if ~obj.opened
                error('No room is opened.');
            end
            if ~exist('allClasses', 'var')
                allClasses = true;
            end
  
            iw = obj.room.getLabelMap(allClasses == true);            
            labels = reshape(iw.pixels, [iw.height, iw.width, 3]);
            labels = uint8(labels);
        end
        
        function labels = getLabelClasses(obj, treatAsBox, classes, ignoreClass)
            if ~obj.opened
                error('No room is opened.');
            end
            if ~exist('treatAsBox', 'var')
                treatAsBox = 0;
            end
            if ~exist('classes', 'var')
                classes = obj.classes;
            end
            if ~exist('ignoreClass', 'var')
                ignoreClass = {};
            end
            
            objects = obj.getAllObjects();
            
            w2 = obj.imgW / 2;
            h2 = obj.imgH / 2;
            labels = zeros([obj.imgH, obj.imgW, length(classes)]);
            for i = 1 : length(objects)
                if ~isempty(find(strcmp(objects(i).name, ignoreClass), 1))
                    continue;
                end
                
                c = find(strcmp(objects(i).className, classes), 1);
                if isempty(c)
                    continue;
                end
                if treatAsBox
                    hull = convhull(objects(i).corners(:, 1), objects(i).corners(:, 2));
                    poly = objects(i).corners(hull(1 : (end - 1)), :);
                else
                    poly = objects(i).poly;
                end
                poly = bsxfun(@plus, [w2, h2], bsxfun(@times, poly, [w2 -w2]));
                mask = find(poly2mask(poly(:, 1), poly(:, 2), obj.imgH, obj.imgW));
                labels(mask + obj.imgH * obj.imgW * (c - 1)) = 1;
            end
        end
        
        function labels = getLabelColor(obj, treatAsBox, ignoreClassName)
            if ~obj.opened
                error('No room is opened.');
            end
            if ~exist('treatAsBox', 'var')
                treatAsBox = 0;
            end
            if ~exist('ignoreClass', 'var')
                ignoreClassName = {'OtherObject'};
            end
            
%             iw = obj.room.getLabelMap(false);
%             labelsID = reshape(iw.pixels, [iw.height, iw.width, 3]);
%             labelsID = reshape(double(labelsID(:, :, 1)), iw.height * iw.width, 1);
%             
%             labels = cat(1, obj.params.annots.colors(labelsID + 1, :));
%             labels = reshape(labels, [iw.height, iw.width, 3]);
%             
%             w2 = obj.imgW / 2;
%             h2 = obj.imgH / 2;
%             labels = min(1, max(0, imresize(labels, [h2, w2])));
            
            w2 = obj.imgW / 2;
            h2 = obj.imgH / 2;
            labels = zeros([obj.imgH, obj.imgW, 3]);
            
            % Draw walls first
            objects = obj.getAllObjects();
            for i = 1 : length(objects)
                if ~isempty(find(strcmp(objects(i).className, ignoreClassName), 1))
                    continue;
                end
                if isempty(find(strcmp(objects(i).className, {'Wall' 'Ceiling' 'Floor'}), 1))
                    continue
                end
                poly = bsxfun(@plus, [w2, h2], bsxfun(@times, objects(i).poly, [w2 -w2]));
                mask = find(poly2mask(poly(:, 1), poly(:, 2), obj.imgH, obj.imgW));
                labels(mask) = objects(i).color(1);
                labels(mask + obj.imgH * obj.imgW) = objects(i).color(2);
                labels(mask + obj.imgH * obj.imgW * 2) = objects(i).color(3);
            end
            
            % Order by depth of center (may be inaccurate)
            depth = zeros(length(objects), 1);
            camPos = -obj.R' * obj.t;
            for i = 1 : length(objects)
                v = obj.R * (camPos - objects(i).center3D);
                depth(i) = v(3);
            end
            [~, I] = sort(depth, 'descend');
            objects = objects(I);

            for i = 1 : length(objects)
                if ~isempty(find(strcmp(objects(i).className, ignoreClassName), 1)) || ...
                   ~isempty(find(strcmp(objects(i).className, {'Wall' 'Ceiling' 'Floor'}), 1))
                    continue;
                end
                if treatAsBox
                    hull = convhull(objects(i).corners(:, 1), objects(i).corners(:, 2));
                    poly = objects(i).corners(hull(1 : (end - 1)), :);
                else
                    poly = objects(i).poly;
                end
                poly = bsxfun(@plus, [w2, h2], bsxfun(@times, poly, [w2 -w2]));
                mask = find(poly2mask(poly(:, 1), poly(:, 2), obj.imgH, obj.imgW));
                labels(mask) = objects(i).color(1);
                labels(mask + obj.imgH * obj.imgW) = objects(i).color(2);
                labels(mask + obj.imgH * obj.imgW * 2) = objects(i).color(3);
            end
        end
        
        function layout = getLayoutMap(obj, ignoreNoClass)
            if ~obj.opened
                error('No room is opened.');
            end
            
            if ~exist('ignoreNoClass', 'var')
                ignoreNoClass = true;
            end
  
            iw = obj.room.getLayoutMap(ignoreNoClass);            
            layout = reshape(iw.pixels, [iw.height, iw.width, 3]);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function coord = img2directCoord(obj, coord) % (2 or 3)xnxm matrix [X, Y]
            coord = RoomManager.img2directCoordStatic(obj.imgW, obj.imgH, coord);
        end
        
        function coord = directCoord2img(obj, coord) % (2 or 3)xnxm matrix [X, Y]
            coord = RoomManager.directCoord2imgStatic(obj.imgW, obj.imgH, coord);
        end        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [coord valid] = coord3D2direct(obj, coord)  % 3xn matrix
            coord = bsxfun(@plus, obj.R * coord, obj.t);
            valid = coord(3, :) < 0;
            coord = obj.K * coord;
            coord = bsxfun(@rdivide, coord([1 2], :), coord(3, :));
        end
    end
    
    methods (Static) 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function obj = loadobj(obj)    
            RoomManager.setup_java(obj.params);            
            
            obj.frame = [];
            obj.opened = false;  
        end
                
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function setup_java(params)
            persistent javaPathSet;
            if isempty(javaPathSet)
                file = fullfile(params.toolpath, 'dist', 'RoomAnnotTool.jar');
                fprintf('Setting up java path to %s\n', file);
                javaaddpath(file, '-end');
                import com.common.*;
                import com.annot.*;
                import com.pose.*;
                com.common.J3DHelper.setupJavaPath(java.io.File(params.objectLib));
                javaPathSet = 1;
            end
            mlock;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [center dims angle] = cornertoBox3D(invKR, cameraPosition, scale, distortion, imgW, imgH, corners)
            corners = RoomManager.getDistordedImageCoordStatic(scale, distortion, imgW, imgH, corners');
            rays = invKR * [corners; ones(1, 8)];
            ground = zeros(3, 4);
            top = zeros(3, 4);
            for i = 1 : 4
                ground(:, i) = intersectPlaneRay([0; 0; 0], [0; 0; 1], cameraPosition, rays(:, i));
                top(:, i) = intersectRayRay(ground(:, i), [0; 0; 1], cameraPosition, rays(:, 4+i));
            end
            center = mean((ground + top) / 2, 2);
            v = ground(:, 1) - ground(:, 4);
            angle = atan2(v(2), v(1));
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function lambda = findBoundingLambda(cst, cft, vmin, vmax)
            l1 = bsxfun(@rdivide, bsxfun(@minus, vmin, cst), cft);
            l1(l1 <= 1e-3) = Inf;            
            l2 = bsxfun(@rdivide, bsxfun(@minus, vmax, cst), cft);
            l2(l2 <= 1e-3) = Inf;            
            
            lambda = min(min(l1, l2), [], 1);                         
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function coord = img2directCoordStatic(imgW, imgH, coord) % (2 or 3)xnxm matrix [X, Y]
            xc = imgW / 2;
            yc = imgH / 2;
            
            coord(1, :, :) = coord(1, :, :) - xc;
            coord(2, :, :) = yc - coord(2, :, :);     
              
            coord(1 : 2, :, :) = coord(1 : 2, :, :) * (1 / xc);
        end
        
        function coord = directCoord2imgStatic(imgW, imgH, coord) % (2 or 3)xnxm matrix [X, Y]
            xc = imgW / 2;
            yc = imgH / 2;
            
            coord(1 : 2, :, :) = coord(1 : 2, :, :) * xc;
            
            coord(1, :, :) = xc + coord(1, :, :);
            coord(2, :, :) = yc - coord(2, :, :);     
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function coord = getDistordedImageCoordStatic(scale, distortion, imgW, imgH, coord) % (2 or 3)xnxm matrix [X, Y]
            coord([1 2], :, :) = coord([1 2], :, :) * scale;            
            
            coord = RoomManager.img2directCoordStatic(imgW, imgH, coord);
                           
            if distortion ~= 0                                            
                coord = RoomManager.getDistordedImageCoordStaticSub(distortion, coord);
            end
        end
        
        function coord = getOriginalImageCoordStatic(scale, distortion, imgW, imgH, coord) % (2 or 3)xnxm matrix [X, Y]
            coord([1 2], :, :) = coord([1 2], :, :) * scale;            
            
            xc = imgW / 2;
            yc = imgH / 2;
            
            coord(1, :, :) = coord(1, :, :) - xc;
            coord(2, :, :) = yc - coord(2, :, :);           
                
            if distortion ~= 0                            
                coordScale = 1 / xc;
                
                coord(1 : 2, :, :) = coord(1 : 2, :, :) * coordScale;
                coord = RoomManager.getOriginalImageCoordStaticSub(distortion, coord);
                coord(1 : 2, :, :) = coord(1 : 2, :, :) / coordScale;
            end
        end
        
        function coord = getDistordedImageCoordStaticSub(distortion, coord) % (2 or 3)xnxm matrix [X, Y]
            if distortion ~= 0                            
                rmax = 1; %(xc * coordScale) ^ 2 + (yc * coordScale) ^ 2;
                p = 1 / distortion;
                q = - rmax / distortion;
                delta = q .^ 2 + 4. * (p .^ 3) / 27.;
                rmax = cbrt((-q + sqrt(delta)) / 2.) + ...
                       cbrt((-q - sqrt(delta)) / 2.);
                rescale = (1 + distortion * rmax * rmax);            

                r2 = sum((coord(1 : 2, :, :) / rescale) .^ 2, 1);
                coord(1 : 2, :, :) = bsxfun(@times, (1 + distortion * r2), coord(1 : 2, :, :));
            end
        end
        
        function coord = getOriginalImageCoordStaticSub(distortion, coord) % (2 or 3)xnxm matrix [X, Y]
            if distortion ~= 0                            
                rmax = 1; %(xc * coordScale) ^ 2 + (yc * coordScale) ^ 2;
                p = 1 / distortion;
                q = - rmax / distortion;
                delta = q .^ 2 + 4. * (p .^ 3) / 27.;
                rmax = cbrt((-q + sqrt(delta)) / 2.) + ...
                       cbrt((-q - sqrt(delta)) / 2.);
                rescale = (1 + distortion * rmax * rmax);          
                
                r1 = sqrt(sum(coord(1 : 2, :, :) .^ 2, 1));
                q = - r1 / distortion;                
                delta = q .^ 2 + 4. * (p .^ 3) / 27.;
                r0 = cbrt((-q + sqrt(delta)) / 2.) + ...
                     cbrt((-q - sqrt(delta)) / 2.);     
                 
                coord(1 : 2, :, :) = bsxfun(@times, rescale ./ (1 + distortion .* r0 .* r0), coord(1 : 2, :, :));
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function frame = showPose(pose, frame) % 3x14 matrix is room coordinate: z is up.
            pose3D = com.pose.Pose3DVisu();
            % we convert from camera coordinates
            pose = [0  1  0; ...
                    0  0  1; ...
                    1  0  0] * pose; 
            pose3D.loadFrom3D(reshape(pose, 1, 3 * 14));
            if nargin == 1
                frame = pose3D.showPose();
            else
                pose3D.showPose(frame);
            end
        end
    end    
end

