function demo()
    if ~exist('RoomAnnotTool', 'dir')
        system('git clone git@github.com:vdel/RoomAnnotTool.git');
    end
    
    addpath('utils');
    
    params = ssem_set_params_sub(pwd);
    params.toolpath = fullfile(pwd, 'RoomAnnotTool');
    params.objectLib = fullfile(params.toolpath, 'objectslibrary.xml');
    
    vids = ssem_load_subvideos(params, [], 1);

    rm = RoomManager(params, 1);
    for i = 1 : length(vids)
        rm.open(vids(i).id);
        
        rm.show();
        depth = rm.getDepthMap();
        labels = rm.getLabelMap();
        layout = rm.getLayoutMap();
        
        figure(1);
        imagesc(depth);
        title('Depth map');
        
        figure(2);
        imagesc(labels(:, :, 1));
        title('Label map');
        
        figure(3);
        imagesc(double(layout(:, :, 1)));
        title('Layout map');
        
        figure(4);
        imagesc(double(layout(:, :, 2)));
        title('Clutter map');
        
        figure(5);
        imagesc(double(layout(:, :, 3)));
        title('Object map');
        
        pause;
        
        rm.close()
    end
end

