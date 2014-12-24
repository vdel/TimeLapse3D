function vids = ssem_load_videos(params)
%   ssem_load_videos(params): Returns a cell with the name of all the videos.
    
    fid = fopen(fullfile(params.root, params.video_file), 'rt');
        
    if fid == 0
        error('Error when opening the file %s.', file);
    end
    
    vids = textscan(fid, '%s', 'delimiter', '\n');
    vids = vids{1};
       
    fclose(fid);    
end
