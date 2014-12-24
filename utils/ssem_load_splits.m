function [splits freqLabels] = ssem_load_splits(params)
%   ssem_load_splits(params): Returns 'splits' and label 'freqLabels'. 'splits' is
%   a cell with the name of the videos belonging to each split of the database and
%   a 'freqLabels' is a matrix reporting the frequence of each labels within each 
%   split (one split per row).

    vids = ssem_load_videos(params);
    load(fullfile(params.root, sprintf(params.split_file, params.n_splits, length(vids))), 'splits', 'freqLabels');    
end
