function subvids = ssem_split_videos(params, vidlist, fullannots)
%   ssem_split_videos(params, vidlist, fullannots): Replace each video in the cell
%   'vidlist' by its sub-videos. 'fullannots' is optionnal. If false this 
%   function return a cell with the name of corresponding sub-videos. It true it
%   return a structure with following fields: the field 'name' is the name of the
%   sub-video, the field 'frames' indicate frames belonging to it (see 
%   tmp/xxx/img/) and the field 'backimg' indicate if a frame should be used to 
%   extract the background.

    if ~exist('fullannots', 'var')
        fullannots = 0;
    end

    subvids = cell(1, length(vidlist));
    for i = 1 : length(vidlist)
         load(fullfile(params.root, 'annots', 'splits', [regexprep(vidlist{i}, '-', '_') '.mat']), 'vids');
         subvids{i} = vids;
    end
    subvids = cat(1, subvids{:});
    
    if ~fullannots
        subvids = {subvids(:).id};
    end
end
