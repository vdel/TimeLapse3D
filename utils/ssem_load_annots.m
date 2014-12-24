function annot = ssem_load_annots(params, vids)
%   ssem_load_annots(params, vids): Returns the object segmentation as a structure
%   for all videos in 'vids'. Second argument is optionnal. It should be a cell 
%   containing name of the sub-videos you want to load the object annotations from
%   (as returned by 'ssem_load_subvideos(params);').
%   Use "ssem_unpack_annot.m" to fully load the ground truth annotation. Each 
%   entry of the returned structure contains the following fields:
%     - size: a vector [height width] of labels after unpacking 
%       (see "ssem_unpack_annot.m").
%     - nlabels: number of labels.
%     - annot: a INT 2D matrix where each coefficient a the binary representation
%       of the set of labels covreing the corresponding pixel. For example is 
%       pixel should be annotated with labels 1 and 4, the matrix entry will
%       be 9 = 2 ^ (1 - 1) + 2 ^ (4 - 1).
%     - vid: name of the sub-video
%     - map: mapping from labels in params.annots.label to labels in the MAT file.

    directory = fullfile(params.root, 'annots', 'labels', params.annots.name);   
    if nargin < 2
        vids = ssem_load_subvideos(params);
    end   
    
    nlabels = length(params.annots.labels);
    annot = cell(1, length(vids));        
    
    for i = 1 : length(vids)                       
        load(fullfile(directory, sprintf('%s.mat', vids{i})), 'labels', 'labels_size', 'labels_name');        
        map = zeros(nlabels, 1);
        for j = 1 : nlabels
            I = find(strcmp(params.annots.labels{j}, labels_name));
            if isempty(I)
                error('Could not find label "%s" for video "%s".', params.annots.labels{j}, vids{i});
            elseif length(I) > 2
                error('Duplicate label "%s" for video "%s".', params.annots.labels{j}, vids{i});
            else
                map(j) = I;
            end
        end        
        annot{i} = struct('size', labels_size(1:2), 'nlabels', labels_size(3), 'annot', labels, 'vid', vids{i}, 'map', map);
    end    
end
