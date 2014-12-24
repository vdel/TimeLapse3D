function perf = ssem_get_perf_split(params, vids, scores, masks)    
%   ssem_get_perf_split(params, vids, scores): compute performance of the model on
%   the given videos. 'vids' is a cell with the names of the test videos, 'scores'
%   is a cell containing a 3D matrix for each video. The matrix should have the 
%   same number of rows and columns as the background image. A coefficient 
%   '(x, y, l)' should contain a non-negative classification score for pixel 
%   '(x, y)' and label 'l'.
  
    nvids = length(scores);
    ngroups = length(params.annots.visuGroups);
    
    % Load ground truth annotation for test videos
    a = ssem_load_annots(params, vids);  
    annots = cell(1, nvids);
    for i = 1 : nvids
        [h w nlabels] = size(scores{i});
        annot = ssem_unpack_annot(a{i});                
        annot = logical(imresize(annot, [h w], 'nearest'));        
        
        annots{i} = reshape(annot, [h * w nlabels]);
        scores{i} = reshape(single(scores{i}), [h * w nlabels]);
        
        if exist('masks', 'var')
            masks{i} = imresize(masks{i}, [h w]);
            masks{i} = reshape(masks{i}, h * w, 1);
        end
    end    
    annots = cat(1, annots{:});
    scores = cat(1, scores{:});
    if exist('masks', 'var')
        masks = cat(1, masks{:});
        annots = annots(masks, :);
        scores = scores(masks, :);
    end    

    perf = cell(1, ngroups);    
    for k = 1 : ngroups 
        nlabels = length(params.annots.visuGroups(k).foregnd);          
        perf{k} = zeros(1, nlabels);                                         
        for i = 1 : nlabels
            l = logical(sum(annots(:, params.annots.visuGroups(k).foregnd{i}), 2));
            [~, ~, perf{k}(i)] = precisionrecall(mean(scores(:, params.annots.visuGroups(k).foregnd{i}), 2), l);                        
        end
        perf{k} = perf{k} * 100;
    end
end
