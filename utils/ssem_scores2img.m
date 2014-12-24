function [img proba] = ssem_scores2img(params, scores, groupID, hardassign)
%   ssem_scores2img(params, scores, groupID): Returns an image corresponding to 
%   the soft segmentation for labels of group 'groupID'. Colors and groups are 
%   defined in "ssem_set_params_sub.m".
  
    [h w ~] = size(scores);
    
    if ~exist('hardassign', 'var')
        hardassign = 0;
    end
    
    % selected only labels from asked group
    Iback = params.annots.visuGroups(groupID).backgnd;
    Igroup = [Iback params.annots.visuGroups(groupID).foregnd];
    
    nlabels = length(Igroup);
    proba = zeros(h, w, length(Igroup));
    color = zeros(nlabels, 3);    
    for i = 1 : nlabels
        proba(:, :, i) = sum(scores(:, :, Igroup{i}), 3);
        if isempty(Iback) || i > 1     % Background is black
            color(i, :) = mean(params.annots.colors(Igroup{i}, :), 1);
        end
    end
    
    if hardassign
        [~, index] = max(proba, [], 3);
        proba = zeros(size(proba));
        npix = size(index, 1) * size(index, 2);
        proba((index(:) - 1) * npix + (1 : npix)') = 1;
    end
    
    proba = bsxfun(@rdivide, proba, sum(proba, 3) + eps);
    proba = reshape(proba, [h * w nlabels]);
    
    img = proba * color;
    img(img > 1) = 1;
    img = reshape(img, [h w 3]);
end