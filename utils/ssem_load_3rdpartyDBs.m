function imgs = ssem_load_3rdpartyDBs(params)
    ndbs = length(params.db_3rdparty);
    imgs = cell(ndbs, 1);
    count = 0;
    for i = 1 : ndbs
        eval(sprintf('imgs{i} = load_3rdpartyDB_%s(params.db_3rdparty(i).path);', params.db_3rdparty(i).name));
        imgs{i}(1).parent = [];
        imgs{i}(1).frames = [];
        imgs{i}(1).backimg = [];
        for j = 1 : length(imgs{i})
            imgs{i}(j).id = sprintf('img%d', count);
            imgs{i}(j).db = i;
            imgs{i}(j).db = i;
            count = count + 1;
        end
    end    
    imgs = cat(1, imgs{:});
    
    fprintf('%d images loaded !\n', length(imgs));
end