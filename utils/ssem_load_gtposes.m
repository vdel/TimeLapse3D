function gtposes = ssem_load_gtposes(params, vid)
    load(fullfile(params.root, 'annots', 'gtposes', [vid '.mat']), 'gtposes');
end
