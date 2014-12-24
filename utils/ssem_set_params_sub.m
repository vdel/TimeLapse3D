function params = ssem_set_params_sub(root)
%   ssem_set_params_sub(root): Sets up the parameter structure.

    params.root = root;                       % path to the database
    params.video_file = 'video_list.txt';     % file for list of video used
    params.n_splits = 5;                      % number of splits for cross-validation
    params.split_file = 'splits_%d_%d.mat';   % file for DB splits

    % Parameters for adpative background estimation
    params.img_per_mixt = 4;
    params.alpha = 0.01;
    params.T = 0.2;
    params.min_sigma = 90;
    params.adptbck_imsize = [120 160];
    
    % Parameters for labels generation and visualization
    params.annots =  struct('name', 'final', ...                           
                           'labels', {{'Wall' 'Ceiling' 'Walkable' 'Sittable' 'Reachable' 'Bed' 'SofaArmchair' 'CoffeeTable' 'Chair' 'Table' 'Cupboard' 'ChristmasTree' 'OtherObject'}}, ...                                                      
                           'normGroups', {{[1 2 3 6 7 8 9 10 11 12 13] [4 5]}}, ... % Normalization groups should be disjoint
                           'visuGroups', struct('foregnd', { {1 2 3 6 7 8 9 10 11 12 13} {3 4 5} }, ...
                                                'backgnd', { [] [1 2 6 7 8 9 10 11 12 13] }, ...
                                                'name', { 'Objects and room layout' 'Functionnal surfaces' } ), ...
                           'colors', [0.25 0.25 0.25; ... % Wall
                                      0.5  0.5  0.5; ...  % Ceiling
                                      0.75 0.75 0.75; ... % Walkable
                                      1    0    0; ...    % Sittable
                                      0    1    0; ...    % Reachable                                      
                                      1    1    0; ...    % Bed 
                                      1    0    0; ...    % Sofa/Armchair
                                      1    0.67 0; ...    % Coffee Table
                                      0    0    1; ...    % Chair 
                                      1    0    1; ...    % Table
                                      0    1    1; ...    % Cupboard 
                                      0    1    0; ...    % ChristmasTree
                                      1    1    1;  ...   % Other Object                                       
                                     ], ...        
                           'preprocess', {{ ... % used for label generation lhs is labelID, rhs is geomID: 1 is Ceiling, 2 is LeftWall, 3 is FrontWall, 4 is RightWall, 5 is Floor
                                            '1 <- 2 | 3 | 4' ...
                                            '2 <- 1' ...
                                            '3 <- 5' ...
                                         }}, ...
                           'postprocess', {{ ...
                                            '1 <- 1 & ~(6 | 7 | 8 | 9 | 10 | 11 | 12 | 13)' ... % Wall is not object
                                            '2 <- 2 & ~(6 | 7 | 8 | 9 | 10 | 11 | 12 | 13)' ... % Ceiling is not object
                                            '3 <- 3 & ~(6 | 7 | 8 | 9 | 10 | 11 | 12 | 13)' ... % Walkable is not object
                                          }} ...
                           );      
end
