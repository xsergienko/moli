function []=Moli_install()
%-------------------------------------------------------------------------
% This function installs MOLI toolbox. Specifically, it adds following
% folders to the MATLAB path:
%           ModelLibrary  - Folder with all models
%           MoliFunctions - Folder with all Moli functions
%-------------------------------------------------------------------------
fprintf('Adding MoLi to hte MATLAB path ... \n')
% Adding MOLI to MATLAB path
addpath( genpath([pwd filesep 'ModelLibrary']) )
addpath( genpath([pwd filesep 'MoliFunctions']) )
fprintf('... process successful!\n')
end




