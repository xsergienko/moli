classdef Moli  < handle
% A team from the Slovak University of Technology, consisting of
% Bc. Sofiia Serhiienko, Dr. Juraj Holaza,  and Assoc. Prof. Juraj Oravec,
% is working on developing a compact software package that provides 
% users with a selection of mathematical models for advanced process 
% control applications. 
% The project aims to construct models for systems with different 
% properties and categorize them into subgroups. Another goal is to create 
% a library and user interface in MATLAB, allowing easy  easy and organized 
% access to the mathematical models.
% Moli (an abbreviation for MOdel LIbrary) is developed using MATLAB version 2023b.

    properties  
        dynamics        % dynamics of the system in continous/discrete time
        property        % properties of the system 
        constraints     % constraints of the system 
        info            % info of the system 
        list            % list of models
    end
       
    methods ( Access = public ) 
   %% Constructor
    function obj = Moli()
     
        obj.dynamics = [];
        obj.property = [];
        obj.constraints = [];
        obj.info = [];
        obj.list = [];

        temp = which('Moli_inverted_pendulum');
        temp = split(temp,'Moli_inverted_pendulum.m');
       
        PathModels = temp{1};
        last_backslash_index = find(PathModels == '\', 1, 'last');
        PathModels = PathModels(1:last_backslash_index-1);
        obj.info.PathModels = PathModels;
    end
    
    function out = getName(obj)
        out = 'Moli model';
    end

   %%
    function obj= nameModel(obj)
        nameModel = {};
    
        fullPath = obj.info.PathModels;
    
        % Get a list of files in the current directory
        files = dir(fullPath);
        
        % Loop through each file
        for i = 1:length(files)
        % Get the file name
        filename = files(i).name;
        
        % Check if the file ends with '.m' and starts with 'Moli_'
        if endsWith(filename, '.m') && startsWith(filename, 'Moli_')
        % Add the filename to the nameModel cell array
        nameModel{end+1,1} = filename;
        end 
        end

        obj.list=nameModel;
     end

   %% LTI model
    function [MPTmodel] = getMPTModel(obj,Q,R)
       if nargin < 2
           Q = QuadFunction(eye(obj.property.nx));
           R = QuadFunction(eye(obj.property.nu));
       end

        MPTmodel = LTISystem(obj.dynamics.sysd);
        MPTmodel.x.min = obj.constraints.x.min - obj.dynamics.params.xs;
        MPTmodel.x.max = obj.constraints.x.max - obj.dynamics.params.xs;
        MPTmodel.u.min = obj.constraints.u.min - obj.dynamics.params.us;
        MPTmodel.u.max = obj.constraints.u.max - obj.dynamics.params.us;
        MPTmodel.x.penalty = Q;
        MPTmodel.u.penalty = R;
    end
 
    %%
    function [] = ShowPicture(obj)
       imshow(obj.info.image_data)
    end

    %%
    function [] =  getModel(obj, nameModel)
        % Check if nameModel is a string
        if isstring(nameModel) || ischar(nameModel)
        % Check if the nameModel exists in the list
        if ismember(nameModel, obj.list.Name_of_list)
        % Call the model function to get properties, dynamics, constraints, and info
        try
            eval(['[prop, dyn, con, infomodel] = ', nameModel, '();'])
        catch
            error('Ups, can not load the selected model!')
        end 

        % Assign the retrieved values to the object's properties
        obj.property = prop;
        obj.dynamics = dyn;
        obj.constraints = con;

            try
            obj.info.source = infomodel.source;
            catch
             error('Ups, can not load the source information from the model!');
            end
            try
            obj.info.text = infomodel.text;
            catch
             error('Ups, can not load the text information from the model!');
            end
            try
            obj.info.image_data = infomodel.image_data;
            catch
             error('Ups, can not load the image data from the model!');
            end
        else
        % Handle the case when nameModel doesn't exist in the list
        error(['Model "%s" does not exist in the list.' ...
            ' Please select another model that is already' ...
            ' included in the list.'], nameModel);
        end
        else
        % Handle the case when nameModel is not a string
        error('Input argument nameModel must be a string.');
        end
    end

    %%
    function [] = barListOfModels(obj)
        T = obj.list;
        
        % Count occurrences for each variable
        counts_states = histcounts(T.Number_of_states_nx, 'BinMethod', 'integers');
        counts_inputs = histcounts(T.Number_of_inputs_nu, 'BinMethod', 'integers');
        counts_outputs = histcounts(T.Number_of_outputs_ny, 'BinMethod', 'integers');
        
        % Create subplot
        figure;
        
        % Plot for Number_of_states_nx
        subplot(3, 1, 1); hold on, grid on, box on,
        bar(unique(T.Number_of_states_nx), counts_states, 0.3,'FaceColor', "#80B3FF");
        xlabel('Number of States (nx)');
        ylabel('Number of Models');
        title('Distribution of Number of States');
        xticks(unique(T.Number_of_states_nx)); xtickformat('%d'); ytickformat('%d'); 
        ylim([0, max(counts_states)+0.5]); yticks(1:max(counts_states));
        set(gca, 'fontsize', 9.5, 'ticklabelinterpreter', 'latex')
        
        % Plot for Number_of_inputs_nu
        subplot(3, 1, 2); hold on, grid on, box on,
        bar(unique(T.Number_of_inputs_nu), counts_inputs, 0.3, 'FaceColor', [70, 100, 205] / 255);
    
        xlabel('Number of Inputs (nu)');
        ylabel('Number of Models');
        title('Distribution of Number of Inputs');
        xticks(unique(T.Number_of_inputs_nu)); xtickformat('%d'); ytickformat('%d'); 
        ylim([0, max(counts_inputs)+0.5]); yticks(1:max(counts_inputs));
        set(gca, 'fontsize', 9.5, 'ticklabelinterpreter', 'latex')
        
        % Plot for Number_of_outputs_ny
        subplot(3, 1, 3); hold on, grid on, box on,
        bar(unique(T.Number_of_outputs_ny),counts_outputs, 0.3, 'FaceColor', [0, 71, 171] / 255);
        xlabel('Number of Outputs (ny)');
        ylabel('Number of Models');
        title('Distribution of Number of Outputs');
        xticks(unique(T.Number_of_outputs_ny)); xtickformat('%d'); ytickformat('%d'); 
        ylim([0, max(counts_outputs)+0.5]); yticks(1:max(counts_outputs));
        set(gca, 'fontsize', 9.5, 'ticklabelinterpreter', 'latex')
    end
    
    %%
    function [filtered_table] = getListOfModels(obj,filterOption)
      if nargin < 2, filterOption = 0; end
      
% --------- Construction of the full list (fast reading) ----------
        nameModel = {};
        fullPath = obj.info.PathModels;

        % Get a list of files in the current directory
        files = dir(fullPath);

        % Loop through each file
        for i = 1:length(files)
            % Get the file name
            filename = files(i).name;

            % Check if the file ends with '.m' and starts with 'Moli_'
            if endsWith(filename, '.m') && startsWith(filename, 'Moli_')
                % Add the filename to the nameModel cell array
                nameModel{end+1,1} = filename;
            end
        end

        list = nameModel;
        
% ------------- Construction of the full list (as Table) -----------------
        % Initialize the prop matrix
        prop = nan(length(list), 3);

        % Loop through each function in the list
        for k = 1:length(list)
            % Load properties from the function
            eval(['properties = ', list{k}(1:end-2), '();'])
            % Store properties in the prop matrix
            prop(k,1) = properties.nx;
            prop(k,2) = properties.nu;
            prop(k,3) = properties.ny;
        end

        % Initialize a cell array to store the modified names
        modified_list = cell(size(list));
        % Loop through each element in the list
        for k = 1:numel(list)
            % Remove the last two characters (.m) from the current element and store it in modified_list
            modified_list{k} = list{k}(1:end-2);
        end
        
        % Create IDs starting from 1
        id = (1:numel(list))';
        
        % Create a table with columns for 'id', 'Name of list', 'Number of states nx', 
        % 'Number of inputs nu', and 'Number of outputs ny'
        T = table(id, string(modified_list), prop(:,1), prop(:,2), prop(:,3), ...
            'VariableNames', {'id', 'Name_of_list', 'Number_of_states_nx', ...
            'Number_of_inputs_nu', 'Number_of_outputs_ny'});
        
        obj.list=T;
% --------------------------------- Filter --------------------------------
        if filterOption
        % Display the table
        disp(T);

        T0=T;
        options = {'States', 'Inputs', 'Outputs'};

    while true
        % Prompt the user to choose what to filter
        disp('Choose what to filter:');
        for i = 1:length(options)
            disp([lower(options{i}(1)) ' - ' options{i}]);
        end
      
    disp('If you want to finish, please<strong> press Enter to exit.</strong>');

    % Construct the input prompt dynamically
    prompt = 'Enter your choice (';
    for i = 1:length(options)
        prompt = [prompt lower(options{i}(1))];
        if i < length(options)
            prompt = [prompt ' for ' options{i} ', '];
        else
            prompt = [prompt ' for ' options{i} '): '];
        end
    end
    
    % Get user input
    choice = lower(input(prompt, 's'));

    % Validate the user input
    if ~ismember(choice, {'s', 'i', 'o'})
        choice = input('Invalid choice. Do you want to try again? (yes/no): ', 's');
        if strcmpi(choice, 'yes')
            continue; % Restart the loop
        else
            disp('Exiting...');
            return;
        end
    end

    % Depending on the user's choice, prompt for the desired number and filter accordingly
    switch choice
        %case 1
        case 's'
  
     disp('States filtering...');
    % Loop until a valid input is provided or the user chooses to exit
    while true
    filtered_table = filter_table_by_number(T0, 'Number_of_states_nx', 'states nx');

    choice = input('Choose an option (from 1 to 4):\n1 - Filter states again\n2 - Return to beginning\n3 - The next round of filtering\n4 - Exit\nEnter your choice: ', 's');
    
    % Validate the user input
    if strcmpi(choice, '4')
        % Exit the loop if the user chooses to exit
        disp('Exiting...');
        return;
    elseif strcmpi(choice, '2')
        % Return to the beginning if the user chooses to start over
        disp('Returning to the beginning...');
        options = {'States', 'Inputs', 'Outputs'};
        T0=T;
        break;
    elseif strcmpi(choice, '1')
        % Continue filtering states
    elseif strcmpi(choice, '3')
       % Update options for the next round
     options = {'Inputs', 'Outputs'};
     T0 = filtered_table;
       break;
    else
        disp('Invalid choice.');
        return;
    end
    end

          %case 2
          case 'i'
    
      disp('Inputs filtering...');
    % Loop until a valid input is provided or the user chooses to exit
    while true
    filtered_table = filter_table_by_number(T0, 'Number_of_inputs_nu', 'inputs nu');

    choice = input('Choose an option (from 1 to 4):\n1 - Filter inputs again\n2 - Return to beginning\n3 - The next round of filtering\n4 - Exit\nEnter your choice: ', 's');
    % Validate the user input
    if strcmpi(choice, '4')
        % Exit the loop if the user chooses to exit
        disp('Exiting...');
        return;
    elseif strcmpi(choice, '2')
        % Return to the beginning if the user chooses to start over
        disp('Returning to the beginning...');
        options = {'States', 'Inputs', 'Outputs'};
        T0=T;
        break;
    elseif strcmpi(choice, '1')
        % Continue filtering inputs
    elseif strcmpi(choice, '3')
      % Update options for the next round
     options = {'States', 'Outputs'};
     T0 = filtered_table;
       break;
    else
        disp('Invalid choice.');
        return;
    end
    end

          %case 3
          case 'o'

    disp('Outputs filtering...');
    % Loop until a valid input is provided or the user chooses to exit
    while true
    filtered_table = filter_table_by_number(T0, 'Number_of_outputs_ny', 'outputs ny');

    choice = input('Choose an option (from 1 to 4):\n1 - Filter outputs again\n2 - Return to beginning\n3 - The next round of filtering\n4 - Exit\nEnter your choice: ', 's');
    % Validate the user input
    if strcmpi(choice, '4')
        % Exit the loop if the user chooses to exit
        disp('Exiting...');
        return;
    elseif strcmpi(choice, '2')
        % Return to the beginning if the user chooses to start over
        disp('Returning to the beginning...');
        options = {'States', 'Inputs', 'Outputs'};
        T0=T;
        break;
    elseif strcmpi(choice, '1')
        % Continue filtering outputs
    elseif strcmpi(choice, '3')
    % Update options for the next round
    options = {'States', 'Inputs'};
     T0 = filtered_table;
       break;
    else
        disp('Invalid choice.');
        return;
    end
        end
        end
        
        end
                else
                    filtered_table = T;
                end
        end
        
    end
    end 

        