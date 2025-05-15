function filtered_table = filter_table_by_number(T, column, label)
    
    % Prompt the user to choose the comparison operator
    operators = {'>=', '<=', '=', '>', '<'};
    disp(['Enter the desired number of ', label, ' (choose one option from 1 to 5):']);
    for i = 1:length(operators)
        fprintf('%d: %s %s\n', i, label(end-1:end), operators{i});
    end
    choice = input('Enter your choice: ');

    % Validate the user input
    if ~ismember(choice, 1:length(operators))
         disp('Invalid choice. Please enter a number from 1 to 5.');
    % Prompt the user to choose again
        disp(['Enter the desired number of ', label, ' (choose one option from 1 to 5):']);
            for i = 1:length(operators)
                fprintf('%d: %s %s\n', i, label(end-1:end), operators{i});
            end
        choice = input('Enter your choice: ');
    end

    % Extract the selected comparison operator
    operator = operators{choice};

    % Prompt the user to enter the desired number of states
    fprintf(['Enter the desired number of ', label, ' (more than 0): ']);
    number = input('');

    % Validate the user input
    if ~isnumeric(number) || number <= 0 || mod(number, 1) ~= 0
        disp('Invalid input. Please enter a positive whole number.');
        return;
    end

    % Filter the table based on the user's input
    switch operator
        case '>='
            filtered_table = T(T.(column) >= number, :);
        case '<='
            filtered_table = T(T.(column) <= number, :);
        case '='
            filtered_table = T(T.(column) == number, :);
        case '>'
            filtered_table = T(T.(column) > number, :);
        case '<'
            filtered_table = T(T.(column) < number, :);
    end

    % Check if any model exists with the specified number of states
    if isempty(filtered_table)
        disp(['Sorry, but no model with the specified number of ', label, ' exists.']);
    else
        % Display the filtered table
        disp(filtered_table);
    end
end
