function controller = constantController(control)
    %CONSTANTCONTROLLER
    %
    % controller = CONSTANTCONTROLLER(control)
    %
    % control    -
    %
    % controller -
    
    controller = @(t,state) control;
end