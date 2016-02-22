function controller = constantController(control)
    controller = @(t,state) control;
end