function simpleGUI
    % Create the main figure
    hFig = figure('Position', [100, 100, 450, 300], 'MenuBar', 'none', 'Name', 'Simple GUI', 'NumberTitle', 'off', 'Resize', 'off');
    
    % Create four sliders
    slider1 = uicontrol('Style', 'slider', 'Min', 1000, 'Max', 2000, 'Value', 1500, 'Position', [50, 220, 300, 20]);
    slider2 = uicontrol('Style', 'slider', 'Min', 1000, 'Max', 2000, 'Value', 1500, 'Position', [50, 180, 300, 20]);
    slider3 = uicontrol('Style', 'slider', 'Min', 1000, 'Max', 2000, 'Value', 1000, 'Position', [50, 140, 300, 20]);
    slider4 = uicontrol('Style', 'slider', 'Min', 1000, 'Max', 2000, 'Value', 1500, 'Position', [50, 100, 300, 20]);

    % Create two switches (checkboxes)
    switch1 = uicontrol('Style', 'checkbox', 'String', 'Arm', 'Position', [50, 60, 100, 30]);
    switch2 = uicontrol('Style', 'checkbox', 'String', 'Switch 2', 'Position', [200, 60, 100, 30]);

    % Create text labels to show the slider values
    slider1Label = uicontrol('Style', 'text', 'Position', [360, 220, 50, 30], 'String', num2str(get(slider1, 'Value')));
    slider2Label = uicontrol('Style', 'text', 'Position', [360, 180, 50, 30], 'String', num2str(get(slider2, 'Value')));
    slider3Label = uicontrol('Style', 'text', 'Position', [360, 140, 50, 30], 'String', num2str(get(slider3, 'Value')));
    slider4Label = uicontrol('Style', 'text', 'Position', [360, 100, 50, 30], 'String', num2str(get(slider4, 'Value')));
    
    % Set callback functions for the sliders
    slider1.Callback = @(src, event) updateSliderLabel(src, slider1Label);
    slider2.Callback = @(src, event) updateSliderLabel(src, slider2Label);
    slider3.Callback = @(src, event) updateSliderLabel(src, slider3Label);
    slider4.Callback = @(src, event) updateSliderLabel(src, slider4Label);
    
    % Set callback functions for the switches
    switch1.Callback = @(src, event) switchCallback(src, 'Arm');
    switch2.Callback = @(src, event) switchCallback(src, 'Mode');
end

% Callback function to update slider labels
function updateSliderLabel(slider, label)
    label.String = num2str(get(slider, 'Value'));
end

% Callback function for switches
function switchCallback(switchObj, switchName)
    if get(switchObj, 'Value')
        disp([switchName ' is ON']);
    else
        disp([switchName ' is OFF']);
    end
end
