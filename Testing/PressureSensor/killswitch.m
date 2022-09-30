clear;
clc;

b = ble("C02874363E30"); %C0285B324333
joy_c = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00008000-0001-11E1-AC36-0002A5D5C51B")

% Attempt shutting drone down for 1 seconds
for i=1:100
    write(joy_c, [0, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse")
    java.lang.Thread.sleep(10);
end