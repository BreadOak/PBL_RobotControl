function callback_PositionMode(message)    
    global PositionMode;
    PositionMode = message.data;
end