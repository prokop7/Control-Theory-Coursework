function sm_cart_dpen_linearize
%SM_CART_DPEN_LINEARIZE - Sets the model to be open loop and generates the
%                         linearized model. The linearized model is put in
%                         the base workspace as the variable sys_cart_dpen.

% Copyright 2011-2014 The MathWorks, Inc.

mdl = 'sm_cart_double_pendulum';
swtch = [mdl,'/Open Loop Switch'];
ctrlr = [mdl '/Controller'];
plant = [gcs '/Cart and Double Pendulum'];

% Operating point : 

% Save initial conditions
init.q1 = get_param(plant, 'q1');
init.w1 = get_param(plant, 'w1');
init.q2 = get_param(plant, 'q2');
init.w2 = get_param(plant, 'w2');

% Save visualization setting
vizSetting = get_param(gcs,'SimMechanicsOpenEditorOnUpdate');

% Set up model to be reset after linearization at operating point
clnup = onCleanup(@()cleanUp(mdl,swtch,ctrlr,vizSetting,plant,init));

% Turn off viz for linearization
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');

% Set targets to linearization operating point
% Operating Point - Inverted upright position
set_param(plant, 'q1', '0.0');
set_param(plant, 'w1', '0.0');
set_param(plant, 'q2', '0.0');
set_param(plant, 'w2', '0.0');

% Make the system open loop
set_param(swtch,'sw','1');
set_param(ctrlr,'sys','[]');

% Get full state corresponding to operating point
[~,X,~] = sim(mdl,0); 

% Linearizing about inverted upright position
[A,B,C,D] = linmod(mdl,X,0);

% Removing the discrete states from the state space model
A = A(1:6,1:6);
B = B(1:6);
C = C(:,1:6);
sys_cart_dpen.A = A;
sys_cart_dpen.B = B;
sys_cart_dpen.C = C;
sys_cart_dpen.D = D;
assignin('base','sys_cart_dpen',sys_cart_dpen);

end

function cleanUp(mdl, swtch, ctrlr, vizSetting, plant, init)

% Make the system closed loop
set_param(swtch,'sw','0');
set_param(ctrlr,'sys','sys_cart_dpen');

% Set viz setting to what it was
set_param(mdl,'SimMechanicsOpenEditorOnUpdate',vizSetting);

set_param(mdl,'Dirty','off');

% Setting initial conditions back
set_param(plant, 'q1', init.q1);
set_param(plant, 'w1', init.w1);
set_param(plant, 'q2', init.q2);
set_param(plant, 'w2', init.w2);

end
