package MyLib
  extends Modelica.Icons.Package;

  package Fluid
    extends Modelica.Icons.Package;

    package Assemblies
      extends Modelica.Icons.Package;

      model ETS
        outer Modelica.Fluid.System system annotation(
          Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        replaceable package PrimaryMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
        replaceable package SecondaryMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
        replaceable model HexHeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer constrainedby Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.PartialFlowHeatTransfer;
        replaceable model HexFlowModel = Modelica.Fluid.Pipes.BaseClasses.FlowModels.DetailedPipeFlow constrainedby Modelica.Fluid.Pipes.BaseClasses.FlowModels.PartialStaggeredFlowModel;
        replaceable function etsValveCharacteristic = Modelica.Fluid.Valves.BaseClasses.ValveCharacteristics.linear constrainedby Modelica.Fluid.Valves.BaseClasses.ValveCharacteristics.baseFun;
        parameter Integer hexChannels annotation(
          Dialog(tab = "General", group = "Geometry"));
        parameter Modelica.SIunits.Diameter hexChannelDiameter(displayUnit = "mm") annotation(
          Dialog(tab = "General", group = "Geometry"));
        parameter Modelica.SIunits.Length hexChannelLength annotation(
          Dialog(tab = "General", group = "Geometry"));
        parameter Modelica.SIunits.CoefficientOfHeatTransfer hexAlpha = 4000 annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Modelica.SIunits.ThermalConductivity hexWallConductivity annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Modelica.SIunits.Length hexWallThickness(displayUnit = "mm") annotation(
          Dialog(tab = "General", group = "Geometry"));
        parameter Modelica.SIunits.PressureDifference valve_dp_nom(displayUnit = "Pa") annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Modelica.SIunits.MassFlowRate valve_flow_nom annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Modelica.SIunits.Volume pumpVolume annotation(
          Dialog(tab = "General", group = "Geometry"));
        parameter Modelica.SIunits.MassFlowRate pump_flow_nom annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Modelica.SIunits.AbsolutePressure pumpInlet_press_nom(displayUnit = "Pa") annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Modelica.SIunits.AbsolutePressure pumpOutlet_press_nom(displayUnit = "Pa") annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Real PID_Gain = 0.5 annotation(
          Dialog(tab = "Advanced", group = "System Behaviour"));
        parameter Real PID_Ti = 120 annotation(
          Dialog(tab = "Advanced", group = "System Behaviour"));
        parameter Real PID_Td = 0 annotation(
          Dialog(tab = "Advanced", group = "System Behaviour"));
        Modelica.Fluid.Pipes.DynamicPipe primaryPipe(redeclare replaceable package Medium = PrimaryMedium, redeclare replaceable model FlowModel = HexFlowModel, redeclare replaceable model HeatTransfer = HexHeatTransfer, diameter = hexChannelDiameter, length = hexChannelLength, heatTransfer(alpha0 = hexAlpha), modelStructure = Modelica.Fluid.Types.ModelStructure.a_v_b, nParallel = hexChannels, use_HeatTransfer = true) annotation(
          Placement(visible = true, transformation(origin = {-30, -20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Fluid.Pipes.DynamicPipe secondaryPipe(redeclare replaceable package Medium = SecondaryMedium, redeclare replaceable model FlowModel = HexFlowModel, redeclare replaceable model HeatTransfer = HexHeatTransfer, diameter = hexChannelDiameter, length = hexChannelLength, heatTransfer(alpha0 = hexAlpha), modelStructure = Modelica.Fluid.Types.ModelStructure.a_v_b, nParallel = hexChannels, use_HeatTransfer = true) annotation(
          Placement(visible = true, transformation(origin = {30, -20}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
        Modelica.Thermal.HeatTransfer.Components.ThermalConductor hexMaterial(G = hexWallConductivity * hexWallThickness, port_a.T.start = system.T_start, port_b.T.start = system.T_start) annotation(
          Placement(visible = true, transformation(origin = {0, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Fluid.Valves.ValveIncompressible valve(redeclare replaceable package Medium = PrimaryMedium, redeclare replaceable function valveCharacteristic = etsValveCharacteristic, CvData = Modelica.Fluid.Types.CvTypes.OpPoint, dp_nominal(displayUnit = "Pa") = valve_dp_nom, dp(displayUnit = "Pa"), m_flow_nominal = valve_flow_nom, dp_start = 0, m_flow_start = system.m_flow_start) annotation(
          Placement(visible = true, transformation(extent = {{-50, -80}, {-70, -60}}, rotation = 0)));
        Modelica.Fluid.Machines.ControlledPump pump(redeclare replaceable package Medium = SecondaryMedium, V = pumpVolume, checkValve = true, energyDynamics = system.energyDynamics, m_flow_nominal = pump_flow_nom, massDynamics = system.massDynamics, p_a_nominal = pumpInlet_press_nom, p_b_nominal = pumpOutlet_press_nom, use_m_flow_set = true) annotation(
          Placement(visible = true, transformation(origin = {60, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Fluid.Interfaces.FluidPort_a primaryHWS(redeclare replaceable package Medium = PrimaryMedium, m_flow(start = system.m_flow_start), p(start = system.p_start)) annotation(
          Placement(visible = true, transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Fluid.Interfaces.FluidPort_b primaryHWR(redeclare replaceable package Medium = PrimaryMedium, m_flow(start = system.m_flow_start), p(start = system.p_start)) annotation(
          Placement(visible = true, transformation(origin = {-90, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Fluid.Interfaces.FluidPort_b secondaryHWS(redeclare replaceable package Medium = SecondaryMedium, m_flow(start = system.m_flow_start), p(start = system.p_start)) annotation(
          Placement(visible = true, transformation(origin = {90, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Fluid.Interfaces.FluidPort_a secondaryHWR(redeclare replaceable package Medium = SecondaryMedium, m_flow(start = system.m_flow_start), p(start = system.p_start)) annotation(
          Placement(visible = true, transformation(origin = {90, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.LimPID TC_PID(Td = PID_Td, Ti = PID_Ti, initType = Modelica.Blocks.Types.InitPID.InitialState, k = PID_Gain, yMax = 100, yMin = 0) annotation(
          Placement(visible = true, transformation(origin = {0, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput HWS_temp_SP annotation(
          Placement(visible = true, transformation(origin = {90, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, 80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput pumpMassFlowIn annotation(
          Placement(visible = true, transformation(origin = {90, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Fluid.Sensors.TemperatureTwoPort secondaryHWS_TE(redeclare replaceable package Medium = SecondaryMedium) annotation(
          Placement(visible = true, transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(secondaryHWS_TE.T, TC_PID.u_m) annotation(
          Line(points = {{50, 42}, {0, 42}, {0, 58}, {0, 58}}, color = {0, 0, 127}));
        connect(secondaryPipe.port_b, secondaryHWS_TE.port_a) annotation(
          Line(points = {{30, -10}, {30, 30}, {40, 30}}, color = {0, 127, 255}));
        connect(secondaryHWS_TE.port_b, secondaryHWS) annotation(
          Line(points = {{60, 30}, {90, 30}, {90, 30}, {90, 30}}, color = {0, 127, 255}));
        connect(secondaryHWR, pump.port_a) annotation(
          Line(points = {{90, -70}, {70, -70}}));
        connect(pump.port_b, secondaryPipe.port_a) annotation(
          Line(points = {{50, -70}, {30, -70}, {30, -30}}, color = {0, 127, 255}));
        connect(hexMaterial.port_b, secondaryPipe.heatPorts[1]) annotation(
          Line(points = {{10, -20}, {26, -20}}, color = {191, 0, 0}));
        connect(primaryPipe.heatPorts[1], hexMaterial.port_a) annotation(
          Line(points = {{-25.6, -20.1}, {-9.6, -20.1}}, color = {127, 0, 0}));
        connect(primaryHWS, primaryPipe.port_a) annotation(
          Line(points = {{-90, 30}, {-30, 30}, {-30, -10}}));
        connect(primaryPipe.port_b, valve.port_a) annotation(
          Line(points = {{-30, -30}, {-30, -70}, {-50, -70}}, color = {0, 127, 255}));
        connect(valve.port_b, primaryHWR) annotation(
          Line(points = {{-70, -70}, {-90, -70}}, color = {0, 127, 255}));
        connect(pumpMassFlowIn, pump.m_flow_set) annotation(
          Line(points = {{90, -30}, {64, -30}, {64, -62}, {66, -62}}, color = {0, 0, 127}));
        connect(HWS_temp_SP, TC_PID.u_s) annotation(
          Line(points = {{90, 70}, {12, 70}}, color = {0, 0, 127}));
        connect(TC_PID.y, valve.opening) annotation(
          Line(points = {{-11, 70}, {-11, 70.5}, {-60, 70.5}, {-60, -62}}, color = {0, 0, 127}));
// ---------- Graphics & Layout ----------
        annotation(
          Icon(graphics = {Rectangle(origin = {-19, 20}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}), Line(origin = {5.27, 20}, points = {{74.7293, 32}, {-55.2707, 32}, {6.72932, 0}, {-55.2707, -32}, {74.7293, -32}}, thickness = 1), Line(origin = {-24.27, -52}, points = {{-3.72703, 12}, {12.273, 12}, {-3.72703, -12}, {12.273, -12}, {-3.72703, 12}}, thickness = 1), Line(origin = {-20, -32}, points = {{0, 12}, {0, -8}}, thickness = 1), Line(origin = {-60, -72}, points = {{40, 8}, {40, -8}, {-20, -8}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {-60, 68}, points = {{40, -8}, {40, 12}, {-20, 12}}, thickness = 1, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 10), Ellipse(origin = {60, 80}, lineThickness = 1, extent = {{-15, 15}, {15, -15}}, endAngle = 360), Line(origin = {60, 56}, points = {{0, 8}, {0, -4}}, thickness = 1), Text(origin = {60, 80}, lineThickness = 1, extent = {{-10, 10}, {10, -10}}, textString = "TC", fontName = "DejaVu Sans Mono"), Ellipse(origin = {50, -12}, lineThickness = 1, extent = {{-15, 15}, {15, -15}}, endAngle = 360), Line(origin = {45.33, -12}, points = {{4.67075, -15}, {-9.32925, 0}, {4.67075, 15}}, thickness = 1), Text(origin = {0, 125}, lineColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", fontName = "DejaVu Sans Mono")}, coordinateSystem(initialScale = 0.1)),
          uses(Modelica(version = "3.2.1")),
          Diagram,
          version = "");
      end ETS;

      model ETS2
        outer Modelica.Fluid.System system annotation(
          Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        replaceable package PrimaryMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
        replaceable package SecondaryMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
        replaceable model HexHeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer constrainedby Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.PartialFlowHeatTransfer;
        replaceable model HexFlowModel = Modelica.Fluid.Pipes.BaseClasses.FlowModels.DetailedPipeFlow constrainedby Modelica.Fluid.Pipes.BaseClasses.FlowModels.PartialStaggeredFlowModel;
        replaceable function etsValveCharacteristic = Modelica.Fluid.Valves.BaseClasses.ValveCharacteristics.linear constrainedby Modelica.Fluid.Valves.BaseClasses.ValveCharacteristics.baseFun;
        parameter Integer hexChannels annotation(
          Dialog(tab = "General", group = "Geometry"));
        parameter Modelica.SIunits.Diameter hexChannelDiameter(displayUnit = "mm") annotation(
          Dialog(tab = "General", group = "Geometry"));
        parameter Modelica.SIunits.Length hexChannelLength annotation(
          Dialog(tab = "General", group = "Geometry"));
        parameter Modelica.SIunits.CoefficientOfHeatTransfer hexAlpha = 4000 annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Modelica.SIunits.ThermalConductivity hexWallConductivity annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Modelica.SIunits.Length hexWallThickness(displayUnit = "mm") annotation(
          Dialog(tab = "General", group = "Geometry"));
        parameter Modelica.SIunits.PressureDifference valve_dp_nom(displayUnit = "Pa") annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Modelica.SIunits.MassFlowRate valve_flow_nom annotation(
          Dialog(tab = "Advanced", group = "Characteristics"));
        parameter Real PID_Gain = 0.5 annotation(
          Dialog(tab = "Advanced", group = "System Behaviour"));
        parameter Real PID_Ti = 120 annotation(
          Dialog(tab = "Advanced", group = "System Behaviour"));
        parameter Real PID_Td = 0 annotation(
          Dialog(tab = "Advanced", group = "System Behaviour"));
        Modelica.Fluid.Pipes.DynamicPipe primaryPipe(redeclare replaceable package Medium = PrimaryMedium, redeclare replaceable model FlowModel = HexFlowModel, redeclare replaceable model HeatTransfer = HexHeatTransfer, diameter = hexChannelDiameter, heatTransfer(alpha0 = hexAlpha), length = hexChannelLength, modelStructure = Modelica.Fluid.Types.ModelStructure.a_v_b, nParallel = hexChannels, use_HeatTransfer = true) annotation(
          Placement(visible = true, transformation(origin = {-30, -20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Fluid.Pipes.DynamicPipe secondaryPipe(redeclare replaceable package Medium = SecondaryMedium, redeclare replaceable model FlowModel = HexFlowModel, redeclare replaceable model HeatTransfer = HexHeatTransfer, diameter = hexChannelDiameter, length = hexChannelLength, heatTransfer(alpha0 = hexAlpha), modelStructure = Modelica.Fluid.Types.ModelStructure.a_v_b, nParallel = hexChannels, use_HeatTransfer = true) annotation(
          Placement(visible = true, transformation(origin = {30, -20}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
        Modelica.Thermal.HeatTransfer.Components.ThermalConductor hexMaterial(G = hexWallConductivity * hexWallThickness, port_a.T.start = system.T_start, port_b.T.start = system.T_start) annotation(
          Placement(visible = true, transformation(origin = {0, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Fluid.Valves.ValveIncompressible valve(redeclare replaceable package Medium = PrimaryMedium, redeclare replaceable function valveCharacteristic = etsValveCharacteristic, CvData = Modelica.Fluid.Types.CvTypes.OpPoint, dp_nominal(displayUnit = "Pa") = valve_dp_nom, dp(displayUnit = "Pa"), m_flow_nominal = valve_flow_nom, dp_start = 0, m_flow_start = system.m_flow_start) annotation(
          Placement(visible = true, transformation(extent = {{-50, -80}, {-70, -60}}, rotation = 0)));
        Modelica.Fluid.Interfaces.FluidPort_a primaryHWS(redeclare replaceable package Medium = PrimaryMedium, m_flow(start = system.m_flow_start), p(start = system.p_start)) annotation(
          Placement(visible = true, transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Fluid.Interfaces.FluidPort_b primaryHWR(redeclare replaceable package Medium = PrimaryMedium, m_flow(start = system.m_flow_start), p(start = system.p_start)) annotation(
          Placement(visible = true, transformation(origin = {-90, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Fluid.Interfaces.FluidPort_b secondaryHWS(redeclare replaceable package Medium = SecondaryMedium, m_flow(start = system.m_flow_start), p(start = system.p_start)) annotation(
          Placement(visible = true, transformation(origin = {90, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Fluid.Interfaces.FluidPort_a secondaryHWR(redeclare replaceable package Medium = SecondaryMedium, m_flow(start = system.m_flow_start), p(start = system.p_start)) annotation(
          Placement(visible = true, transformation(origin = {90, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.LimPID TC_PID(Td = PID_Td, Ti = PID_Ti, initType = Modelica.Blocks.Types.InitPID.InitialState, k = PID_Gain, yMax = 100, yMin = 0) annotation(
          Placement(visible = true, transformation(origin = {0, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput HWS_temp_SP annotation(
          Placement(visible = true, transformation(origin = {90, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, 80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Fluid.Sensors.TemperatureTwoPort secondaryHWS_TE(redeclare replaceable package Medium = SecondaryMedium) annotation(
          Placement(visible = true, transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(secondaryHWR, secondaryPipe.port_a) annotation(
          Line(points = {{90, -70}, {30, -70}, {30, -30}, {30, -30}}));
        connect(secondaryHWS_TE.T, TC_PID.u_m) annotation(
          Line(points = {{50, 42}, {0, 42}, {0, 58}, {0, 58}}, color = {0, 0, 127}));
        connect(secondaryPipe.port_b, secondaryHWS_TE.port_a) annotation(
          Line(points = {{30, -10}, {30, 30}, {40, 30}}, color = {0, 127, 255}));
        connect(secondaryHWS_TE.port_b, secondaryHWS) annotation(
          Line(points = {{60, 30}, {90, 30}, {90, 30}, {90, 30}}, color = {0, 127, 255}));
        connect(hexMaterial.port_b, secondaryPipe.heatPorts[1]) annotation(
          Line(points = {{10, -20}, {26, -20}}, color = {191, 0, 0}));
        connect(primaryPipe.heatPorts[1], hexMaterial.port_a) annotation(
          Line(points = {{-25.6, -20.1}, {-9.6, -20.1}}, color = {127, 0, 0}));
        connect(primaryHWS, primaryPipe.port_a) annotation(
          Line(points = {{-90, 30}, {-30, 30}, {-30, -10}}));
        connect(primaryPipe.port_b, valve.port_a) annotation(
          Line(points = {{-30, -30}, {-30, -70}, {-50, -70}}, color = {0, 127, 255}));
        connect(valve.port_b, primaryHWR) annotation(
          Line(points = {{-70, -70}, {-90, -70}}, color = {0, 127, 255}));
        connect(HWS_temp_SP, TC_PID.u_s) annotation(
          Line(points = {{90, 70}, {12, 70}}, color = {0, 0, 127}));
        connect(TC_PID.y, valve.opening) annotation(
          Line(points = {{-11, 70}, {-11, 70.5}, {-60, 70.5}, {-60, -62}}, color = {0, 0, 127}));
// ---------- Graphics & Layout ----------
        annotation(
          Icon(graphics = {Rectangle(origin = {-19, 20}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}), Line(origin = {5.27, 20}, points = {{74.7293, 32}, {-55.2707, 32}, {6.72932, 0}, {-55.2707, -32}, {74.7293, -32}}, thickness = 1), Line(origin = {-24.27, -52}, points = {{-3.72703, 12}, {12.273, 12}, {-3.72703, -12}, {12.273, -12}, {-3.72703, 12}}, thickness = 1), Line(origin = {-20, -32}, points = {{0, 12}, {0, -8}}, thickness = 1), Line(origin = {-60, -72}, points = {{40, 8}, {40, -8}, {-20, -8}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {-60, 68}, points = {{40, -8}, {40, 12}, {-20, 12}}, thickness = 1, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 10), Ellipse(origin = {60, 80}, lineThickness = 1, extent = {{-15, 15}, {15, -15}}, endAngle = 360), Line(origin = {60, 56}, points = {{0, 8}, {0, -4}}, thickness = 1), Text(origin = {60, 80}, lineThickness = 1, extent = {{-10, 10}, {10, -10}}, textString = "TC", fontName = "DejaVu Sans Mono"), Text(origin = {0, 125}, lineColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", fontName = "DejaVu Sans Mono")}, coordinateSystem(initialScale = 0.1)),
          uses(Modelica(version = "3.2.2")),
          Diagram,
          version = "");
      end ETS2;

      model SolenoidPump
        extends Modelica.Fluid.Interfaces.PartialTwoPort(final allowFlowReversal = false);
        replaceable package Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium "Medium in the component" annotation(
           choicesAllMatching = true);
        parameter Modelica.SIunits.Diameter portDia "Internal diameter of fluidPorts" annotation(
          Dialog(group = "Fluid"));
        parameter Modelica.SIunits.Mass pistonMass "Mass of pump piston" annotation(
          Dialog(group = "Mechanics"));
        parameter Modelica.SIunits.Length pistonLength "Length of pump piston" annotation(
          Dialog(group = "Mechanics"));
        parameter Modelica.SIunits.Area pistonCrossArea "Active surface area of pump piston" annotation(
          Dialog(group = "Mechanics"));
        parameter Modelica.SIunits.Length springLength "Length of unstreched pump spring" annotation(
          Dialog(group = "Mechanics"));
        parameter Modelica.SIunits.TranslationalSpringConstant c_spring "Pump spring constant" annotation(
          Dialog(group = "Mechanics"));
        parameter Modelica.SIunits.TranslationalDampingConstant d_spring = 0.1 * c_spring "Pump spring constant" annotation(
          Dialog(group = "Mechanics"));
        parameter Modelica.SIunits.Resistance R_Resistor "Resistance of solenoid coil" annotation(
          Dialog(group = "Electrical"));
        parameter Real k_EMF "Transformation coeff. (k=Voltage/Velocity=-1*Force/Current)" annotation(
          Dialog(group = "Electrical"));
        parameter Modelica.Fluid.Types.Dynamics energyDynamics = system.energyDynamics "Formulation of energy balances" annotation(
          Evaluate = true,
          Dialog(tab = "Assumptions", group = "Dynamics"));
        parameter Modelica.Fluid.Types.Dynamics massDynamics = system.massDynamics "Formulation of mass balances" annotation(
          Evaluate = true,
          Dialog(tab = "Assumptions", group = "Dynamics"));
        MyLib.Fluid.Valves.CheckValve inletValve(redeclare replaceable package Medium = Medium, diameter = portDia, zeta = 1) annotation(
          Placement(visible = true, transformation(origin = {-30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Fluid.Machines.SweptVolume pumpBody(redeclare replaceable package Medium = Medium, clearance = pistonCrossArea * portDia, energyDynamics = energyDynamics, massDynamics = massDynamics, nPorts = 2, pistonCrossArea = pistonCrossArea, portsData = {Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter = portDia), Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter = portDia)}) annotation(
          Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        MyLib.Fluid.Valves.CheckValve outletValve(redeclare replaceable package Medium = Medium, diameter = portDia, zeta = 1) annotation(
          Placement(visible = true, transformation(origin = {30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.Translational.Components.Mass pumpPiston(L = pistonLength, m = pistonMass, s(fixed = true, start = springLength + pistonLength / 2), v(fixed = true)) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Mechanics.Translational.Components.SpringDamper pistonChamber(c = c_spring, d = d_spring, s_rel(fixed = false, start = springLength), s_rel0 = springLength, v_rel(fixed = false)) annotation(
          Placement(visible = true, transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Mechanics.Translational.Components.Fixed housing annotation(
          Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Electrical.Analog.Interfaces.PositivePin p annotation(
          Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Electrical.Analog.Basic.TranslationalEMF solenoidCoil(k = k_EMF, useSupport = true) annotation(
          Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
        Modelica.Electrical.Analog.Basic.Resistor coilResistance(R = R_Resistor, T = system.T_ambient) annotation(
          Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Electrical.Analog.Interfaces.NegativePin n annotation(
          Placement(visible = true, transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(solenoidCoil.n, n) annotation(
          Line(points = {{10, 60}, {100, 60}, {100, 60}, {100, 60}}, color = {0, 0, 255}));
        connect(p, coilResistance.p) annotation(
          Line(points = {{-100, 60}, {-70, 60}, {-70, 60}, {-70, 60}}, color = {0, 0, 255}));
        connect(port_a, inletValve.port_a) annotation(
          Line(points = {{-100, 0}, {-80, 0}, {-80, -60}, {-40, -60}, {-40, -60}}, color = {0, 127, 255}));
        connect(outletValve.port_b, port_b) annotation(
          Line(points = {{40, -60}, {80, -60}, {80, 0}, {100, 0}, {100, 0}}, color = {0, 127, 255}));
        connect(housing.flange, pistonChamber.flange_a) annotation(
          Line(points = {{0, 90}, {0, 80}, {30, 80}, {30, 50}}, color = {0, 127, 0}));
        connect(coilResistance.n, solenoidCoil.p) annotation(
          Line(points = {{-50, 60}, {-10, 60}}, color = {0, 0, 255}));
        connect(solenoidCoil.flange, pumpPiston.flange_a) annotation(
          Line(points = {{0, 50}, {0, 10}}, color = {0, 127, 0}));
        connect(solenoidCoil.support, housing.flange) annotation(
          Line(points = {{0, 70}, {0, 90}}, color = {0, 127, 0}));
        connect(pistonChamber.flange_b, pumpPiston.flange_a) annotation(
          Line(points = {{30, 30}, {30, 20}, {0, 20}, {0, 10}}, color = {0, 127, 0}));
        connect(pumpPiston.flange_b, pumpBody.flange) annotation(
          Line(points = {{0, -10}, {0, -20}}, color = {0, 127, 0}));
        connect(inletValve.port_b, pumpBody.ports[1]) annotation(
          Line(points = {{-20, -60}, {0, -60}, {0, -38}}, color = {0, 127, 255}));
        connect(pumpBody.ports[2], outletValve.port_a) annotation(
          Line(points = {{0, -38}, {0, -60}, {20, -60}}, color = {0, 127, 255}, thickness = 0.5));
        annotation(
          Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {170, 213, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-50, 36}, {50, -58}}), Polygon(lineColor = {95, 95, 95}, fillColor = {135, 135, 135}, fillPattern = FillPattern.Backward, points = {{-52, 62}, {-48, 62}, {-48, -30}, {-52, -30}, {-52, 62}}), Polygon(lineColor = {95, 95, 95}, fillColor = {135, 135, 135}, fillPattern = FillPattern.Backward, points = {{48, 60}, {52, 60}, {52, -34}, {48, -34}, {48, 60}}), Rectangle(lineColor = {95, 95, 95}, fillColor = {135, 135, 135}, fillPattern = FillPattern.Forward, extent = {{-48, 40}, {48, 30}}), Rectangle(lineColor = {95, 95, 95}, fillColor = {135, 135, 135}, fillPattern = FillPattern.Forward, extent = {{-6, 92}, {6, 40}}), Polygon(lineColor = {95, 95, 95}, fillColor = {135, 135, 135}, fillPattern = FillPattern.Backward, points = {{-48, -56}, {48, -56}, {48, 70}, {52, 70}, {52, -60}, {-52, -60}, {-52, 70}, {-48, 70}, {-48, -56}}), Line(visible = false, points = {{-100, 0}, {-52, 0}}, color = {198, 0, 0}), Ellipse(origin = {-20, 86}, fillColor = {170, 85, 0}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-6, 6}, {6, -6}}, endAngle = 360), Ellipse(origin = {20, 86}, fillColor = {170, 85, 0}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-6, 6}, {6, -6}}, endAngle = 360), Ellipse(origin = {-20, 74}, fillColor = {170, 85, 0}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-6, 6}, {6, -6}}, endAngle = 360), Ellipse(origin = {20, 74}, fillColor = {170, 85, 0}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-6, 6}, {6, -6}}, endAngle = 360), Ellipse(origin = {-20, 62}, fillColor = {170, 85, 0}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-6, 6}, {6, -6}}, endAngle = 360), Ellipse(origin = {20, 62}, fillColor = {170, 85, 0}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-6, 6}, {6, -6}}, endAngle = 360), Ellipse(origin = {-20, 50}, fillColor = {170, 85, 0}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-6, 6}, {6, -6}}, endAngle = 360), Ellipse(origin = {20, 50}, fillColor = {170, 85, 0}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-6, 6}, {6, -6}}, endAngle = 360), Ellipse(origin = {-88, -40}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-4, 4}, {4, -4}}, endAngle = 360), Line(origin = {-73.8154, -49.5693}, points = {{-14, -10}, {-14, 10}, {14, -10}, {14, 10}}, thickness = 0.5), Line(origin = {74.1385, -49.2616}, points = {{-14, -10}, {-14, 10}, {14, -10}, {14, 10}}, thickness = 0.5), Ellipse(origin = {60, -38}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}, endAngle = 360), Line(origin = {-40.8, -49.6462}, points = {{-18, 0}, {-12, 0}}, thickness = 0.5), Line(origin = {71.2923, -49.0154}, points = {{-18, 0}, {-12, 0}}, thickness = 0.5), Line(origin = {43, 68}, points = {{57, -7}, {57, 18}, {-7, 18}, {-7, -18}, {-17, -18}}, thickness = 0.5), Line(origin = {-94, -25}, points = {{6, -25}, {-6, -25}, {-6, 25}, {-6, 25}}, thickness = 0.5), Line(origin = {94, -25}, points = {{-6, -25}, {6, -25}, {6, 25}}, thickness = 0.5), Line(origin = {-63, 73}, points = {{37, 13}, {-37, 13}, {-37, -13}}, thickness = 0.5)}),
          uses(Modelica(version = "3.2.2")));
      end SolenoidPump;

    end Assemblies;

    package Valves
      extends Modelica.Icons.VariantsPackage;

      model ValveBodyDiscrete
        extends Modelica.Fluid.Interfaces.PartialTwoPortTransport;
        parameter Modelica.SIunits.AbsolutePressure dp_nominal "Nominal pressure drop at full opening=1" annotation(
          Dialog(group = "Nominal operating point"));
        parameter Medium.MassFlowRate m_flow_nominal "Nominal mass flowrate at full opening=1" annotation(
          Dialog(group = "Nominal operating point"));
        final parameter Modelica.Fluid.Types.HydraulicConductance k = m_flow_nominal / dp_nominal "Hydraulic conductance at full opening=1";
        parameter Real opening_min(min = 0) = 0 "Remaining opening if closed, causing small leakage flow";
        parameter Real thresholdClosed(min = 0) = 0 "Stem position beyond which valves falls open";
        Modelica.Mechanics.Translational.Interfaces.Flange_b flange_b annotation(
          Placement(visible = true, transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.Translational.Sensors.PositionSensor stemPosition annotation(
          Placement(visible = true, transformation(origin = {20, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Logical.Greater checkOpen annotation(
          Placement(visible = true, transformation(origin = {70, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        m_flow = if checkOpen.y then 1 * k * dp else opening_min * k * dp;
// Isenthalpic state transformation (no storage and no loss of energy)
        port_a.h_outflow = inStream(port_b.h_outflow);
        port_b.h_outflow = inStream(port_a.h_outflow);
        connect(thresholdClosed, checkOpen.u2);
        connect(stemPosition.s, checkOpen.u1) annotation(
          Line(points = {{32, 70}, {58, 70}, {58, 68}, {58, 68}}, color = {0, 0, 127}));
        connect(flange_b, stemPosition.flange) annotation(
          Line(points = {{0, 100}, {0, 100}, {0, 70}, {10, 70}, {10, 70}}, color = {0, 127, 0}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{0, 50}, {0, 0}}), Rectangle(extent = {{-20, 60}, {20, 50}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Polygon(points = {{-100, 50}, {100, -50}, {100, 50}, {0, 0}, {-100, -50}, {-100, 50}}, fillColor = DynamicSelect({255, 255, 255}, if open > 0.5 then {0, 255, 0} else {255, 255, 255}), lineColor = {0, 0, 0}, fillPattern = FillPattern.Solid)}));
      end ValveBodyDiscrete;

      model CheckValve "Simple generic check valve defined by pressure loss coefficient and diameter (only for flow from port_a to port_b)"
        extends Modelica.Fluid.Interfaces.PartialTwoPortTransport(dp_start = dp_nominal, m_flow_small = if system.use_eps_Re then system.eps_m_flow * m_flow_nominal else system.m_flow_small, m_flow(stateSelect = if momentumDynamics == Modelica.Fluid.Types.Dynamics.SteadyState then StateSelect.default else StateSelect.prefer), final allowFlowReversal = false);
        extends Modelica.Fluid.Interfaces.PartialLumpedFlow(final pathLength = 0, final momentumDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, final allowFlowReversal = false);
        parameter Modelica.SIunits.Diameter diameter "Diameter of orifice";
        parameter Real zeta "Loss factor for flow of port_a -> port_b" annotation(
          Dialog(enable = use_zeta));
        parameter Boolean use_zeta = true "= false to obtain zeta from dp_nominal and m_flow_nominal";
        // Operational conditions
        parameter Modelica.SIunits.MassFlowRate m_flow_nominal = if system.use_eps_Re then system.m_flow_nominal else 1e2 * system.m_flow_small "Mass flow rate for dp_nominal" annotation(
          Dialog(group = "Nominal operating point"));
        parameter Modelica.SIunits.Pressure dp_nominal = if not system.use_eps_Re then 1e3 else Modelica.Fluid.Fittings.BaseClasses.lossConstant_D_zeta(diameter, zeta) / Medium.density_pTX(Medium.p_default, Medium.T_default, Medium.X_default) * m_flow_nominal ^ 2 "Nominal pressure drop" annotation(
          Dialog(group = "Nominal operating point"));
        parameter Modelica.SIunits.Pressure dp_open = 0 "Threshold to open valve" annotation(
          Dialog(group = "Nominal operating point"));
        parameter Boolean use_Re = system.use_eps_Re "= true, if turbulent region is defined by Re, otherwise by m_flow_small" annotation(
          Dialog(tab = "Advanced"),
          Evaluate = true);
        parameter Boolean from_dp = true "= true, use m_flow = f(dp) else dp = f(m_flow)" annotation(
          Evaluate = true,
          Dialog(tab = "Advanced"));
      protected
        parameter Medium.AbsolutePressure dp_small(min = 0) = if system.use_eps_Re then dp_nominal / m_flow_nominal * m_flow_small else system.dp_small "Regularization of zero flow if |dp| < dp_small" annotation(
          Dialog(tab = "Advanced", enable = not use_Re and from_dp));
        // Variables
      public
        Real zeta_nominal;
        Medium.Density d = 0.5 * (Medium.density(state_a) + Medium.density(state_b));
        Modelica.SIunits.Pressure dp_fg(start = dp_start) "pressure loss due to friction and gravity";
        Modelica.SIunits.Area A_mean = Modelica.Constants.pi / 4 * diameter ^ 2 "mean cross flow area";
        constant Modelica.SIunits.ReynoldsNumber Re_turbulent = 10000 "cf. sharpEdgedOrifice";
        Modelica.SIunits.MassFlowRate m_flow_turbulent = if not use_Re then m_flow_small else max(m_flow_small, Modelica.Constants.pi / 8 * diameter * (Medium.dynamicViscosity(state_a) + Medium.dynamicViscosity(state_b)) * Re_turbulent);
        Modelica.SIunits.AbsolutePressure dp_turbulent = if not use_Re then dp_small else max(dp_small, Modelica.Fluid.Fittings.BaseClasses.lossConstant_D_zeta(diameter, zeta_nominal) / d * m_flow_turbulent ^ 2);
      equation
        if use_zeta then
          zeta_nominal = zeta;
        else
          zeta_nominal * m_flow_nominal ^ 2 = 2 * A_mean ^ 2 * d * dp_nominal;
        end if;
        Ib_flow = 0;
        F_p = A_mean * (Medium.pressure(state_b) - Medium.pressure(state_a));
        F_fg = A_mean * dp_fg;
        if from_dp then
          m_flow = homotopy(Modelica.Fluid.Utilities.regRoot2(dp_fg - dp_open, dp_turbulent, Medium.density(state_a) / Modelica.Fluid.Fittings.BaseClasses.lossConstant_D_zeta(diameter, zeta_nominal), 0.0, use_yd0 = true, yd0 = 0.0), m_flow_nominal * (dp_fg - dp_open) / dp_nominal);
        else
          dp_fg - dp_open = homotopy(Modelica.Fluid.Utilities.regSquare2(m_flow, m_flow_turbulent, Modelica.Fluid.Fittings.BaseClasses.lossConstant_D_zeta(diameter, zeta_nominal) / Medium.density(state_a), 0.0, use_yd0 = true, yd0 = 0.0), dp_nominal * m_flow / m_flow_nominal);
        end if;
        assert(dp_open >= 0, "Negative check valve opening pressure 'dp_open' is not allowed.");
// Isenthalpic state transformation (no storage and no loss of energy)
        port_a.h_outflow = if allowFlowReversal then inStream(port_b.h_outflow) else Medium.h_default;
        port_b.h_outflow = inStream(port_a.h_outflow);
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics = {Line(points = {{-60, -50}, {-60, 50}, {60, -50}, {60, 50}}, thickness = 0.5), Line(points = {{-60, 0}, {-100, 0}}, color = {0, 127, 255}), Line(points = {{60, 0}, {100, 0}}, color = {0, 127, 255}), Text(origin = {0, 2}, extent = {{-173, 104}, {175, 62}}, textString = "zeta=%zeta"), Ellipse(origin = {-60, 50}, fillPattern = FillPattern.Solid, extent = {{-15, 15}, {15, -15}}, endAngle = 360)}),
          Documentation(info = "<html>
<p>
This pressure drop component defines a
simple, generic check valve with orifice behaviour, where the loss factor &zeta; is provided
for one flow direction (e.g., from loss table of a book):
</p>
 
<pre>   &Delta;p = 0.5*&zeta;*&rho;*v*|v|
    = 8*&zeta;/(&pi;^2*D^4*&rho;) * m_flow*|m_flow|
</pre>
 
<p>
where
</p>
<ul>
<li> &Delta;p is the pressure drop: &Delta;p = port_a.p - port_b.p</li>
<li> D is the diameter of the orifice at the position where
   &zeta; is defined (either at port_a or port_b). If the orifice has not a
   circular cross section, D = 4*A/P, where A is the cross section
   area and P is the wetted perimeter.</li>
<li> &zeta; is the loss factor with respect to D
   that depends on the geometry of
   the orifice. In the turbulent flow regime, it is assumed that
   &zeta; is constant.<br>
   For small mass flow rates, the flow is laminar and is approximated
   by a polynomial that has a finite derivative for m_flow=0.</li>
<li> v is the mean velocity.</li>
<li> &rho; is the upstream density.</li>
</ul>
 
<p>
Since the pressure loss factor zeta is provided only for a mass flow
from port_a to port_b, the pressure loss is not correct when then
flow is reversing. If reversing flow only occurs in a short time interval,
this is most likely uncritical. If significant reversing flow
can appear, this component should not be used.
</p>
</html>"));
      end CheckValve;

      package BaseClasses
        extends Modelica.Icons.BasesPackage;

        partial model PartialValveBody
          import Modelica.Fluid.Types.CvTypes;
          extends Modelica.Fluid.Interfaces.PartialTwoPortTransport(dp_start = dp_nominal, m_flow_small = if system.use_eps_Re then system.eps_m_flow * m_flow_nominal else system.m_flow_small, m_flow_start = m_flow_nominal);
          parameter Modelica.Fluid.Types.CvTypes CvData = Modelica.Fluid.Types.CvTypes.OpPoint "Selection of flow coefficient" annotation(
            Dialog(group = "Flow Coefficient"));
          parameter Modelica.SIunits.Area Av(fixed = if CvData == Modelica.Fluid.Types.CvTypes.Av then true else false, start = m_flow_nominal / sqrt(rho_nominal * dp_nominal) * valveCharacteristic(opening_nominal)) "Av (metric) flow coefficient" annotation(
            Dialog(group = "Flow Coefficient", enable = CvData == Modelica.Fluid.Types.CvTypes.Av));
          parameter Real Kv = 0 "Kv (metric) flow coefficient [m3/h]" annotation(
            Dialog(group = "Flow Coefficient", enable = CvData == Modelica.Fluid.Types.CvTypes.Kv));
          parameter Real Cv = 0 "Cv (US) flow coefficient [USG/min]" annotation(
            Dialog(group = "Flow Coefficient", enable = CvData == Modelica.Fluid.Types.CvTypes.Cv));
          parameter Modelica.SIunits.Pressure dp_nominal "Nominal pressure drop" annotation(
            Dialog(group = "Nominal operating point"));
          parameter Medium.MassFlowRate m_flow_nominal "Nominal mass flowrate" annotation(
            Dialog(group = "Nominal operating point"));
          parameter Medium.Density rho_nominal = Medium.density_pTX(Medium.p_default, Medium.T_default, Medium.X_default) "Nominal inlet density" annotation(
            Dialog(group = "Nominal operating point", enable = CvData == Modelica.Fluid.Types.CvTypes.OpPoint));
          parameter Real opening_nominal(min = 0, max = 1) = 1 "Nominal opening" annotation(
            Dialog(group = "Nominal operating point", enable = CvData == Modelica.Fluid.Types.CvTypes.OpPoint));
          parameter Boolean filteredOpening = false "= true, if opening is filtered with a 2nd order CriticalDamping filter" annotation(
            Dialog(group = "Filtered opening"),
            choices(checkBox = true));
          parameter Modelica.SIunits.Time riseTime = 1 "Rise time of the filter (time to reach 99.6 % of an opening step)" annotation(
            Dialog(group = "Filtered opening", enable = filteredOpening));
          parameter Real leakageOpening(min = 0, max = 1) = 1e-3 "The opening signal is limited by leakageOpening (to improve the numerics)" annotation(
            Dialog(group = "Filtered opening", enable = filteredOpening));
          parameter Boolean checkValve = false "Reverse flow stopped" annotation(
            Dialog(tab = "Assumptions"));
          replaceable function valveCharacteristic = Modelica.Fluid.Valves.BaseClasses.ValveCharacteristics.linear constrainedby Modelica.Fluid.Valves.BaseClasses.ValveCharacteristics.baseFun "Inherent flow characteristic" annotation(
             choicesAllMatching = true);
          constant Modelica.SIunits.Area Kv2Av = 27.7e-6 "Conversion factor";
          constant Modelica.SIunits.Area Cv2Av = 24.0e-6 "Conversion factor";
          Modelica.Blocks.Interfaces.RealOutput opening_filtered if filteredOpening "Filtered valve position in the range 0..1" annotation(
            Placement(transformation(extent = {{60, 40}, {80, 60}}), iconTransformation(extent = {{60, 50}, {80, 70}})));
          Modelica.Blocks.Continuous.Filter filter(order = 2, f_cut = 5 / (2 * Modelica.Constants.pi * riseTime)) if filteredOpening annotation(
            Placement(transformation(extent = {{34, 44}, {48, 58}})));
          Modelica.Mechanics.Translational.Sensors.PositionSensor stemPosition annotation(
            Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Mechanics.Translational.Interfaces.Flange_b flange_b annotation(
            Placement(visible = true, transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        protected
          parameter Modelica.SIunits.Pressure dp_small = if system.use_eps_Re then dp_nominal / m_flow_nominal * m_flow_small else system.dp_small "Regularisation of zero flow" annotation(
            Dialog(tab = "Advanced"));
          Modelica.Blocks.Interfaces.RealOutput opening_actual annotation(
            Placement(transformation(extent = {{60, 10}, {80, 30}})));

          block MinLimiter "Limit the signal above a threshold"
            parameter Real uMin = 0 "Lower limit of input signal";
            extends Modelica.Blocks.Interfaces.SISO;
          equation
            y = smooth(0, noEvent(if u < uMin then uMin else u));
            annotation(
              Documentation(info = "<html>
        <p>
        The block passes its input signal as output signal
        as long as the input is above uMin. If this is not the case,
        y=uMin is passed as output.
        </p>
        </html>"),
              Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{0, -90}, {0, 68}}, color = {192, 192, 192}), Polygon(points = {{0, 90}, {-8, 68}, {8, 68}, {0, 90}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid), Line(points = {{-90, 0}, {68, 0}}, color = {192, 192, 192}), Polygon(points = {{90, 0}, {68, -8}, {68, 8}, {90, 0}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid), Line(points = {{-80, -70}, {-50, -70}, {50, 70}, {64, 90}}), Text(extent = {{-150, -150}, {150, -110}}, lineColor = {0, 0, 0}, textString = "uMin=%uMin"), Text(extent = {{-150, 150}, {150, 110}}, textString = "%name", lineColor = {0, 0, 255})}),
              Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{0, -60}, {0, 50}}, color = {192, 192, 192}), Polygon(points = {{0, 60}, {-5, 50}, {5, 50}, {0, 60}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid), Line(points = {{-60, 0}, {50, 0}}, color = {192, 192, 192}), Polygon(points = {{60, 0}, {50, -5}, {50, 5}, {60, 0}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid), Line(points = {{-50, -40}, {-30, -40}, {30, 40}, {50, 40}}), Text(extent = {{46, -6}, {68, -18}}, lineColor = {128, 128, 128}, textString = "u"), Text(extent = {{-30, 70}, {-5, 50}}, lineColor = {128, 128, 128}, textString = "y"), Text(extent = {{-58, -54}, {-28, -42}}, lineColor = {128, 128, 128}, textString = "uMin"), Text(extent = {{26, 40}, {66, 56}}, lineColor = {128, 128, 128}, textString = "uMax")}));
          end MinLimiter;

          MinLimiter minLimiter(uMin = leakageOpening) annotation(
            Placement(transformation(extent = {{10, 44}, {24, 58}})));
        initial equation
          if CvData == CvTypes.Kv then
            Av = Kv * Kv2Av "Unit conversion";
          elseif CvData == CvTypes.Cv then
            Av = Cv * Cv2Av "Unit conversion";
          end if;
        equation
// Isenthalpic state transformation (no storage and no loss of energy)
          port_a.h_outflow = inStream(port_b.h_outflow);
          port_b.h_outflow = inStream(port_a.h_outflow);
          connect(filter.y, opening_filtered) annotation(
            Line(points = {{48.7, 51}, {60, 51}, {60, 50}, {70, 50}}, color = {0, 0, 127}));
          if filteredOpening then
            connect(filter.y, opening_actual);
          else
            connect(stemPosition.s, opening_actual);
          end if;
          connect(flange_b, stemPosition.flange) annotation(
            Line(points = {{0, 100}, {0, 100}, {0, 80}, {-40, 80}, {-40, 50}, {-20, 50}, {-20, 50}}, color = {0, 127, 0}));
          connect(stemPosition.s, minLimiter.u) annotation(
            Line(points = {{1, 50}, {8, 50}, {8, 52}}, color = {0, 0, 127}));
          connect(minLimiter.y, filter.u) annotation(
            Line(points = {{24.7, 51}, {32.6, 51}}, color = {0, 0, 127}));
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end PartialValveBody;
        annotation(
          Icon(coordinateSystem(grid = {2, 8})));
      end BaseClasses;
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end Valves;

    package Sensors
      extends Modelica.Icons.SensorsPackage;

      model FluidThermostat
        extends Modelica.Fluid.Sensors.Temperature;
        parameter Modelica.SIunits.Temperature tLow, tHigh;
        parameter Boolean pre_y_start = false;
        Modelica.Blocks.Logical.Hysteresis hysteresis(uHigh = tHigh, uLow = tLow, pre_y_start = pre_y_start) annotation(
          Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.BooleanOutput y annotation(
          Placement(visible = true, transformation(origin = {130, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(hysteresis.y, y) annotation(
          Line(points = {{102, 0}, {122, 0}, {122, 0}, {130, 0}}, color = {255, 0, 255}));
        connect(T, hysteresis.u) annotation(
          Line(points = {{70, 0}, {78, 0}, {78, 0}, {78, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(grid = {2, 8})));
      end FluidThermostat;

      model FluidPressostat
        extends Modelica.Fluid.Sensors.Pressure;
        parameter Modelica.SIunits.Pressure tLow, tHigh;
        parameter Boolean pre_y_start = false;
        Modelica.Blocks.Logical.Hysteresis hysteresis(uHigh = tHigh, uLow = tLow, pre_y_start = pre_y_start) annotation(
          Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.BooleanOutput y annotation(
          Placement(visible = true, transformation(origin = {130, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(hysteresis.y, y) annotation(
          Line(points = {{102, 0}, {122, 0}, {122, 0}, {130, 0}}, color = {255, 0, 255}));
        connect(p, hysteresis.u) annotation(
          Line(points = {{70, 0}, {78, 0}, {78, 0}, {78, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(grid = {2, 8})));
      end FluidPressostat;

      model FlowTransmitter
        extends Modelica.Fluid.Sensors.BaseClasses.PartialFlowSensor;
        extends Modelica.Icons.RotationalSensor;
        parameter Modelica.SIunits.Volume volumeFlowPerPulse;
        Modelica.SIunits.Volume V;
        Integer i;
        Modelica.Blocks.Interfaces.BooleanOutput V_flowPulseSignal "Volume flow rate from port_a to port_b" annotation(
          Placement(transformation(origin = {0, 110}, extent = {{10, -10}, {-10, 10}}, rotation = 270)));
      protected
        Medium.Density rho_a_inflow "Density of inflowing fluid at port_a";
        Medium.Density rho_b_inflow "Density of inflowing fluid at port_b or rho_a_inflow, if uni-directional flow";
        Medium.Density d "Density of the passing fluid";
      equation
        if allowFlowReversal then
          rho_a_inflow = Medium.density(Medium.setState_phX(port_b.p, port_b.h_outflow, port_b.Xi_outflow));
          rho_b_inflow = Medium.density(Medium.setState_phX(port_a.p, port_a.h_outflow, port_a.Xi_outflow));
          d = Modelica.Fluid.Utilities.regStep(port_a.m_flow, rho_a_inflow, rho_b_inflow, m_flow_small);
        else
          d = Medium.density(Medium.setState_phX(port_b.p, port_b.h_outflow, port_b.Xi_outflow));
          rho_a_inflow = d;
          rho_b_inflow = d;
        end if;
        der(V)*d=port_a.m_flow;
        when V>(i+1)*volumeFlowPerPulse then 
          V_flowPulseSignal=true;
          i= pre(i)+1;
        elsewhen V>(i+0.5)*volumeFlowPerPulse then 
          V_flowPulseSignal=false;
        end when;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{160, 120}, {0, 90}}, lineColor = {0, 0, 0}, textString = "V_flow"), Line(points = {{0, 100}, {0, 70}}, color = {0, 0, 127}), Line(points = {{-100, 0}, {-70, 0}}, color = {0, 128, 255}), Line(points = {{70, 0}, {100, 0}}, color = {0, 128, 255})}),
          Documentation(info = "<html>
      <p>
      This component monitors the volume flow rate flowing from port_a to port_b.
      The sensor is ideal, i.e., it does not influence the fluid.
      </p>
      </html>"));
      end FlowTransmitter;






      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end Sensors;
    annotation(
      Icon(graphics = {Polygon(points = {{-70, 26}, {68, -44}, {68, 26}, {2, -10}, {-70, -42}, {-70, 26}}, lineColor = {0, 0, 0}), Line(points = {{2, 42}, {2, -10}}), Rectangle(extent = {{-18, 50}, {22, 42}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid)}));
  end Fluid;

  package DE
    extends Modelica.Icons.Package;

    model EN253pipe
      // ---------- Definitions ----------
      // ---------- Inheritances ----------
      extends Modelica.Fluid.Pipes.DynamicPipe(redeclare replaceable package Medium = DH_Medium, redeclare replaceable model HeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.IdealFlowHeatTransfer constrainedby Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.PartialFlowHeatTransfer, roughness = globalDHconfig.roughness, heatTransfer.k = lambda_insul * 2 * Modelica.Constants.pi / Modelica.Math.log(outsideDiaInsul / 2 / (outsideDiaPipe / 2)), diameter = insideDiaPipe, use_HeatTransfer = true);
      // ---------- Declarations ----------
      replaceable package DH_Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
      parameter MyLib.DE.PipeSpec.InsulClass insulClass = globalDHconfig.insulClass annotation(
        Dialog(tab = "General", group = "EN253 Pipe Type"));
      parameter MyLib.DE.PipeSpec.DNtype DN annotation(
        Dialog(tab = "General", group = "EN253 Pipe Type"));
      parameter Modelica.SIunits.ThermalConductivity lambda_insul = globalDHconfig.lambda_insul annotation(
        Dialog(tab = "General", group = "EN253 Pipe Type"));
    protected
      parameter Modelica.SIunits.Diameter insideDiaPipe = outsideDiaPipe - 2 * pipeThickness;
    protected
      parameter Modelica.SIunits.Diameter outsideDiaPipe = MyLib.DE.SpecTables.EN253Spec[insulClass, DN, 1];
    protected
      parameter Modelica.SIunits.Radius pipeThickness = MyLib.DE.SpecTables.EN253Spec[insulClass, DN, 2];
    protected
      parameter Modelica.SIunits.Diameter outsideDiaInsul = MyLib.DE.SpecTables.EN253Spec[insulClass, DN, 3];
    protected
      outer parameter MyLib.DE.GlobalDHconfig globalDHconfig annotation(
        Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // ---------- Components ----------
      // ---------- Connectors ----------
      // ---------- Equations ----------
      // ---------- Graphics & Layout ----------
      annotation(
        Icon(coordinateSystem(grid = {2, 8}, initialScale = 0.1), graphics = {Rectangle(origin = {-1, 60}, fillColor = {240, 240, 119}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-99, 4}, {101, -20}}), Rectangle(origin = {0, -60}, fillColor = {240, 240, 119}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-100, 20}, {100, -4}}), Text(origin = {0, 80}, extent = {{-64, 16}, {64, -16}}, textString = "EN253 Pipe", fontName = "DejaVu Sans Mono")}),
        Diagram);
    end EN253pipe;

    record PipeSpec
      extends Modelica.Icons.Record;
      type InsulClass = enumeration(Class_1, Class_2, Class_3);
      type DNtype = enumeration(DN20, DN25, DN32, DN40, DN50, DN65, DN80, DN100, DN125, DN150, DN200, DN250, DN300, DN350, DN400, DN450, DN500, DN600, DN700, DN800, DN900, DN1000);
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end PipeSpec;

    package SpecTables
      final constant Real EN253Spec[MyLib.DE.PipeSpec.InsulClass, MyLib.DE.PipeSpec.DNtype, 3] = {{{0.0269, 0.0026, 0.090}, {0.0337, 0.0026, 0.090}, {0.0424, 0.0026, 0.110}, {0.0483, 0.0026, 0.110}, {0.0603, 0.0029, 0.125}, {0.0761, 0.0029, 0.140}, {0.0889, 0.0032, 0.160}, {0.1143, 0.0036, 0.200}, {0.1397, 0.0036, 0.225}, {0.1683, 0.0040, 0.250}, {0.2191, 0.0045, 0.315}, {0.273, 0.0050, 0.400}, {0.3239, 0.0056, 0.450}, {0.3556, 0.0056, 0.500}, {0.4064, 0.0063, 0.560}, {0.4572, 0.0063, 0.630}, {0.508, 0.0063, 0.710}, {0.610, 0.0071, 0.800}, {0.711, 0.008, 0.900}, {0.813, 0.0088, 1.000}, {0.914, 0.01, 1.100}, {1.016, 0.011, 1.200}}, {{0.0269, 0.0026, 0.110}, {0.0337, 0.0026, 0.110}, {0.0424, 0.0026, 0.125}, {0.0483, 0.0026, 0.125}, {0.0603, 0.0029, 0.140}, {0.0761, 0.0029, 0.160}, {0.0889, 0.0032, 0.180}, {0.1143, 0.0036, 0.225}, {0.1397, 0.0036, 0.250}, {0.1683, 0.0040, 0.280}, {0.2191, 0.0045, 0.355}, {0.273, 0.0050, 0.450}, {0.3239, 0.0056, 0.500}, {0.3556, 0.0056, 0.560}, {0.4064, 0.0063, 0.560}, {0.4572, 0.0063, 0.630}, {0.508, 0.0063, 0.710}, {0.610, 0.0071, 0.800}, {0.711, 0.008, 0.900}, {0.813, 0.0088, 1.000}, {0.914, 0.01, 1.100}, {1.016, 0.011, 1.200}}, {{0.0269, 0.0026, 0.125}, {0.0337, 0.0026, 0.125}, {0.0424, 0.0026, 0.140}, {0.0483, 0.0026, 0.140}, {0.0603, 0.0029, 0.160}, {0.0761, 0.0029, 0.180}, {0.0889, 0.0032, 0.200}, {0.1143, 0.0036, 0.250}, {0.1397, 0.0036, 0.280}, {0.1683, 0.0040, 0.315}, {0.2191, 0.0045, 0.400}, {0.273, 0.0050, 0.500}, {0.3239, 0.0056, 0.560}, {0.3556, 0.0056, 0.560}, {0.4064, 0.0063, 0.630}, {0.4572, 0.0063, 0.710}, {0.508, 0.0063, 0.800}, {0.610, 0.0071, 0.800}, {0.711, 0.008, 0.900}, {0.813, 0.0088, 1.000}, {0.914, 0.01, 1.100}, {1.016, 0.011, 1.200}}};
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end SpecTables;

    model DH_ETS
      // ---------- Definitions and Inheritances ----------
      extends MyLib.Fluid.Assemblies.ETS(redeclare replaceable package PrimaryMedium = DH_Medium, redeclare replaceable package SecondaryMedium = BldgMedium, hexChannels = integer(hexArea / hexChannelArea), hexChannelDiameter = 0.011, hexChannelLength = hexChannelArea / hexChannelDiameter, hexAlpha = 4000, hexWallConductivity = 16.5, hexWallThickness = 0.0005, valve_dp_nom = 30000, valve_flow_nom = (spaceHeatCapacity + dhwHeatCapacity + processHeatCapacity) / (bldgMedium_cp * (globalDHconfig.bldgHWS_temp - globalDHconfig.bldgHWR_temp)), pumpVolume = 0.001, pump_flow_nom = (spaceHeatCapacity + dhwHeatCapacity + processHeatCapacity) / (bldgMedium_cp * (globalDHconfig.bldgHWS_temp - globalDHconfig.bldgHWR_temp)));
      // ---------- Declarations ----------
      replaceable package DH_Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
      replaceable package BldgMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
      parameter Modelica.SIunits.Power spaceHeatCapacity annotation(
        Dialog(tab = "General", group = "Design Conditions"));
      parameter Modelica.SIunits.Power dhwHeatCapacity annotation(
        Dialog(tab = "General", group = "Design Conditions"));
      parameter Modelica.SIunits.Power processHeatCapacity annotation(
        Dialog(tab = "General", group = "Design Conditions"));
      parameter Modelica.SIunits.Temperature hexLMTD = globalDHconfig.hexLMTD annotation(
        Dialog(tab = "General", group = "Design Conditions"));
      parameter Modelica.SIunits.Area hexArea = (spaceHeatCapacity + dhwHeatCapacity + processHeatCapacity) / (hexLMTD * (2 * hexAlpha + hexWallConductivity * hexWallThickness)) annotation(
        Dialog(tab = "General", group = "Geometry"));
      parameter Modelica.SIunits.Area hexChannelArea = 0.67 annotation(
        Dialog(tab = "Assumptions", group = "Geometry"));
    protected
      parameter Modelica.SIunits.SpecificHeatCapacity bldgMedium_cp(fixed = false) = BldgMedium.BaseProperties.specificHeatCapacityCp(bldgMediumState);
    protected
      parameter BldgMedium.ThermodynamicState bldgMediumState;
      outer MyLib.DE.GlobalDHconfig globalDHconfig annotation(
        Placement(visible = true, transformation(origin = {250, 90}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      // ---------- Components ----------
      Modelica.Fluid.Sources.FixedBoundary bldgHWS(redeclare replaceable package Medium = BldgMedium, T = globalDHconfig.bldgHWS_temp, nPorts = 1) annotation(
        Placement(visible = true, transformation(origin = {250, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Fluid.Sources.FixedBoundary bldgHWR(redeclare replaceable package Medium = BldgMedium, T = globalDHconfig.bldgHWR_temp, nPorts = 1) annotation(
        Placement(visible = true, transformation(origin = {250, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum loadCalc(k = array(spaceHeatCapacity, dhwHeatCapacity, processHeatCapacity), nu = 3) annotation(
        Placement(visible = true, transformation(origin = {150, 10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Division massflowCalc annotation(
        Placement(visible = true, transformation(origin = {110, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Product spezificHeat annotation(
        Placement(visible = true, transformation(origin = {140, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add bldgDeltaT(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {170, -42}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Fluid.Sensors.Temperature HWS_TE2 annotation(
        Placement(visible = true, transformation(origin = {210, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Fluid.Sensors.Temperature HWR_TE2 annotation(
        Placement(visible = true, transformation(origin = {210, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression bldgDesignCp(y = bldgMedium_cp) annotation(
        Placement(visible = true, transformation(origin = {250, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    initial equation
      bldgMediumState = BldgMedium.setState_pTX(bldgHWR.p, (globalDHconfig.bldgHWS_temp + globalDHconfig.bldgHWR_temp) / 2, bldgHWR.X);
    equation
// ---------- Connectors ----------
      connect(HWR_TE2.T, bldgDeltaT.u2) annotation(
        Line(points = {{203, -50}, {191, -50}, {191, -48}, {182, -48}}, color = {0, 0, 127}));
      connect(HWR_TE2.port, secondaryHWR) annotation(
        Line(points = {{210, -60}, {210, -70}, {90, -70}}, color = {0, 127, 255}));
      connect(bldgHWR.ports[1], HWR_TE2.port) annotation(
        Line(points = {{240, -70}, {210, -70}, {210, -60}, {210, -60}}, color = {0, 127, 255}));
      connect(HWS_TE2.T, bldgDeltaT.u1) annotation(
        Line(points = {{203, 50}, {191, 50}, {191, -36}, {182, -36}}, color = {0, 0, 127}));
      connect(loadCalc.y, massflowCalc.u1) annotation(
        Line(points = {{138, 10}, {126, 10}, {126, -24}, {122, -24}}, color = {0, 0, 127}));
      if globalDHconfig.use_HWS_temp_SP then
        connect(globalDHconfig.heatLoads, loadCalc.u) annotation(
          Line(points = {{240.8, 89.2}, {179.8, 89.2}, {179.8, 10}, {160, 10}}, color = {0, 0, 127}));
      else
        connect(globalDHconfig.constHeatLoads, loadCalc.u);
      end if;
      if globalDHconfig.use_HWS_temp_SP then
        connect(globalDHconfig.bldgHWS_temp_SP, HWS_temp_SP) annotation(
          Line(points = {{242, 98}, {100, 98}, {100, 70}, {90, 70}}, color = {0, 0, 127}));
      else
        connect(globalDHconfig.constHWS_temp_SP, HWS_temp_SP);
      end if;
      connect(HWS_TE2.port, bldgHWS.ports[1]) annotation(
        Line(points = {{210, 40}, {210, 30}, {240, 30}}));
      connect(secondaryHWS, HWS_TE2.port) annotation(
        Line(points = {{90, 30}, {210, 30}, {210, 40}}));
      connect(bldgDesignCp.y, spezificHeat.u1) annotation(
        Line(points = {{239, -10}, {157.5, -10}, {157.5, -30}, {152, -30}}, color = {0, 0, 127}));
      connect(spezificHeat.y, massflowCalc.u2) annotation(
        Line(points = {{129, -36}, {122, -36}}, color = {0, 0, 127}));
      connect(bldgDeltaT.y, spezificHeat.u2) annotation(
        Line(points = {{159, -42}, {152, -42}}, color = {0, 0, 127}));
      connect(massflowCalc.y, pumpMassFlowIn) annotation(
        Line(points = {{99, -30}, {90, -30}}, color = {0, 0, 127}));
// ---------- Equations ----------
// ---------- Graphics & Layout ----------
      annotation(
        Icon(coordinateSystem(extent = {{-100, -100}, {260, 100}}, initialScale = 0.1), graphics = {Rectangle(origin = {121, 20}, fillColor = {255, 255, 127}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-41, 40}, {59, -60}}), Rectangle(origin = {150, -21}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-10, 19}, {10, -19}}), Line(origin = {130, 76}, points = {{0, 0}}), Polygon(origin = {130, 79.89}, fillColor = {163, 55, 0}, fillPattern = FillPattern.Solid, lineThickness = 1, points = {{-50, -20}, {0, 20}, {50, -20}, {-50, -20}})}),
        uses(Modelica(version = "3.2.1")),
        Diagram(coordinateSystem(extent = {{-100, -100}, {260, 100}})),
        version = "");
    end DH_ETS;

    model GlobalDHconfig
      // ---------- Declarations ----------
      replaceable package DH_Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium annotation(
         Dialog(tab = "General", group = "DH Distribution Medium"),
         choicesAllMatching = true);
      parameter Boolean use_HWS_temp_SP = false "Set true if set-point shall be variable" annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Boolean use_calculatedHeatLoads = false "Set true if heat loads shall be variable" annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Modelica.SIunits.Temperature constHWS_temp_SP(displayUnit = "degC") "If use_HWS_temp_SP is false" annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Modelica.SIunits.Temperature actualHWR_temp(displayUnit = "degC") "Actual building return temperature" annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Real spaceHeatGain = 1 annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Real dhwHeatGain = 1 annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Real processHeatGain = 1 annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Real constHeatLoads[3] = array(spaceHeatGain, dhwHeatGain, processHeatGain) "If use_calculatedHeatLoads is false" annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Real oatTable[:, 2] = {{0.0, -8.0}, {86400.0, -8.0}} annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Real HWS_temp_SP_Table[:, 2] = {{-273.0, 1.0}, {-8.0, 1.0}, {15.0, 1.0}, {273.0, 1.0}} annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Real spaceHeatTable[:, 2] = {{0.0, 1.0}, {86400.0, 1.0}} annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Real dhwHeatTable[:, 2] = {{0.0, 1.0}, {86400.0, 1.0}} annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Real processHeatTable[:, 2] = {{0.0, 1.0}, {86400.0, 1.0}} annotation(
        Dialog(tab = "General", group = "Simulation"));
      parameter Modelica.SIunits.Height roughness(displayUnit = "mm") = 0.00005 annotation(
        Dialog(tab = "Distribution Pipe", group = "Geometry"));
      parameter MyLib.DE.PipeSpec.InsulClass insulClass = MyLib.DE.PipeSpec.InsulClass.Class_2 annotation(
        Dialog(tab = "Distribution Pipe", group = "Geometry"));
      parameter Modelica.SIunits.ThermalConductivity lambda_insul = 0.03 annotation(
        Dialog(tab = "Distribution Pipe", group = "Characteristics"));
      parameter Modelica.SIunits.TemperatureDifference distrTempOffset = 5 annotation(
        Dialog(tab = "ETS", group = "Control Parameters"));
      parameter Modelica.SIunits.TemperatureDifference hexLMTD = 2 annotation(
        Dialog(tab = "ETS", group = "Design Conditions"));
      parameter Modelica.SIunits.Temperature bldgHWS_temp(displayUnit = "degC") annotation(
        Dialog(tab = "ETS", group = "Design Conditions"));
      parameter Modelica.SIunits.Temperature bldgHWR_temp(displayUnit = "degC") annotation(
        Dialog(tab = "ETS", group = "Design Conditions"));
      parameter Modelica.SIunits.Pressure initialFillPressure(displayUnit = "Pa") = 150000 annotation(
        Dialog(tab = "Initialization"));
      // ---------- Declarations ----------
      Modelica.Blocks.Sources.TimeTable dhwHeatCurve(table = dhwHeatTable) annotation(
        Placement(visible = true, transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.TimeTable processHeatCurve(table = processHeatTable) annotation(
        Placement(visible = true, transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.TimeTable oatCurve(table = oatTable) annotation(
        Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain spaceHeatLoad(k = spaceHeatGain) annotation(
        Placement(visible = true, transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain dhwHeatLoad(k = dhwHeatGain) annotation(
        Placement(visible = true, transformation(origin = {50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain processHeatLoad(k = processHeatGain) annotation(
        Placement(visible = true, transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Tables.CombiTable1Ds HWS_temp_SP_curve(table = HWS_temp_SP_Table) annotation(
        Placement(visible = true, transformation(origin = {-30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Tables.CombiTable1Ds spaceHeatLoadCurve(table = spaceHeatTable) annotation(
        Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput bldgHWS_temp_SP if use_HWS_temp_SP annotation(
        Placement(visible = true, transformation(origin = {90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput heatLoads[3] if use_calculatedHeatLoads annotation(
        Placement(visible = true, transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {92, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      if use_calculatedHeatLoads then
        connect(processHeatLoad.y, heatLoads[3]) annotation(
          Line(points = {{62, -50}, {82, -50}, {82, -10}, {90, -10}}, color = {0, 0, 127}));
        connect(dhwHeatLoad.y, heatLoads[2]) annotation(
          Line(points = {{62, -10}, {84, -10}, {84, -10}, {90, -10}}, color = {0, 0, 127}));
        connect(spaceHeatLoad.y, heatLoads[1]) annotation(
          Line(points = {{62, 30}, {82, 30}, {82, -10}, {90, -10}}, color = {0, 0, 127}));
      end if;
      if use_HWS_temp_SP then
        connect(HWS_temp_SP_curve.y[1], bldgHWS_temp_SP) annotation(
          Line(points = {{-18, 70}, {84, 70}, {84, 70}, {90, 70}}, color = {0, 0, 127}));
      else
        connect(constHWS_temp_SP, bldgHWS_temp_SP);
      end if;
      connect(oatCurve.y, spaceHeatLoadCurve.u) annotation(
        Line(points = {{-59, 50}, {-50, 50}, {-50, 30}, {-42, 30}}, color = {0, 0, 127}));
      connect(oatCurve.y, HWS_temp_SP_curve.u) annotation(
        Line(points = {{-59, 50}, {-50, 50}, {-50, 70}, {-42, 70}}, color = {0, 0, 127}));
      connect(spaceHeatLoadCurve.y[1], spaceHeatLoad.u) annotation(
        Line(points = {{-18, 30}, {38, 30}, {38, 30}, {38, 30}}, color = {0, 0, 127}));
      connect(dhwHeatCurve.y, dhwHeatLoad.u) annotation(
        Line(points = {{-59, -10}, {37, -10}}, color = {0, 0, 127}));
      connect(processHeatCurve.y, processHeatLoad.u) annotation(
        Line(points = {{-59, -50}, {37, -50}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8}, initialScale = 0.1), graphics = {Ellipse(origin = {-10, 12}, fillColor = {255, 170, 127}, fillPattern = FillPattern.Sphere, extent = {{-70, 68}, {90, -92}}, endAngle = 360), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name", fontName = "DejaVu Sans Mono"), Text(origin = {1, 80}, extent = {{-80, 0}, {80, -50}}, textString = "Global", fontName = "DejaVu Sans Mono", textStyle = {TextStyle.Bold}), Text(origin = {5, -64}, lineThickness = 1, extent = {{-80, 30}, {80, -20}}, textString = "Design", fontName = "DejaVu Sans Mono", textStyle = {TextStyle.Bold}), Text(origin = {0, 8}, lineColor = {170, 0, 0}, lineThickness = 1, extent = {{-80, 24}, {80, -40}}, textString = "DH", fontName = "DejaVu Sans Mono", textStyle = {TextStyle.Bold})}),
        uses(Modelica(version = "3.2.1")),
        defaultComponentName = "globalDHconfig");
    end GlobalDHconfig;
    annotation(
      Icon(coordinateSystem(grid = {2, 8}, initialScale = 0.1), graphics = {Bitmap(origin = {3, 24}, extent = {{-93, 24}, {89, -64}}, imageSource = "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")}),
      Diagram);
  end DE;

  package Electrical
    extends Modelica.Icons.Package;

    package Analog
      extends Modelica.Icons.Package;

      package Basic
        extends Modelica.Icons.Package;

        model SteinhartResistor
          extends Modelica.Electrical.Analog.Interfaces.OnePort;
          extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T = T_ref, useHeatPort = true);
          parameter Modelica.SIunits.Resistance R_ref "Resistance at temperature T_ref";
          parameter Modelica.SIunits.Temperature T_ref = 298.15 "Reference temperature";
          parameter Boolean use_BetaOnly = false "= true to use Beta coeff. and a simplified Steinhart equation";
          parameter Real A = 1 / R_ref, B = 0, C = 0, D = 0 if not use_BetaOnly;
          parameter Real Beta = -1e12 if use_BetaOnly;
          Modelica.SIunits.Resistance R;
        equation
          if not use_BetaOnly then
            1 / T_heatPort = A + B * log(R / R_ref) + C * log(R / R_ref) ^ 2 + D * log(R / R_ref) ^ 3;
          elseif use_BetaOnly then
            1 / T_heatPort = 1 / T_ref + 1 / Beta * log(R / R_ref);
          end if;
          v = R * i;
          LossPower = v * i;
          annotation(
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-90, 0}, {-70, 0}}, color = {0, 0, 255}), Line(points = {{70, 0}, {90, 0}}, color = {0, 0, 255}), Rectangle(extent = {{-70, 30}, {70, -30}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Line(points = {{-52, -50}, {48, 50}}, color = {0, 0, 255}), Polygon(points = {{40, 52}, {50, 42}, {54, 56}, {40, 52}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid), Line(visible = useHeatPort, points = {{0, -100}, {0, -30}}, color = {127, 0, 0}, pattern = LinePattern.Dot), Text(extent = {{-156, 109}, {144, 69}}, textString = "%name", lineColor = {0, 0, 255})}));
        end SteinhartResistor;
        annotation(
          Icon(coordinateSystem(grid = {2, 8})));
      end Basic;

      package Ideal
        extends Modelica.Icons.Package;

        model BooleanToDO
          parameter Modelica.SIunits.Resistance RsupplyToSignal = 1.E-6;
          parameter Modelica.SIunits.Resistance RsignalToGround = 1.E6;
          Modelica.Electrical.Analog.Ideal.IdealClosingSwitch idealClosingSwitch(Goff = 0, Ron = 0) annotation(
            Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
          Modelica.Electrical.Analog.Basic.Resistor supplyResistor(R = RsupplyToSignal, T_ref = 298.15) annotation(
            Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Electrical.Analog.Basic.Resistor groundResistor(R = RsignalToGround, T_ref = 298.15) annotation(
            Placement(visible = true, transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Electrical.Analog.Interfaces.PositivePin p_Vsupply annotation(
            Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Electrical.Analog.Interfaces.NegativePin n_ground annotation(
            Placement(visible = true, transformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Electrical.Analog.Interfaces.NegativePin n_signalOut annotation(
            Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.BooleanInput u_signalIn annotation(
            Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        equation
          connect(supplyResistor.n, idealClosingSwitch.p) annotation(
            Line(points = {{0, 40}, {0, 40}, {0, 10}, {0, 10}}, color = {0, 0, 255}));
          connect(p_Vsupply, supplyResistor.p) annotation(
            Line(points = {{0, 90}, {0, 90}, {0, 60}, {0, 60}}, color = {0, 0, 255}));
          connect(idealClosingSwitch.n, n_signalOut) annotation(
            Line(points = {{0, -10}, {0, -10}, {0, -20}, {60, -20}, {60, 0}, {90, 0}, {90, 0}}, color = {0, 0, 255}));
          connect(groundResistor.n, n_ground) annotation(
            Line(points = {{0, -60}, {0, -60}, {0, -90}, {0, -90}}, color = {0, 0, 255}));
          connect(idealClosingSwitch.n, groundResistor.p) annotation(
            Line(points = {{0, -10}, {0, -10}, {0, -40}, {0, -40}}, color = {0, 0, 255}));
          connect(u_signalIn, idealClosingSwitch.control) annotation(
            Line(points = {{-100, 0}, {-6, 0}, {-6, 0}, {-6, 0}}, color = {255, 0, 255}));
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end BooleanToDO;

        model AIToInteger
          extends MyLib.Electrical.Analog.Ideal.BaseClasses.AIToInteger_Partial;
          Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation(
            Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
          Modelica.Electrical.Analog.Interfaces.PositivePin p_signalIn annotation(
            Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Electrical.Analog.Interfaces.NegativePin n_ground annotation(
            Placement(visible = true, transformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
          connect(voltageSensor.v, signalNoise.u) annotation(
            Line(points = {{-40, -30}, {-28, -30}, {-28, -30}, {-28, -30}}, color = {0, 0, 127}));
          connect(p_signalIn, voltageSensor.p) annotation(
            Line(points = {{0, 90}, {0, 60}, {-50, 60}, {-50, -20}}, color = {0, 0, 255}));
          connect(voltageSensor.n, n_ground) annotation(
            Line(points = {{-50, -40}, {-50, -66}, {0, -66}, {0, -90}}, color = {0, 0, 255}));
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end AIToInteger;

        model DIToBoolean
          extends MyLib.Electrical.Analog.Ideal.BaseClasses.DIToBoolean_Partial;
          Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation(
            Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
          Modelica.Electrical.Analog.Interfaces.PositivePin p_signalIn annotation(
            Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Electrical.Analog.Interfaces.NegativePin n_ground annotation(
            Placement(visible = true, transformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
          connect(voltageSensor.v, signalNoise.u) annotation(
            Line(points = {{-40, -30}, {-30, -30}, {-30, -30}, {-28, -30}}, color = {0, 0, 127}));
          connect(p_signalIn, voltageSensor.p) annotation(
            Line(points = {{0, 90}, {0, 60}, {-50, 60}, {-50, -20}}, color = {0, 0, 255}));
          connect(voltageSensor.n, n_ground) annotation(
            Line(points = {{-50, -40}, {-50, -66}, {0, -66}, {0, -90}}, color = {0, 0, 255}));
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end DIToBoolean;

        package BaseClasses
          extends Modelica.Icons.BasesPackage;

          partial model AIToInteger_Partial
            parameter Real referenceSignal;
            parameter Real signalResolution;
            parameter Boolean enableNoise = false;
            parameter Real noiseAmplitude = 0.01;
            Modelica.Blocks.Interfaces.IntegerOutput y_signalOut annotation(
              Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            Modelica.Blocks.Math.RealToInteger realToInteger annotation(
              Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            Modelica.Blocks.Math.Product signalScaling annotation(
              Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            Modelica.Blocks.Sources.Constant scaleMaxValue(k = signalResolution / referenceSignal) annotation(
              Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            MyLib.Utilities.SignalNoise signalNoise(enableNoise = enableNoise, noiseAmplitude = noiseAmplitude) annotation(
              Placement(visible = true, transformation(origin = {-20, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          equation
            connect(signalNoise.y, signalScaling.u2) annotation(
              Line(points = {{-8, -30}, {0, -30}, {0, -6}, {18, -6}, {18, -6}}, color = {0, 0, 127}));
            connect(scaleMaxValue.y, signalScaling.u1) annotation(
              Line(points = {{-19, 30}, {0, 30}, {0, 6}, {18, 6}}, color = {0, 0, 127}));
            connect(signalScaling.y, realToInteger.u) annotation(
              Line(points = {{42, 0}, {56, 0}, {56, 0}, {58, 0}}, color = {0, 0, 127}));
            connect(realToInteger.y, y_signalOut) annotation(
              Line(points = {{82, 0}, {102, 0}, {102, 0}, {110, 0}}, color = {255, 127, 0}));
            annotation(
              Icon(coordinateSystem(grid = {2, 8})));
          end AIToInteger_Partial;

          partial model DIToBoolean_Partial
            parameter Real switchingThreshold;
            parameter Boolean enableNoise = false;
            parameter Real noiseAmplitude = 0.01;
            Modelica.Blocks.Interfaces.BooleanOutput y_signalOut annotation(
              Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            Modelica.Blocks.Sources.Constant scaleMaxValue(k = switchingThreshold) annotation(
              Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            Modelica.Blocks.Logical.LessEqual lessEqual annotation(
              Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            MyLib.Utilities.SignalNoise signalNoise(enableNoise = enableNoise, noiseAmplitude = noiseAmplitude) annotation(
              Placement(visible = true, transformation(origin = {-20, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          equation
            connect(signalNoise.y, lessEqual.u2) annotation(
              Line(points = {{-8, -30}, {20, -30}, {20, -8}, {38, -8}, {38, -8}}, color = {0, 0, 127}));
            connect(lessEqual.y, y_signalOut) annotation(
              Line(points = {{62, 0}, {102, 0}, {102, 0}, {110, 0}}, color = {255, 0, 255}));
            connect(scaleMaxValue.y, lessEqual.u1) annotation(
              Line(points = {{-18, 30}, {20, 30}, {20, 0}, {38, 0}, {38, 0}}, color = {0, 0, 127}));
            annotation(
              Icon(coordinateSystem(grid = {2, 8})));
          end DIToBoolean_Partial;
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end BaseClasses;
      end Ideal;
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end Analog;

    package Digital
      extends Modelica.Icons.Package;

      package Basic
        extends Modelica.Icons.Package;

        model Xor "Xor logic component with multiple input and one output"
          extends Modelica.Electrical.Digital.Interfaces.MISO;
        protected
          Modelica.Electrical.Digital.Interfaces.Logic auxiliary[n](each start = Modelica.Electrical.Digital.Interfaces.Logic.'U');
          Modelica.Electrical.Digital.Interfaces.Logic auxiliary_n(start = Modelica.Electrical.Digital.Interfaces.Logic.'U', fixed = true);
        equation
          auxiliary[1] = x[1];
          for i in 1:n - 1 loop
            auxiliary[i + 1] = Modelica.Electrical.Digital.Tables.XorTable[auxiliary[i], x[i + 1]];
          end for;
          auxiliary_n = auxiliary[n];
          y = pre(auxiliary_n);
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end Xor;

        model Not "Not logic component without delay"
          extends Modelica.Electrical.Digital.Interfaces.SISO;
        protected
          Modelica.Electrical.Digital.Interfaces.Logic auxiliary(start = Modelica.Electrical.Digital.Interfaces.Logic.'0', fixed = true);
        equation
          auxiliary = Modelica.Electrical.Digital.Tables.NotTable[x];
          y = pre(auxiliary);
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end Not;

        model And "And logic component with multiple input and one output"
          extends Modelica.Electrical.Digital.Interfaces.MISO;
        protected
          Modelica.Electrical.Digital.Interfaces.Logic auxiliary[n](each start = Modelica.Electrical.Digital.Interfaces.Logic.'U');
          Modelica.Electrical.Digital.Interfaces.Logic auxiliary_n(start = Modelica.Electrical.Digital.Interfaces.Logic.'U', fixed = true);
        equation
          auxiliary[1] = x[1];
          for i in 1:n - 1 loop
            auxiliary[i + 1] = Modelica.Electrical.Digital.Tables.AndTable[auxiliary[i], x[i + 1]];
          end for;
          auxiliary_n = auxiliary[n];
          y = pre(auxiliary_n);
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end And;

        model Or "Or logic component with multiple input and one output"
          extends Modelica.Electrical.Digital.Interfaces.MISO;
        protected
          Modelica.Electrical.Digital.Interfaces.Logic auxiliary[n](each start = Modelica.Electrical.Digital.Interfaces.Logic.'U');
          Modelica.Electrical.Digital.Interfaces.Logic auxiliary_n(start = Modelica.Electrical.Digital.Interfaces.Logic.'U', fixed = true);
        equation
          auxiliary[1] = x[1];
          for i in 1:n - 1 loop
            auxiliary[i + 1] = Modelica.Electrical.Digital.Tables.OrTable[auxiliary[i], x[i + 1]];
          end for;
          auxiliary_n = auxiliary[n];
          y = pre(auxiliary_n);
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end Or;
        annotation(
          Icon(coordinateSystem(grid = {2, 8})));
      end Basic;
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end Digital;
    annotation(
      Documentation(info = "<html>
  <p>
  This library contains electrical components to build up analog and digital circuits,
  as well as machines to model electrical motors and generators,
  especially three phase induction machines such as an asynchronous motor.
  </p>
  
  </html>"),
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Rectangle(origin = {20.3125, 82.8571}, extent = {{-45.3125, -57.8571}, {4.6875, -27.8571}}), Line(origin = {8.0, 48.0}, points = {{32.0, -58.0}, {72.0, -58.0}}), Line(origin = {9.0, 54.0}, points = {{31.0, -49.0}, {71.0, -49.0}}), Line(origin = {-2.0, 55.0}, points = {{-83.0, -50.0}, {-33.0, -50.0}}), Line(origin = {-3.0, 45.0}, points = {{-72.0, -55.0}, {-42.0, -55.0}}), Line(origin = {1.0, 50.0}, points = {{-61.0, -45.0}, {-61.0, -10.0}, {-26.0, -10.0}}), Line(origin = {7.0, 50.0}, points = {{18.0, -10.0}, {53.0, -10.0}, {53.0, -45.0}}), Line(origin = {6.2593, 48.0}, points = {{53.7407, -58.0}, {53.7407, -93.0}, {-66.2593, -93.0}, {-66.2593, -58.0}})}));
  end Electrical;

  package Thermal
    extends Modelica.Icons.Package;

    package HeatTransfer
      extends Modelica.Icons.Package;

      package Sensors
        extends Modelica.Icons.SensorsPackage;

        model Thermostat
          extends Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor;
          parameter Modelica.SIunits.Temperature tLow, tHigh;
          parameter Boolean pre_y_start = false;
          Modelica.Blocks.Logical.Hysteresis hysteresis(uHigh = tHigh, uLow = tLow, pre_y_start = pre_y_start) annotation(
            Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.BooleanOutput y annotation(
            Placement(visible = true, transformation(origin = {130, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
          connect(hysteresis.y, y) annotation(
            Line(points = {{102, 0}, {122, 0}, {122, 0}, {130, 0}}, color = {255, 0, 255}));
          connect(T, hysteresis.u) annotation(
            Line(points = {{70, 0}, {78, 0}, {78, 0}, {78, 0}}, color = {0, 0, 127}));
          annotation(
            Icon(coordinateSystem(grid = {2, 8})));
        end Thermostat;
        annotation(
          Icon(coordinateSystem(grid = {2, 8})));
      end Sensors;
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end HeatTransfer;
    annotation(
      Icon(coordinateSystem(extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Line(origin = {-47.5, 11.6667}, points = {{-2.5, -91.6667}, {17.5, -71.6667}, {-22.5, -51.6667}, {17.5, -31.6667}, {-22.5, -11.667}, {17.5, 8.3333}, {-2.5, 28.3333}, {-2.5, 48.3333}}, smooth = Smooth.Bezier), Polygon(origin = {-50.0, 68.333}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{0.0, 21.667}, {-10.0, -8.333}, {10.0, -8.333}}), Line(origin = {2.5, 11.6667}, points = {{-2.5, -91.6667}, {17.5, -71.6667}, {-22.5, -51.6667}, {17.5, -31.6667}, {-22.5, -11.667}, {17.5, 8.3333}, {-2.5, 28.3333}, {-2.5, 48.3333}}, smooth = Smooth.Bezier), Polygon(origin = {0.0, 68.333}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{0.0, 21.667}, {-10.0, -8.333}, {10.0, -8.333}}), Line(origin = {52.5, 11.6667}, points = {{-2.5, -91.6667}, {17.5, -71.6667}, {-22.5, -51.6667}, {17.5, -31.6667}, {-22.5, -11.667}, {17.5, 8.3333}, {-2.5, 28.3333}, {-2.5, 48.3333}}, smooth = Smooth.Bezier), Polygon(origin = {50.0, 68.333}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{0.0, 21.667}, {-10.0, -8.333}, {10.0, -8.333}})}),
      Documentation(info = "<html>
  <p>
  This package contains libraries to model heat transfer
  and fluid heat flow.
  </p>
  </html>"));
  end Thermal;

  package Utilities
    extends Modelica.Icons.Package;

    block SignalNoise
      parameter Boolean enableNoise;
      parameter Real noiseAmplitude;
      Modelica.Blocks.Noise.TruncatedNormalNoise noiseGenerator(enableNoise = enableNoise, samplePeriod = 0.2, y_max = noiseAmplitude, y_min = -noiseAmplitude) annotation(
        Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add addNoiseToSignal annotation(
        Placement(visible = true, transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput y annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(addNoiseToSignal.y, y) annotation(
        Line(points = {{14, 0}, {102, 0}, {102, 0}, {110, 0}}, color = {0, 0, 127}));
      connect(noiseGenerator.y, addNoiseToSignal.u1) annotation(
        Line(points = {{-58, 50}, {-40, 50}, {-40, 6}, {-10, 6}, {-10, 6}}, color = {0, 0, 127}));
      connect(u, addNoiseToSignal.u2) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -6}, {-10, -6}, {-10, -6}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end SignalNoise;
    annotation(
      Icon(coordinateSystem(extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Polygon(origin = {1.3835, -4.1418}, rotation = 45.0, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-15.0, 93.333}, {-15.0, 68.333}, {0.0, 58.333}, {15.0, 68.333}, {15.0, 93.333}, {20.0, 93.333}, {25.0, 83.333}, {25.0, 58.333}, {10.0, 43.333}, {10.0, -41.667}, {25.0, -56.667}, {25.0, -76.667}, {10.0, -91.667}, {0.0, -91.667}, {0.0, -81.667}, {5.0, -81.667}, {15.0, -71.667}, {15.0, -61.667}, {5.0, -51.667}, {-5.0, -51.667}, {-15.0, -61.667}, {-15.0, -71.667}, {-5.0, -81.667}, {0.0, -81.667}, {0.0, -91.667}, {-10.0, -91.667}, {-25.0, -76.667}, {-25.0, -56.667}, {-10.0, -41.667}, {-10.0, 43.333}, {-25.0, 58.333}, {-25.0, 83.333}, {-20.0, 93.333}}), Polygon(origin = {10.1018, 5.218}, rotation = -45.0, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-15.0, 87.273}, {15.0, 87.273}, {20.0, 82.273}, {20.0, 27.273}, {10.0, 17.273}, {10.0, 7.273}, {20.0, 2.273}, {20.0, -2.727}, {5.0, -2.727}, {5.0, -77.727}, {10.0, -87.727}, {5.0, -112.727}, {-5.0, -112.727}, {-10.0, -87.727}, {-5.0, -77.727}, {-5.0, -2.727}, {-20.0, -2.727}, {-20.0, 2.273}, {-10.0, 7.273}, {-10.0, 17.273}, {-20.0, 27.273}, {-20.0, 82.273}})}));
  end Utilities;

  package Mechanics
    extends Modelica.Icons.Package;

    package Translational "Library to model 1-dimensional, translational mechanical systems"
      extends Modelica.Icons.Package;

      package Components "Components for 1D translational mechanical drive trains"
        extends Modelica.Icons.Package;

        model MassWithStops
          extends Modelica.Mechanics.Translational.Interfaces.PartialRigid(L = 0, s(start = 0, stateSelect = StateSelect.always));
          Modelica.SIunits.Velocity v(start = 0, stateSelect = StateSelect.always) "Absolute velocity of flange_a and flange_b";
          Modelica.SIunits.Acceleration a(start = 0) "Absolute acceleration of flange_a and flange_b";
          parameter Modelica.SIunits.Mass m(start = 1) "Mass";
          parameter Modelica.SIunits.Position smax(start = 25) "Right stop for (right end of) sliding mass";
          parameter Modelica.SIunits.Position smin(start = -25) "Left stop for (left end of) sliding mass";
          Integer stopped "Mode of stop (-1: hard stop at flange_a, 0: no stop, +1: hard stop at flange_b";
        equation
// Velocity and acceleration of flanges
          v = der(s);
          a = der(v);
// Equilibrium of forces
          0 = flange_a.f + flange_b.f - m * der(v);
          when initial() then
            assert(s > smin + L / 2 or s >= smin + L / 2 and v >= 0, "Error in initialization of hard stop. (s - L/2) must be >= smin\n" + "(s=" + String(s) + ", L=" + String(L) + ", smin=" + String(smin) + ")");
            assert(s < smax - L / 2 or s <= smax - L / 2 and v <= 0, "Error in initialization of hard stop. (s + L/2) must be <= smax\n" + "(s=" + String(s) + ", L=" + String(L) + ", smax=" + String(smax) + ")");
          end when;
// Define events for hard stops and reinitialize the state variables velocity v and position s
          stopped = if s <= smin + L / 2 then -1 else if s >= smax - L / 2 then +1 else 0;
          when stopped <> 0 then
            reinit(s, if stopped < 0 then smin + L / 2 else smax - L / 2);
            reinit(v, 0);
          end when;
/*
        Version 1:
        
        when not (s < smax - L/2) then
        reinit(s, smax - L/2);
        if (not initial() or v>0) then
          reinit(v, 0);
        end if;
        end when;
        
        when not (s > smin + L/2) then
        reinit(s, smin + L/2);
        if (not initial() or v<0) then
          reinit(v, 0);
        end if;
        end when;
        
        Version 2:
            stopped := if s <= smin + L/2 then -1 else if s >= smax - L/2 then +1 else 0;
          when (initial()) then
            assert(s > smin + L/2 or s >= smin + L/2 and v >= 0,
              "Error in initialization of hard stop. (s - L/2) must be >= smin\n"+
              "(s=" + String(s) + ", L=" + String(L) + ", smin=" + String(smin) + ")");
            assert(s < smax - L/2 or s <= smax - L/2 and v <= 0,
              "Error in initialization of hard stop. (s + L/2) must be <= smax\n"+
              "(s=" + String(s) + ", L=" + String(L) + ", smax=" + String(smax) + ")");
          end when;
          when stopped <> 0 then
            reinit(s, if stopped < 0 then smin + L/2 else smax - L/2);
            if (not initial() or stopped*v>0) then
               reinit(v, 0);
            end if;
          end when;
        */
          annotation(
            Documentation(info = "<html>
        <p>This element describes the <i>Stribeck friction characteristics</i> of a sliding mass,
        i. e. the frictional force acting between the sliding mass and the support. Included is a
        <i>hard stop</i> for the position. </p>
        <p>
        The surface is fixed and there is friction between sliding mass and surface.
        The frictional force f is given for positive velocity v by:
        </p>
        <blockquote><pre>
        f = F_Coulomb + F_prop * v + F_Stribeck * exp (-fexp * v)
        </pre></blockquote>
        
        <p>
        <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/Translational/Stribeck.png\">
        </p>
        
        <p>
        The distance between the left and the right connector is given by parameter L.
        The position of the center of gravity, coordinate s, is in the middle between
        the two flanges.</p>
        <p>
        There are hard stops at smax and smin, i. e. if
        <i><code>flange_a.s &gt;= smin</code></i> and <i><code>flange_b.s &lt;= xmax </code></i> the sliding mass can move freely.</p>
        <p>When the absolute velocity becomes zero, the sliding mass becomes stuck, i.e., the absolute position remains constant. In this phase the
        friction force is calculated from a force balance due to the requirement that the
        absolute acceleration shall be zero. The elements begin to slide when the friction
        force exceeds a threshold value, called the maximum static friction force, computed via:</p>
        <blockquote><pre>
        maximum_static_friction =  F_Coulomb + F_Stribeck
        </pre></blockquote>
        <p>
        <font color=\"#ff0000\"> <b>This requires the states Stop.s and Stop.v</b> </font>. If these states are eliminated during the index reduction
        the model will not work. To avoid this any inertias should be connected via springs
        to the Stop element, other sliding masses, dampers or hydraulic chambers must be avoided.</p>
        <p>For more details of the used friction model see the following reference: </p>
        
        <dl>
        <dt>Beater P. (1999):</dt>
        <dd><a href=\"http://www.springer.de/cgi-bin/search_book.pl?isbn=3-540-65444-5\">
        Entwurf hydraulischer Maschinen</a>. Springer Verlag Berlin Heidelberg New York.</dd>
        </dl>
        
        <p>The friction model is implemented in a \"clean\" way by state events and leads to
        continuous/discrete systems of equations which have to be solved by appropriate
        numerical methods. The method is described in
        (see also a short sketch in <a href=\"modelica://Modelica.Mechanics.Rotational.UsersGuide.ModelingOfFriction\">UsersGuide.ModelingOfFriction</a>):
        </p>
        
        <dl>
        <dt>Otter M., Elmqvist H., and Mattsson S.E. (1999):</dt>
        <dd><i>Hybrid Modeling in Modelica based on the Synchronous Data Flow Principle</i>.
        CACSD'99, Aug. 22.-26, Hawaii. </dd>
        </dl>
        
        <p>More precise friction models take into account the elasticity of the material when
        the two elements are \"stuck\", as well as other effects, like hysteresis. This has
        the advantage that the friction element can be completely described by a differential
        equation without events. The drawback is that the system becomes stiff (about 10-20 times
        slower simulation) and that more material constants have to be supplied which requires more
        sophisticated identification. For more details, see the following references, especially
        (Armstrong and Canudas de Witt 1996): </p>
        <dl>
        <dt>
        Armstrong B. (1991):</dt>
        <dd><i>Control of Machines with Friction</i>. Kluwer Academic Press, Boston MA.<br>
        </dd>
        <dt>Armstrong B., and Canudas de Wit C. (1996): </dt>
        <dd><i>Friction Modeling and Compensation.</i> The Control Handbook, edited by W.S.Levine, CRC Press, pp. 1369-1382.<br>
        </dd>
        <dt>Canudas de Wit C., Olsson H., Astroem K.J., and Lischinsky P. (1995): </dt>
        <dd><i>A new model for control of systems with friction.</i> IEEE Transactions on Automatic Control, Vol. 40, No. 3, pp. 419-425.<br>
        </dd>
        </dl>
        
        <h4>Optional heatPort</h4>
        <p>
        The dissipated energy is transported in form of heat to the optional heatPort connector
        that can be enabled via parameter \"useHeatPort\". Independently whether the heatPort is
        or is not enabled, the dissipated power is defined with variable \"lossPower\".
        If contact occurs at the hard stops, the lossPower is not correctly modelled
        at this time instant, because the hard stop would introduce a Dirac impulse
        in the lossPower due to the discontinuously changing kinetic energy of the mass
        (lossPower is the derivative of the kinetic energy at the time instant of the impact).
        </p>
        
        </html>", revisions = "<html>
        <h4>Release Notes:</h4>
        <ul>
        <li><i>First Version from December 7, 1999 by P. Beater (based on Rotational.BearingFriction)</i> </li>
        <li><i>July 14, 2001 by P. Beater, assert on initialization added, diagram modified</i> </li>
        <li><i>October 11, 2001, by Hans Olsson, Dassault Syst&egrave;mes AB, modified assert to handle start at stops,
        modified event logic such if you have friction parameters equal to zero you do not get events
        between the stops.</i> </li>
        <li><i>June 10, 2002 by P. Beater, StateSelect.always for variables s and v (instead of fixed=true). </i> </li>
        </ul>
        </html>"),
            Icon(coordinateSystem(initialScale = 0.1), graphics = {Polygon(lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{80, -100}, {50, -90}, {50, -110}, {80, -100}}), Line(points = {{-30, -100}, {50, -100}}), Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-30, 30}, {35, -35}}), Line(points = {{-90, 0}, {-30, 0}}, color = {0, 127, 0}), Rectangle(fillPattern = FillPattern.Solid, extent = {{-63, -15}, {-55, -45}}), Rectangle(fillPattern = FillPattern.Solid, extent = {{60, -16}, {69, -45}}), Line(points = {{35, 0}, {90, 0}}, color = {0, 127, 0}), Text(lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 40}}, textString = "%name"), Text(extent = {{-150, -110}, {150, -140}}, textString = "m=%m"), Line(visible = false, points = {{-100, -100}, {-100, -40}, {3, -40}}, color = {191, 0, 0}, pattern = LinePattern.Dot)}),
            Diagram(coordinateSystem(initialScale = 0.1), graphics = {Polygon(lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{50, -75}, {20, -65}, {20, -85}, {50, -75}}), Line(points = {{-60, -75}, {20, -75}}), Rectangle(origin = {0, -8}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-30, 26}, {35, -9}}), Line(points = {{-90, 0}, {-30, 0}}, color = {0, 127, 0}), Line(points = {{35, 0}, {90, 0}}, color = {0, 127, 0}), Rectangle(fillPattern = FillPattern.Solid, extent = {{-119, 43}, {-111, 17}}), Line(points = {{-111, 43}, {-111, 50}}), Line(points = {{-151, 49}, {-113, 49}}), Text(lineColor = {0, 0, 255}, extent = {{-149, 51}, {-126, 60}}, textString = "s min"), Polygon(fillPattern = FillPattern.Solid, points = {{-121, 52}, {-111, 49}, {-121, 46}, {-121, 52}}), Rectangle(fillPattern = FillPattern.Solid, extent = {{124, 42}, {132, 17}}), Line(points = {{124, 39}, {124, 87}}), Line(points = {{-19, 78}, {121, 78}}), Text(lineColor = {0, 0, 255}, extent = {{-17, 83}, {6, 92}}, textString = "s max"), Polygon(fillPattern = FillPattern.Solid, points = {{114, 81}, {124, 78}, {114, 75}, {114, 81}}), Line(origin = {-4.96923, 0}, points = {{5, 18}, {5, 63}}), Line(origin = {-4.96923, 0}, points = {{-77, 58}, {-1, 58}}), Text(origin = {-4, 0}, lineColor = {0, 0, 255}, extent = {{-75, 60}, {-38, 71}}, textString = "Position s"), Polygon(origin = {-4, 0}, fillPattern = FillPattern.Solid, points = {{-5, 61}, {5, 58}, {-5, 55}, {-5, 61}}), Line(points = {{-100, -10}, {-100, -60}}), Line(points = {{100, -10}, {100, -60}}), Polygon(fillPattern = FillPattern.Solid, points = {{90, -47}, {100, -50}, {90, -53}, {90, -47}}), Polygon(fillPattern = FillPattern.Solid, points = {{-90, -47}, {-90, -53}, {-100, -50}, {-90, -47}}), Line(points = {{-90, -50}, {92, -50}}), Text(lineColor = {0, 0, 255}, extent = {{-11, -46}, {26, -36}}, textString = "Length L")}));
        end MassWithStops;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {11.5, 31.183}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-67, -66}, {44, -6}})}),
          Documentation(info = "<html>
      <p>
      This package contains basic components 1D mechanical translational drive trains.
      </p>
      </html>"));
      end Components;
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(origin = {14, 53}, points = {{-84, -73}, {66, -73}}), Rectangle(origin = {14, 53}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Sphere, extent = {{-81, -65}, {-8, -22}}), Line(origin = {14, 53}, points = {{-8, -43}, {-1, -43}, {6, -64}, {17, -23}, {29, -65}, {40, -23}, {50, -44}, {61, -44}}), Line(origin = {14, 53}, points = {{-59, -73}, {-84, -93}}), Line(origin = {14, 53}, points = {{-11, -73}, {-36, -93}}), Line(origin = {14, 53}, points = {{-34, -73}, {-59, -93}}), Line(origin = {14, 53}, points = {{14, -73}, {-11, -93}}), Line(origin = {14, 53}, points = {{39, -73}, {14, -93}}), Line(origin = {14, 53}, points = {{63, -73}, {38, -93}})}),
        Documentation(info = "<html>
    <p>
    This package contains components to model <i>1-dimensional translational
    mechanical</i> systems.
    </p>
    <p>
    The <i>filled</i> and <i>non-filled green squares</i> at the left and
    right side of a component represent <i>mechanical flanges</i>.
    Drawing a line between such squares means that the corresponding
    flanges are <i>rigidly attached</i> to each other. The components of this
    library can be usually connected together in an arbitrary way. E.g. it is
    possible to connect two springs or two sliding masses with inertia directly
    together.
    </p>
    <p> The only <i>connection restriction</i> is that the Coulomb friction
    elements (e.g., MassWithStopAndFriction) should be only connected
    together provided a compliant element, such as a spring, is in between.
    The reason is that otherwise the frictional force is not uniquely
    defined if the elements are stuck at the same time instant (i.e., there
    does not exist a unique solution) and some simulation systems may not be
    able to handle this situation, since this leads to a singularity during
    simulation. It can only be resolved in a \"clean way\" by combining the
    two connected friction elements into
    one component and resolving the ambiguity of the frictional force in the
    stuck mode.
    </p>
    <p> Another restriction arises if the hard stops in model MassWithStopAndFriction are used, i. e.
    the movement of the mass is limited by a stop at smax or smin.
    <font color=\"#ff0000\"> <b>This requires the states Stop.s and Stop.v</b> </font>. If these states are eliminated during the index reduction
    the model will not work. To avoid this any inertias should be connected via springs
    to the Stop element, other sliding masses, dampers or hydraulic chambers must be avoided.</p>
    <p>
    In the <i>icon</i> of every component an <i>arrow</i> is displayed in grey
    color. This arrow characterizes the coordinate system in which the vectors
    of the component are resolved. It is directed into the positive
    translational direction (in the mathematical sense).
    In the flanges of a component, a coordinate system is rigidly attached
    to the flange. It is called <i>flange frame</i> and is directed in parallel
    to the component coordinate system. As a result, e.g., the positive
    cut-force of a \"left\" flange (flange_a) is directed into the flange, whereas
    the positive cut-force of a \"right\" flange (flange_b) is directed out of the
    flange. A flange is described by a Modelica connector containing
    the following variables:
    </p>
    <pre>
       Modelica.SIunits.Position s    \"Absolute position of flange\";
       <b>flow</b> Modelica.SIunits.Force f  \"Cut-force in the flange\";
    </pre>
    
    <p>
    This library is designed in a fully object oriented way in order that
    components can be connected together in every meaningful combination
    (e.g., direct connection of two springs or two shafts with inertia).
    As a consequence, most models lead to a system of
    differential-algebraic equations of <i>index 3</i> (= constraint
    equations have to be differentiated twice in order to arrive at
    a state space representation) and the Modelica translator or
    the simulator has to cope with this system representation.
    According to our present knowledge, this requires that the
    Modelica translator is able to symbolically differentiate equations
    (otherwise it is e.g., not possible to provide consistent initial
    conditions; even if consistent initial conditions are present, most
    numerical DAE integrators can cope at most with index 2 DAEs).
    </p>
    
    <p>
    In version 3.2 of the Modelica Standard Library, all <b>dissipative</b> components
    of the Translational library got an optional <b>heatPort</b> connector to which the
    dissipated energy is transported in form of heat. This connector is enabled
    via parameter \"useHeatPort\". If the heatPort connector is enabled,
    it must be connected, and if it is not enabled, it must not be connected.
    Independently, whether the heatPort is enabled or not,
    the dissipated power is available from the new variable \"<b>lossPower</b>\" (which is
    positive if heat is flowing out of the heatPort). For an example, see
    <a href=\"modelica://Modelica.Mechanics.Translational.Examples.HeatLosses\">Examples.HeatLosses</a>.
    </p>
    
    <dl>
    <dt><b>Library Officer</b></dt>
    <dd><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> <br>
        Deutsches Zentrum f&uuml;r Luft und Raumfahrt e.V. (DLR)<br>
        Institut f&uuml;r Robotik und Mechatronik (DLR-RM)<br>
        Abteilung Systemdynamik und Regelungstechnik<br>
        Postfach 1116<br>
        D-82230 Wessling<br>
        Germany<br>
        email: <A HREF=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</A><br><br></dd>
    </dl>
    
    <p>
    <b>Contributors to this library:</b>
    </p>
    
    <ul>
    <li> Main author until 2006:<br>
         Peter Beater <br>
         Universit&auml;t Paderborn, Abteilung Soest<br>
         Fachbereich Maschinenbau/Automatisierungstechnik<br>
         L&uuml;becker Ring 2 <br>
         D 59494 Soest <br>
         Germany <br>
         email: <A HREF=\"mailto:info@beater.de\">info@beater.de</A><br><br>
         </li>
    
    <li> <a href=\"http://www.haumer.at/\">Anton Haumer</a><br>
         Technical Consulting &amp; Electrical Engineering<br>
         A-3423 St.Andrae-Woerdern, Austria<br>
         email: <a href=\"mailto:a.haumer@haumer.at\">a.haumer@haumer.at</a><br><br></li>
    
    <li> <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> (DLR-RM)</li>
    </ul>
    
    <p>
    Copyright &copy; 1998-2016, Modelica Association, Anton Haumer and Universit&auml;t Paderborn, FB 12.
    </p>
    <p>
    <i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\"> https://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
    </p>
    </html>", revisions = "<html>
    <ul>
    <li><i>Version 1.2.0 2010-07-22</i>
           by Anton Haumer and Martin Otter<br>
           heatPort introduced for all dissipative elements, and
           text in icons improved.
           <br></li>
    
    <li><i>Version 1.1.0 2007-11-16</i>
           by Anton Haumer<br>
           Redesign for Modelica 3.0-compliance<br>
           Added new components according to Mechanics.Rotational library
           <br></li>
    
    <li><i>Version 1.01 (July 18, 2001)</i>
           by Peter Beater <br>
           Assert statement added to \"Stop\", small bug fixes in examples.
           <br></li>
    
    <li><i>Version 1.0 (January 5, 2000)</i>
           by Peter Beater <br>
           Realized a first version based on Modelica library Mechanics.Rotational
           by Martin Otter and an existing Dymola library onedof.lib by Peter Beater.</li>
    </ul>
    </html>"));
    end Translational;
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Rectangle(origin = {8.6, 63.3333}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-4.6, -93.3333}, {41.4, -53.3333}}), Ellipse(origin = {9.0, 46.0}, extent = {{-90.0, -60.0}, {-80.0, -50.0}}), Line(origin = {9.0, 46.0}, points = {{-85.0, -55.0}, {-60.0, -21.0}}, thickness = 0.5), Ellipse(origin = {9.0, 46.0}, extent = {{-65.0, -26.0}, {-55.0, -16.0}}), Line(origin = {9.0, 46.0}, points = {{-60.0, -21.0}, {9.0, -55.0}}, thickness = 0.5), Ellipse(origin = {9.0, 46.0}, fillPattern = FillPattern.Solid, extent = {{4.0, -60.0}, {14.0, -50.0}}), Line(origin = {9.0, 46.0}, points = {{-10.0, -26.0}, {72.0, -26.0}, {72.0, -86.0}, {-10.0, -86.0}})}),
      Documentation(info = "<html>
  <p>
  This package contains components to model the movement
  of 1-dim. rotational, 1-dim. translational, and
  3-dim. <b>mechanical systems</b>.
  </p>
  
  <p>
  Note, all <b>dissipative</b> components of the Modelica.Mechanics library have
  an optional <b>heatPort</b> connector to which the
  dissipated energy is transported in form of heat. This connector is enabled
  via parameter \"useHeatPort\". If the heatPort connector is enabled,
  it must be connected, and if it is not enabled, it must not be connected.
  Independently, whether the heatPort is enabled or not,
  the dissipated power is available from variable \"<b>lossPower</b>\" (which is
  positive if heat is flowing out of the heatPort).
  </p>
  </html>"));
  end Mechanics;

  package Constants
    extends Modelica.Icons.Package;
    annotation(
      Icon(coordinateSystem(extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Polygon(origin = {-9.2597, 25.6673}, fillColor = {102, 102, 102}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{48.017, 11.336}, {48.017, 11.336}, {10.766, 11.336}, {-25.684, 10.95}, {-34.944, -15.111}, {-34.944, -15.111}, {-32.298, -15.244}, {-32.298, -15.244}, {-22.112, 0.168}, {11.292, 0.234}, {48.267, -0.097}, {48.267, -0.097}}, smooth = Smooth.Bezier), Polygon(origin = {-19.9923, -8.3993}, fillColor = {102, 102, 102}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{3.239, 37.343}, {3.305, 37.343}, {-0.399, 2.683}, {-16.936, -20.071}, {-7.808, -28.604}, {6.811, -22.519}, {9.986, 37.145}, {9.986, 37.145}}, smooth = Smooth.Bezier), Polygon(origin = {23.753, -11.5422}, fillColor = {102, 102, 102}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-10.873, 41.478}, {-10.873, 41.478}, {-14.048, -4.162}, {-9.352, -24.8}, {7.912, -24.469}, {16.247, 0.27}, {16.247, 0.27}, {13.336, 0.071}, {13.336, 0.071}, {7.515, -9.983}, {-3.134, -7.271}, {-2.671, 41.214}, {-2.671, 41.214}}, smooth = Smooth.Bezier)}));
  end Constants;
  annotation(
    Icon(coordinateSystem(initialScale = 0.1), graphics = {Polygon(origin = {-6.9888, 20.048}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-93.0112, 10.3188}, {-93.0112, 10.3188}, {-73.011, 24.6}, {-63.011, 31.221}, {-51.219, 36.777}, {-39.842, 38.629}, {-31.376, 36.248}, {-25.819, 29.369}, {-24.232, 22.49}, {-23.703, 17.463}, {-15.501, 25.135}, {-6.24, 32.015}, {3.02, 36.777}, {15.191, 39.423}, {27.097, 37.306}, {32.653, 29.633}, {35.035, 20.108}, {43.501, 28.046}, {54.085, 35.19}, {65.991, 39.952}, {77.897, 39.688}, {87.422, 33.338}, {91.126, 21.696}, {90.068, 9.525}, {86.099, -1.058}, {79.749, -10.054}, {71.283, -21.431}, {62.816, -33.337}, {60.964, -32.808}, {70.489, -16.14}, {77.368, -2.381}, {81.072, 10.054}, {79.749, 19.05}, {72.605, 24.342}, {61.758, 23.019}, {49.587, 14.817}, {39.003, 4.763}, {29.214, -6.085}, {21.012, -16.669}, {13.339, -26.458}, {5.401, -36.777}, {-1.213, -46.037}, {-6.24, -53.446}, {-8.092, -52.387}, {-0.684, -40.746}, {5.401, -30.692}, {12.81, -17.198}, {19.424, -3.969}, {23.658, 7.938}, {22.335, 18.785}, {16.514, 23.283}, {8.047, 23.019}, {-1.478, 19.05}, {-11.267, 11.113}, {-19.734, 2.381}, {-29.259, -8.202}, {-38.519, -19.579}, {-48.044, -31.221}, {-56.511, -43.392}, {-64.449, -55.298}, {-72.386, -66.939}, {-77.678, -74.612}, {-79.53, -74.083}, {-71.857, -61.383}, {-62.861, -46.037}, {-52.278, -28.046}, {-44.869, -15.346}, {-38.784, -2.117}, {-35.344, 8.731}, {-36.403, 19.844}, {-42.488, 23.813}, {-52.013, 22.49}, {-60.744, 16.933}, {-68.947, 10.054}, {-76.884, 2.646}, {-93.0112, -12.1707}, {-93.0112, -12.1707}, {-93.0112, 10.3188}}, smooth = Smooth.Bezier), Line(origin = {51.9539, -62.7531}, points = {{19, 39}, {-13, -11}}, color = {170, 0, 0}, thickness = 6, smooth = Smooth.Bezier), Line(origin = {10.5838, -61.8031}, points = {{25, 37}, {41, 17}}, color = {170, 0, 0}, thickness = 6, smooth = Smooth.Bezier)}),
    uses(Modelica(version = "3.2.2")));
end MyLib;
