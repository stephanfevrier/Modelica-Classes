package DE
  // ---------- Definitions ----------
  // ---------- Inheritances ----------
  // ---------- Declarations ----------
  // ---------- Connectors ----------
  // ---------- Equations ----------
  // ---------- Graphics & Layout ----------

  model EN253pipe
    // ---------- Definitions ----------
    // ---------- Inheritances ----------
    extends Modelica.Fluid.Pipes.DynamicPipe(
  	redeclare replaceable package Medium = DH_Medium, 
  	redeclare replaceable model HeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.IdealFlowHeatTransfer constrainedby Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.PartialFlowHeatTransfer, 
  	roughness = globalDHconfig.roughness, 
  	heatTransfer.k = lambda_insul * 2 * Modelica.Constants.pi / Modelica.Math.log((outsideDiaInsul / 2) / (outsideDiaPipe / 2)), 
  	diameter = insideDiaPipe, 
  	use_HeatTransfer = true);
  
    // ---------- Declarations ----------
    replaceable package DH_Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    parameter DE.PipeSpec.InsulClass insulClass = globalDHconfig.insulClass annotation(Dialog(tab = "General", group = "EN253 Pipe Type"));
    parameter DE.PipeSpec.DNtype DN annotation(Dialog(tab = "General", group = "EN253 Pipe Type"));
    parameter Modelica.SIunits.ThermalConductivity lambda_insul = 0.03 annotation(Dialog(tab = "General", group = "EN253 Pipe Type"));
    protected parameter Modelica.SIunits.Diameter insideDiaPipe = outsideDiaPipe - 2 * pipeThickness;
    protected parameter Modelica.SIunits.Diameter outsideDiaPipe = DE.SpecTables.EN253Spec[insulClass, DN, 1];
    protected parameter Modelica.SIunits.Radius pipeThickness = DE.SpecTables.EN253Spec[insulClass, DN, 2];
    protected parameter Modelica.SIunits.Diameter outsideDiaInsul = DE.SpecTables.EN253Spec[insulClass, DN, 3];
    protected outer parameter DE.GlobalDHconfig globalDHconfig annotation(Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // ---------- Components ----------
// ---------- Connectors ----------
// ---------- Equations ----------
// ---------- Graphics & Layout ----------
    annotation(Icon(coordinateSystem(grid = {2, 8}, initialScale = 0.1), graphics = {Rectangle(origin = {-1, 60}, fillColor = {240, 240, 119}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-99, 4}, {101, -20}}), Rectangle(origin = {0, -60}, fillColor = {240, 240, 119}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-100, 20}, {100, -4}}), Text(origin = {0, 80}, extent = {{-64, 16}, {64, -16}}, textString = "EN253 Pipe", fontName = "DejaVu Sans Mono")}), Diagram);
  end EN253pipe;

  record PipeSpec
    type InsulClass = enumeration(Class_1, Class_2, Class_3);
    type DNtype = enumeration(DN20, DN25, DN32, DN40, DN50, DN65, DN80, DN100, DN125, DN150, DN200, DN250, DN300, DN350, DN400, DN450, DN500, DN600, DN700, DN800, DN900, DN1000);
    annotation(Icon(coordinateSystem(grid = {2, 8})));
  end PipeSpec;

  package SpecTables
    final constant Real EN253Spec[DE.PipeSpec.InsulClass, DE.PipeSpec.DNtype, 3] = {{{0.0269, 0.0026, 0.090}, {0.0337, 0.0026, 0.090}, {0.0424, 0.0026, 0.110}, {0.0483, 0.0026, 0.110}, {0.0603, 0.0029, 0.125}, {0.0761, 0.0029, 0.140}, {0.0889, 0.0032, 0.160}, {0.1143, 0.0036, 0.200}, {0.1397, 0.0036, 0.225}, {0.1683, 0.0040, 0.250}, {0.2191, 0.0045, 0.315}, {0.273, 0.0050, 0.400}, {0.3239, 0.0056, 0.450}, {0.3556, 0.0056, 0.500}, {0.4064, 0.0063, 0.560}, {0.4572, 0.0063, 0.630}, {0.508, 0.0063, 0.710}, {0.610, 0.0071, 0.800}, {0.711, 0.008, 0.900}, {0.813, 0.0088, 1.000}, {0.914, 0.01, 1.100}, {1.016, 0.011, 1.200}}, {{0.0269, 0.0026, 0.110}, {0.0337, 0.0026, 0.110}, {0.0424, 0.0026, 0.125}, {0.0483, 0.0026, 0.125}, {0.0603, 0.0029, 0.140}, {0.0761, 0.0029, 0.160}, {0.0889, 0.0032, 0.180}, {0.1143, 0.0036, 0.225}, {0.1397, 0.0036, 0.250}, {0.1683, 0.0040, 0.280}, {0.2191, 0.0045, 0.355}, {0.273, 0.0050, 0.450}, {0.3239, 0.0056, 0.500}, {0.3556, 0.0056, 0.560}, {0.4064, 0.0063, 0.560}, {0.4572, 0.0063, 0.630}, {0.508, 0.0063, 0.710}, {0.610, 0.0071, 0.800}, {0.711, 0.008, 0.900}, {0.813, 0.0088, 1.000}, {0.914, 0.01, 1.100}, {1.016, 0.011, 1.200}}, {{0.0269, 0.0026, 0.125}, {0.0337, 0.0026, 0.125}, {0.0424, 0.0026, 0.140}, {0.0483, 0.0026, 0.140}, {0.0603, 0.0029, 0.160}, {0.0761, 0.0029, 0.180}, {0.0889, 0.0032, 0.200}, {0.1143, 0.0036, 0.250}, {0.1397, 0.0036, 0.280}, {0.1683, 0.0040, 0.315}, {0.2191, 0.0045, 0.400}, {0.273, 0.0050, 0.500}, {0.3239, 0.0056, 0.560}, {0.3556, 0.0056, 0.560}, {0.4064, 0.0063, 0.630}, {0.4572, 0.0063, 0.710}, {0.508, 0.0063, 0.800}, {0.610, 0.0071, 0.800}, {0.711, 0.008, 0.900}, {0.813, 0.0088, 1.000}, {0.914, 0.01, 1.100}, {1.016, 0.011, 1.200}}};
  
    annotation(Icon(coordinateSystem(grid = {2, 8})));
  end SpecTables;

  model ETS
    // ---------- Definitions ----------
    // ---------- Inheritances ----------
    // ---------- Declarations ----------
    replaceable package PrimaryMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    replaceable package SecondaryMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    replaceable model HexHeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer constrainedby Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.PartialFlowHeatTransfer;
    replaceable model HexFlowModel = Modelica.Fluid.Pipes.BaseClasses.FlowModels.DetailedPipeFlow constrainedby Modelica.Fluid.Pipes.BaseClasses.FlowModels.PartialStaggeredFlowModel;
    parameter Integer hexChannels annotation(Dialog(tab = "General", group = "Geometry"));
    parameter Modelica.SIunits.Length hexChannelLength annotation(Dialog(tab = "General", group = "Geometry"));
    parameter Modelica.SIunits.Diameter hexChannelDiameter annotation(Dialog(tab = "General", group = "Geometry"));
    parameter Modelica.SIunits.Length hexWallThickness annotation(Dialog(tab = "General", group = "Geometry"));
    parameter Modelica.SIunits.PressureDifference valve_dp_nom(displayUnit="Pa") annotation(Dialog(tab = "Advanced", group = "Characteristics"));
    parameter Modelica.SIunits.MassFlowRate valve_flow_nom annotation(Dialog(tab = "Advanced", group = "Characteristics"));
    parameter Modelica.SIunits.ThermalConductivity hexWallConductivity = 16.5 annotation(Dialog(tab = "Advanced", group = "Characteristics"));
    parameter Modelica.SIunits.CoefficientOfHeatTransfer hexAlpha = 4000 annotation(Dialog(tab = "Advanced", group = "Characteristics"));
    parameter Modelica.SIunits.MassFlowRate pump_flow_nom annotation(Dialog(tab = "Advanced", group = "Characteristics"));
    parameter Modelica.SIunits.AbsolutePressure pumpInlet_press_nom(displayUnit="Pa") annotation(Dialog(tab = "Advanced", group = "Characteristics"));
    parameter Modelica.SIunits.AbsolutePressure pumpOutlet_press_nom(displayUnit="Pa") annotation(Dialog(tab = "Advanced", group = "Characteristics"));
    parameter Real PID_Gain = 0.5 annotation(Dialog(tab = "Advanced", group = "System Behaviour"));
    parameter Real PID_Ti = 120 annotation(Dialog(tab = "Advanced", group = "System Behaviour"));
    parameter Real PID_Td = 0 annotation(Dialog(tab = "Advanced", group = "System Behaviour"));
    // ---------- Components ----------
    Modelica.Fluid.Pipes.DynamicPipe primaryPipe(redeclare replaceable package Medium = PrimaryMedium, redeclare replaceable model FlowModel = HexFlowModel, redeclare replaceable model HeatTransfer = HexHeatTransfer, heatTransfer.alpha0 = hexAlpha, diameter = hexChannelDiameter, length = hexChannelLength, nParallel = hexChannels, use_HeatTransfer = true) annotation(Placement(visible = true, transformation(origin = {-30, -20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Fluid.Pipes.DynamicPipe secondaryPipe(redeclare replaceable package Medium = SecondaryMedium, redeclare replaceable model FlowModel = HexFlowModel, redeclare replaceable model HeatTransfer = HexHeatTransfer, heatTransfer.alpha0 = hexAlpha, diameter = hexChannelDiameter, length = hexChannelLength, nParallel = hexChannels, use_HeatTransfer = true) annotation(Placement(visible = true, transformation(origin = {30, -20}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor HExMaterial(G = hexWallConductivity * hexWallThickness) annotation(Placement(visible = true, transformation(origin = {0, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Valves.ValveIncompressible valve(redeclare replaceable package Medium = PrimaryMedium, CvData = Modelica.Fluid.Types.CvTypes.OpPoint, dp(displayUnit = "Pa"), dp_nominal(displayUnit = "Pa") = valve_dp_nom, m_flow_nominal = valve_flow_nom) annotation(Placement(visible = true, transformation(extent = {{-50, -80}, {-70, -60}}, rotation = 0)));
    Modelica.Fluid.Machines.ControlledPump pump(redeclare replaceable package Medium = SecondaryMedium, checkValve = true, m_flow_nominal = pump_flow_nom, p_a_nominal = pumpInlet_press_nom, p_b_nominal = pumpOutlet_press_nom, use_m_flow_set = true) annotation(Placement(visible = true, transformation(origin = {60, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Fluid.Interfaces.FluidPort_a primaryHWS(redeclare replaceable package Medium = PrimaryMedium) annotation(Placement(visible = true, transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Interfaces.FluidPort_b primaryHWR(redeclare replaceable package Medium = PrimaryMedium) annotation(Placement(visible = true, transformation(origin = {-90, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Interfaces.FluidPort_b secondaryHWS(redeclare replaceable package Medium = SecondaryMedium) annotation(Placement(visible = true, transformation(origin = {90, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Fluid.Interfaces.FluidPort_a secondaryHWR(redeclare replaceable package Medium = SecondaryMedium) annotation(Placement(visible = true, transformation(origin = {90, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, -14}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Fluid.Sensors.Temperature HWS_TE(redeclare replaceable package Medium = SecondaryMedium) annotation(Placement(visible = true, transformation(origin = {60, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimPID TC_PID(Td = PID_Td, Ti = PID_Ti, k = PID_Gain, yMax = 100, yMin = 0) annotation(Placement(visible = true, transformation(origin = {0, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput HWS_temp_SP annotation(Placement(visible = true, transformation(origin = {90, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, 80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput pumpMassFlowIn annotation(Placement(visible = true, transformation(origin = {90, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
// ---------- Connectors ----------
    connect(secondaryHWR, pump.port_a) annotation(Line(points = {{90, -70}, {70, -70}}));
    connect(pump.port_b, secondaryPipe.port_a) annotation(Line(points = {{50, -70}, {30, -70}, {30, -30}}, color = {0, 127, 255}));
    connect(secondaryPipe.port_b, HWS_TE.port) annotation(Line(points = {{30, -10}, {30, 30}, {60, 30}, {60, 40}}, color = {0, 127, 255}));
    connect(HWS_TE.port, secondaryHWS) annotation(Line(points = {{60, 40}, {60, 30}, {90, 30}, {90, 30}}, color = {0, 127, 255}));
    connect(HExMaterial.port_b, secondaryPipe.heatPorts[1]) annotation(Line(points = {{10, -20}, {26, -20}}, color = {191, 0, 0}));
    connect(primaryPipe.heatPorts[1], HExMaterial.port_a) annotation(Line(points = {{-25.6, -20.1}, {-9.6, -20.1}}, color = {127, 0, 0}));
    connect(primaryHWS, primaryPipe.port_a) annotation(Line(points = {{-90, 30}, {-30, 30}, {-30, -10}}));
    connect(primaryPipe.port_b, valve.port_a) annotation(Line(points = {{-30, -30}, {-30, -70}, {-50, -70}}, color = {0, 127, 255}));
    connect(valve.port_b, primaryHWR) annotation(Line(points = {{-70, -70}, {-90, -70}}, color = {0, 127, 255}));
  connect(pumpMassFlowIn, pump.m_flow_set) annotation(Line(points = {{90, -30}, {64, -30}, {64, -62}, {66, -62}}, color = {0, 0, 127}));
    connect(HWS_temp_SP, TC_PID.u_s) annotation(Line(points = {{90, 70}, {12, 70}}, color = {0, 0, 127}));
    connect(TC_PID.y, valve.opening) annotation(Line(points = {{-11, 70}, {-11, 70.5}, {-60, 70.5}, {-60, -62}}, color = {0, 0, 127}));
    connect(HWS_TE.T, TC_PID.u_m) annotation(Line(points = {{54, 50}, {0, 50}, {0, 58}}, color = {0, 0, 127}));
  // ---------- Equations ----------
// ---------- Graphics & Layout ----------
    annotation(Icon(graphics = {Rectangle(origin = {-19, 16}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}), Line(origin = {5.27, 16}, points = {{74.7293, 32}, {-55.2707, 32}, {6.72932, 0}, {-55.2707, -32}, {74.7293, -32}}, thickness = 1), Line(origin = {-24.27, -52}, points = {{-3.72703, 12}, {12.273, 12}, {-3.72703, -12}, {12.273, -12}, {-3.72703, 12}}, thickness = 1), Line(origin = {-20, -32}, points = {{0, 8}, {0, -8}}, thickness = 1), Line(origin = {-60, -72}, points = {{40, 8}, {40, -8}, {-20, -8}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {-60, 68}, points = {{40, -12}, {40, 12}, {-20, 12}}, thickness = 1, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 10), Ellipse(origin = {60, 80}, lineThickness = 1, extent = {{-15, 15}, {15, -15}}, endAngle = 360), Line(origin = {60, 52}, points = {{0, 12}, {0, -4}}, thickness = 1), Text(origin = {60, 80}, lineThickness = 1, extent = {{-10, 10}, {10, -10}}, textString = "TC", fontName = "DejaVu Sans Mono"), Ellipse(origin = {50, -16}, lineThickness = 1, extent = {{-15, 15}, {15, -15}}, endAngle = 360), Line(origin = {45.33, -16}, points = {{4.67075, -15}, {-9.32925, 0}, {4.67075, 15}}, thickness = 1), Text(origin = {0, 125}, lineColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", fontName = "DejaVu Sans Mono")}), uses(Modelica(version = "3.2.1")), Diagram, version = "");
  end ETS;

  model DH_ETS
    // ---------- Definitions and Inheritances ----------
    extends DE.ETS(
  	redeclare replaceable package PrimaryMedium = DH_Medium, 
  	redeclare replaceable package SecondaryMedium = BldgMedium, 
  	hexChannels = integer(hexArea / hexChannelArea), 
  	hexChannelDiameter = 0.011, 
  	hexChannelLength = hexChannelArea / hexChannelDiameter, 
  	hexWallThickness = 0.0004, 
  	valve_dp_nom = 30000, 
  	valve_flow_nom = (spaceHeatCapacity + dhwHeatCapacity + processHeatCapacity) / (bldgMedium_cp * (globalDHconfig.bldgHWS_temp - globalDHconfig.bldgHWR_temp)));
  // ---------- Declarations ----------
    replaceable package DH_Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    replaceable package BldgMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    parameter Modelica.SIunits.Power spaceHeatCapacity annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Modelica.SIunits.Power dhwHeatCapacity annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Modelica.SIunits.Power processHeatCapacity annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Modelica.SIunits.Temperature hexLMTD = globalDHconfig.hexLMTD annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Modelica.SIunits.Area hexArea = (spaceHeatCapacity + dhwHeatCapacity + processHeatCapacity) / (hexLMTD * (2 * hexAlpha + hexWallConductivity * hexWallThickness)) annotation(Dialog(tab = "General", group = "Geometry"));
    parameter Modelica.SIunits.Area hexChannelArea = 0.67 annotation(Dialog(tab = "Assumptions", group = "Geometry"));
    protected parameter Modelica.SIunits.SpecificHeatCapacity bldgMedium_cp(fixed=false) = BldgMedium.BaseProperties.specificHeatCapacityCp(bldgMediumState);
    protected parameter BldgMedium.ThermodynamicState bldgMediumState;
    outer DE.GlobalDHconfig globalDHconfig annotation(Placement(visible = true, transformation(origin = {250, 90}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  // ---------- Components ----------
    Modelica.Fluid.Sources.FixedBoundary bldgHWS(redeclare replaceable package Medium = BldgMedium, T = globalDHconfig.bldgHWS_temp, nPorts = 1) annotation(Placement(visible = true, transformation(origin = {250, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Fluid.Sources.FixedBoundary bldgHWR(redeclare replaceable package Medium = BldgMedium, T = globalDHconfig.bldgHWR_temp, nPorts = 1) annotation(Placement(visible = true, transformation(origin = {250, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MultiSum loadCalc(k = array(spaceHeatCapacity, dhwHeatCapacity, processHeatCapacity), nu = 3) annotation(Placement(visible = true, transformation(origin = {150, 10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Division massflowCalc annotation(Placement(visible = true, transformation(origin = {110, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product spezificHeat annotation(Placement(visible = true, transformation(origin = {140, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add bldgDeltaT(k2 = -1) annotation(Placement(visible = true, transformation(origin = {170, -42}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Fluid.Sensors.Temperature HWS_TE2 annotation(Placement(visible = true, transformation(origin = {210, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Fluid.Sensors.Temperature HWR_TE2 annotation(Placement(visible = true, transformation(origin = {210, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression bldgDesignCp(y = bldgMedium_cp) annotation(Placement(visible = true, transformation(origin = {250, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  initial equation
    bldgMediumState = BldgMedium.setState_pTX(bldgHWR.p, (globalDHconfig.bldgHWS_temp + globalDHconfig.bldgHWR_temp) / 2, bldgHWR.X);
  equation
  // ---------- Connectors ----------
    connect(HWR_TE2.T, bldgDeltaT.u2) annotation(Line(points = {{203, -50}, {191, -50}, {191, -48}, {182, -48}}, color = {0, 0, 127}));
    connect(HWR_TE2.port, secondaryHWR) annotation(Line(points = {{210, -60}, {210, -70}, {90, -70}}, color = {0, 127, 255}));
    connect(bldgHWR.ports[1], HWR_TE2.port) annotation(Line(points = {{240, -70}, {210, -70}, {210, -60}, {210, -60}}, color = {0, 127, 255}));
    connect(HWS_TE2.T, bldgDeltaT.u1) annotation(Line(points = {{203, 50}, {191, 50}, {191, -36}, {182, -36}}, color = {0, 0, 127}));
    connect(loadCalc.y, massflowCalc.u1) annotation(Line(points = {{138, 10}, {126, 10}, {126, -24}, {122, -24}}, color = {0, 0, 127}));
    if globalDHconfig.use_HWS_temp_SP then
      connect(globalDHconfig.heatLoads, loadCalc.u) annotation(Line(points = {{240.8, 89.2}, {179.8, 89.2}, {179.8, 10}, {160, 10}}, color = {0, 0, 127}));
    else
  	connect(globalDHconfig.constHeatLoads, loadCalc.u);
    end if;
    if globalDHconfig.use_HWS_temp_SP then
      connect(globalDHconfig.bldgHWS_temp_SP, HWS_temp_SP) annotation(Line(points = {{242, 98}, {100, 98}, {100, 70}, {90, 70}}, color = {0, 0, 127}));
    else
  	connect(globalDHconfig.constHWS_temp_SP, HWS_temp_SP);
    end if;
    connect(HWS_TE2.port, bldgHWS.ports[1]) annotation(Line(points = {{210, 40}, {210, 30}, {240, 30}}));
    connect(secondaryHWS, HWS_TE2.port) annotation(Line(points = {{90, 30}, {210, 30}, {210, 40}}));
    connect(bldgDesignCp.y, spezificHeat.u1) annotation(Line(points = {{239, -10}, {157.5, -10}, {157.5, -30}, {152, -30}}, color = {0, 0, 127}));
    connect(spezificHeat.y, massflowCalc.u2) annotation(Line(points = {{129, -36}, {122, -36}}, color = {0, 0, 127}));
    connect(bldgDeltaT.y, spezificHeat.u2) annotation(Line(points = {{159, -42}, {152, -42}}, color = {0, 0, 127}));
    connect(massflowCalc.y, pumpMassFlowIn) annotation(Line(points = {{99, -30}, {90, -30}}, color = {0, 0, 127}));
// ---------- Equations ----------
// ---------- Graphics & Layout ----------
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {260, 100}}, initialScale = 0.1), graphics = {Rectangle(origin = {121, 20}, fillColor = {255, 255, 127}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-41, 40}, {59, -60}}), Rectangle(origin = {150, -21}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-10, 19}, {10, -19}}), Line(origin = {130, 76}, points = {{0, 0}}), Polygon(origin = {130, 79.89}, fillColor = {163, 55, 0}, fillPattern = FillPattern.Solid, lineThickness = 1, points = {{-50, -20}, {0, 20}, {50, -20}, {-50, -20}})}), uses(Modelica(version = "3.2.1")), Diagram(coordinateSystem(extent = {{-100, -100}, {260, 100}})), version = "");
  end DH_ETS;

model GlobalDHconfig
  // ---------- Declarations ----------
  replaceable package DH_Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium 
  annotation(Dialog(tab = "General", group = "DH Distribution Medium"), choicesAllMatching=true);
  parameter Boolean use_HWS_temp_SP = false "Set true if set-point shall be variable" annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Boolean use_calculatedHeatLoads = false "Set true if heat loads shall be variable" annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Modelica.SIunits.Temperature constHWS_temp_SP(displayUnit="degC") "If use_HWS_temp_SP is false" annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Modelica.SIunits.Temperature actualHWR_temp(displayUnit="degC") "Actual building return temperature" annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Real spaceHeatGain = 1 annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Real dhwHeatGain = 1 annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Real processHeatGain = 1 annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Real constHeatLoads[3]=array(spaceHeatGain, dhwHeatGain, processHeatGain) "If use_calculatedHeatLoads is false" annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Real oatTable[:,2]={{0.0,-8.0},{86400.0,-8.0}} annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Real HWS_temp_SP_Table[:,2]={{-273.0,1.0},{-8.0,1.0},{15.0,1.0},{273.0,1.0}} annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Real spaceHeatTable[:,2]={{0.0,1.0},{86400.0,1.0}} annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Real dhwHeatTable[:,2]={{0.0,1.0},{86400.0,1.0}} annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Real processHeatTable[:,2]={{0.0,1.0},{86400.0,1.0}} annotation(Dialog(tab = "General", group = "Simulation"));
  parameter Modelica.SIunits.Height roughness = 0.00005 annotation(Dialog(tab = "Distribution Pipe", group = "Geometry"));
  parameter DE.PipeSpec.InsulClass insulClass  = DE.PipeSpec.InsulClass.Class_2 annotation(Dialog(tab = "Distribution Pipe", group = "Geometry"));
  parameter Modelica.SIunits.TemperatureDifference distrTempOffset = 5 annotation(Dialog(tab = "ETS", group = "Control Parameters"));
  parameter Modelica.SIunits.TemperatureDifference hexLMTD = 2 annotation(Dialog(tab = "ETS", group = "Design Conditions"));
  parameter Modelica.SIunits.Temperature bldgHWS_temp(displayUnit="degC") annotation(Dialog(tab = "ETS", group = "Design Conditions"));
  parameter Modelica.SIunits.Temperature bldgHWR_temp(displayUnit="degC") annotation(Dialog(tab = "ETS", group = "Design Conditions"));
  // ---------- Declarations ----------
  Modelica.Blocks.Sources.TimeTable dhwHeatCurve(table = dhwHeatTable)  annotation(Placement(visible = true, transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.TimeTable processHeatCurve(table = processHeatTable)  annotation(Placement(visible = true, transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Sources.TimeTable oatCurve(table = oatTable) annotation(Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain spaceHeatLoad(k = spaceHeatGain) annotation(Placement(visible = true, transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain dhwHeatLoad(k = dhwHeatGain) annotation(Placement(visible = true, transformation(origin = {50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain processHeatLoad(k = processHeatGain) annotation(Placement(visible = true, transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Tables.CombiTable1Ds HWS_temp_SP_curve(table = HWS_temp_SP_Table)  annotation(Placement(visible = true, transformation(origin = {-30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Tables.CombiTable1Ds spaceHeatLoadCurve(table = spaceHeatTable)  annotation(Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput bldgHWS_temp_SP if use_HWS_temp_SP annotation(Placement(visible = true, transformation(origin = {90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput heatLoads[3] if use_calculatedHeatLoads annotation(Placement(visible = true, transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {92, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  if use_calculatedHeatLoads then
    connect(processHeatLoad.y, heatLoads[3]) annotation(Line(points = {{62, -50}, {82, -50}, {82, -10}, {90, -10}}, color = {0, 0, 127}));
    connect(dhwHeatLoad.y, heatLoads[2]) annotation(Line(points = {{62, -10}, {84, -10}, {84, -10}, {90, -10}}, color = {0, 0, 127}));
    connect(spaceHeatLoad.y, heatLoads[1]) annotation(Line(points = {{62, 30}, {82, 30}, {82, -10}, {90, -10}}, color = {0, 0, 127}));
  end if;
  if use_HWS_temp_SP then
    connect(HWS_temp_SP_curve.y[1], bldgHWS_temp_SP) annotation(Line(points = {{-18, 70}, {84, 70}, {84, 70}, {90, 70}}, color = {0, 0, 127}));
  end if;
  connect(oatCurve.y, spaceHeatLoadCurve.u) annotation(Line(points = {{-59, 50}, {-50, 50}, {-50, 30}, {-42, 30}}, color = {0, 0, 127}));
  connect(oatCurve.y, HWS_temp_SP_curve.u) annotation(Line(points = {{-59, 50}, {-50, 50}, {-50, 70}, {-42, 70}}, color = {0, 0, 127}));
  connect(spaceHeatLoadCurve.y[1], spaceHeatLoad.u) annotation(Line(points = {{-18, 30}, {38, 30}, {38, 30}, {38, 30}}, color = {0, 0, 127}));
  connect(dhwHeatCurve.y, dhwHeatLoad.u) annotation(Line(points = {{-59, -10}, {37, -10}}, color = {0, 0, 127}));
  connect(processHeatCurve.y, processHeatLoad.u) annotation(Line(points = {{-59, -50}, {37, -50}}, color = {0, 0, 127}));
  annotation(Icon(coordinateSystem(grid = {2, 8}, initialScale = 0.1), graphics = {Ellipse(origin = {-10, 12}, fillColor = {255, 170, 127}, fillPattern = FillPattern.Sphere, extent = {{-70, 68}, {90, -92}}, endAngle = 360), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name", fontName = "DejaVu Sans Mono"), Text(origin = {1, 80}, extent = {{-80, 0}, {80, -50}}, textString = "Global", fontName = "DejaVu Sans Mono", textStyle = {TextStyle.Bold}), Text(origin = {5, -64}, lineThickness = 1, extent = {{-80, 30}, {80, -20}}, textString = "Design", fontName = "DejaVu Sans Mono", textStyle = {TextStyle.Bold}), Text(origin = {0, 8}, lineColor = {170, 0, 0}, lineThickness = 1, extent = {{-80, 24}, {80, -40}}, textString = "DH", fontName = "DejaVu Sans Mono", textStyle = {TextStyle.Bold})}), uses(Modelica(version = "3.2.1")), defaultComponentName = "globalDHconfig");
end GlobalDHconfig;
  annotation(Icon(coordinateSystem(grid = {2, 8}, initialScale = 0.1), graphics = {Bitmap(origin = {3, 24}, extent = {{-93, 24}, {89, -64}}, imageSource = "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")}), Diagram);
end DE;
