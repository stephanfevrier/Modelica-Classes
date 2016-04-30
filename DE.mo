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
    extends Modelica.Fluid.Pipes.DynamicPipe(redeclare replaceable package Medium = Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium, redeclare replaceable model HeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.IdealFlowHeatTransfer constrainedby Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.PartialFlowHeatTransfer, final heatTransfer.k = lambda_insul * 2 * Modelica.Constants.pi / Modelica.Math.log(outsideDiaInsul / 2 / (outsideDiaPipe / 2)), final diameter = insideDiaPipe, final use_HeatTransfer = true ", nNodes=integer(Modelica.SIunits.Conversions.to_unit1(length)/12)");
    // ---------- Declarations ----------
    parameter DE.PipeSpec.InsulClass insulClass annotation(Dialog(tab = "General", group = "EN253 Pipe Type"));
    parameter DE.PipeSpec.DNtype DN annotation(Dialog(tab = "General", group = "EN253 Pipe Type"));
    parameter Modelica.SIunits.ThermalConductivity lambda_insul = 0.03 annotation(Dialog(tab = "General", group = "EN253 Pipe Type"));
    constant Modelica.SIunits.Diameter insideDiaPipe = outsideDiaPipe - 2 * pipeThickness;
    constant Modelica.SIunits.Diameter outsideDiaPipe = DE.SpecTables.EN253Spec[insulClass, DN, 1];
    constant Modelica.SIunits.Radius pipeThickness = DE.SpecTables.EN253Spec[insulClass, DN, 2];
    constant Modelica.SIunits.Diameter outsideDiaInsul = DE.SpecTables.EN253Spec[insulClass, DN, 3];
    // ---------- Components ----------
  equation
// ---------- Connectors ----------
// ---------- Equations ----------
// ---------- Graphics & Layout ----------
    annotation(Icon(coordinateSystem(grid = {2, 8}, initialScale = 0.1), graphics = {Rectangle(origin = {-1, 60}, fillColor = {240, 240, 119}, fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-99, 4}, {101, -20}}), Rectangle(origin = {0, -60}, fillColor = {240, 240, 119}, fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-100, 20}, {100, -4}}), Text(origin = {0, 80}, extent = {{-64, 16}, {64, -16}}, textString = "EN253 Pipe", fontName = "DejaVu Sans Mono")}), Diagram);
  end EN253pipe;

  record PipeSpec
    type InsulClass = enumeration(Class_1, Class_2, Class_3);
    type DNtype = enumeration(DN20, DN25, DN32, DN40, DN50, DN65, DN80, DN100, DN125, DN150, DN200, DN250, DN300, DN350, DN400, DN450, DN500, DN600, DN700, DN800, DN900, DN1000);
    annotation(Icon(coordinateSystem(grid = {2, 8})));
  end PipeSpec;

  package SpecTables
    final constant Real[DE.PipeSpec.InsulClass, DE.PipeSpec.DNtype, 3] EN253Spec = {{{0.0269, 0.0026, 0.090}, {0.0337, 0.0026, 0.090}, {0.0424, 0.0026, 0.110}, {0.0483, 0.0026, 0.110}, {0.0603, 0.0029, 0.125}, {0.0761, 0.0029, 0.140}, {0.0889, 0.0032, 0.160}, {0.1143, 0.0036, 0.200}, {0.1397, 0.0036, 0.225}, {0.1683, 0.0040, 0.250}, {0.2191, 0.0045, 0.315}, {0.273, 0.0050, 0.400}, {0.3239, 0.0056, 0.450}, {0.3556, 0.0056, 0.500}, {0.4064, 0.0063, 0.560}, {0.4572, 0.0063, 0.630}, {0.508, 0.0063, 0.710}, {0.610, 0.0071, 0.800}, {0.711, 0.008, 0.900}, {0.813, 0.0088, 1.000}, {0.914, 0.01, 1.100}, {1.016, 0.011, 1.200}}, {{0.0269, 0.0026, 0.110}, {0.0337, 0.0026, 0.110}, {0.0424, 0.0026, 0.125}, {0.0483, 0.0026, 0.125}, {0.0603, 0.0029, 0.140}, {0.0761, 0.0029, 0.160}, {0.0889, 0.0032, 0.180}, {0.1143, 0.0036, 0.225}, {0.1397, 0.0036, 0.250}, {0.1683, 0.0040, 0.280}, {0.2191, 0.0045, 0.355}, {0.273, 0.0050, 0.450}, {0.3239, 0.0056, 0.500}, {0.3556, 0.0056, 0.560}, {0.4064, 0.0063, 0.560}, {0.4572, 0.0063, 0.630}, {0.508, 0.0063, 0.710}, {0.610, 0.0071, 0.800}, {0.711, 0.008, 0.900}, {0.813, 0.0088, 1.000}, {0.914, 0.01, 1.100}, {1.016, 0.011, 1.200}}, {{0.0269, 0.0026, 0.125}, {0.0337, 0.0026, 0.125}, {0.0424, 0.0026, 0.140}, {0.0483, 0.0026, 0.140}, {0.0603, 0.0029, 0.160}, {0.0761, 0.0029, 0.180}, {0.0889, 0.0032, 0.200}, {0.1143, 0.0036, 0.250}, {0.1397, 0.0036, 0.280}, {0.1683, 0.0040, 0.315}, {0.2191, 0.0045, 0.400}, {0.273, 0.0050, 0.500}, {0.3239, 0.0056, 0.560}, {0.3556, 0.0056, 0.560}, {0.4064, 0.0063, 0.630}, {0.4572, 0.0063, 0.710}, {0.508, 0.0063, 0.800}, {0.610, 0.0071, 0.800}, {0.711, 0.008, 0.900}, {0.813, 0.0088, 1.000}, {0.914, 0.01, 1.100}, {1.016, 0.011, 1.200}}};
    //Isulation Class
    //DN20 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN25 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN32 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN40 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN50 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN65 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN80 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN100 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN125 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN150 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN200 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN250 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN300 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN350 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN400 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN450 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN500 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN600 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN700 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN800 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN900 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    //DN1000 - outside diameter pipe, wall thickness pipe, outside diameter insulation
    annotation(Icon(coordinateSystem(grid = {2, 8})));
  end SpecTables;

  model ETS
    // ---------- Definitions ----------
    // ---------- Inheritances ----------
    // ---------- Declarations ----------
    parameter Real hexAlpha = 4000 annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Real hexWallConductivity annotation(Dialog(tab = "General", group = "Characteristics"));
    parameter Real hexWallThickness annotation(Dialog(tab = "General", group = "Geometry"));
    replaceable package primaryMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    replaceable package secondaryMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    // ---------- Components ----------
    Modelica.Fluid.Pipes.DynamicPipe primaryPipe(redeclare replaceable package Medium = primaryMedium, redeclare replaceable model FlowModel = Modelica.Fluid.Pipes.BaseClasses.FlowModels.DetailedPipeFlow, redeclare replaceable model HeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer(alpha0 = hexAlpha), use_HeatTransfer = true) annotation(Placement(visible = true, transformation(origin = {-30, -20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Fluid.Pipes.DynamicPipe secondaryPipe(redeclare replaceable package Medium = secondaryMedium, redeclare replaceable model FlowModel = Modelica.Fluid.Pipes.BaseClasses.FlowModels.DetailedPipeFlow, redeclare replaceable model HeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer(alpha0 = hexAlpha), use_HeatTransfer = true) annotation(Placement(visible = true, transformation(origin = {30, -20}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor HExMaterial(G = hexWallConductivity * hexWallThickness) annotation(Placement(visible = true, transformation(origin = {0, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    replaceable Modelica.Fluid.Valves.ValveIncompressible valve(redeclare replaceable package Medium = primaryMedium, dp(displayUnit = "Pa"), dp_nominal(displayUnit = "Pa"), CvData = Modelica.Fluid.Types.CvTypes.OpPoint, allowFlowReversal = true) annotation(Placement(visible = true, transformation(extent = {{-50, -80}, {-70, -60}}, rotation = 0)));
    replaceable Modelica.Fluid.Machines.ControlledPump pump(redeclare replaceable package Medium = secondaryMedium, checkValve = true, use_m_flow_set = true) annotation(Placement(visible = true, transformation(origin = {60, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Fluid.Interfaces.FluidPort_a primaryHWS(redeclare replaceable package Medium = primaryMedium) annotation(Placement(visible = true, transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Interfaces.FluidPort_b primaryHWR(redeclare replaceable package Medium = primaryMedium) annotation(Placement(visible = true, transformation(origin = {-90, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Interfaces.FluidPort_b secondaryHWS(redeclare replaceable package Medium = secondaryMedium) annotation(Placement(visible = true, transformation(origin = {90, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Fluid.Interfaces.FluidPort_a secondaryHWR(redeclare replaceable package Medium = secondaryMedium) annotation(Placement(visible = true, transformation(origin = {90, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {90, -14}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Fluid.Sensors.Temperature HWS_TT(redeclare replaceable package Medium = secondaryMedium) annotation(Placement(visible = true, transformation(origin = {60, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID TC_PID(Td = 0, Ti = 120, k = 0.5, yMax = 100, yMin = 0) annotation(Placement(visible = true, transformation(origin = {0, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput pumpMassflow_u annotation(Placement(visible = true, transformation(origin = {67, -91}, extent = {{9, -9}, {-9, 9}}, rotation = -90), iconTransformation(origin = {50, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealInput HWS_u_s annotation(Placement(visible = true, transformation(origin = {90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {90, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  equation
// ---------- Connectors ----------
    connect(HWS_TT.T, TC_PID.u_m) annotation(Line(points = {{54, 50}, {0, 50}, {0, 58}}, color = {0, 0, 127}));
    connect(HWS_u_s, TC_PID.u_s) annotation(Line(points = {{90, 70}, {12, 70}}, color = {0, 0, 127}));
    connect(TC_PID.y, valve.opening) annotation(Line(points = {{-11, 70}, {-11, 70.5}, {-60, 70.5}, {-60, -62}}, color = {0, 0, 127}));
    connect(secondaryHWR, pump.port_a) annotation(Line(points = {{90, -70}, {70, -70}}));
    connect(pump.port_b, secondaryPipe.port_a) annotation(Line(points = {{50, -70}, {30, -70}, {30, -30}}, color = {0, 127, 255}));
    connect(secondaryPipe.port_b, HWS_TT.port) annotation(Line(points = {{30, -10}, {30, 30}, {60, 30}, {60, 40}}, color = {0, 127, 255}));
    connect(HWS_TT.port, secondaryHWS) annotation(Line(points = {{60, 40}, {60, 30}, {90, 30}, {90, 30}}, color = {0, 127, 255}));
    connect(HExMaterial.port_b, secondaryPipe.heatPorts[1]) annotation(Line(points = {{10, -20}, {26, -20}}, color = {191, 0, 0}));
    connect(primaryPipe.heatPorts[1], HExMaterial.port_a) annotation(Line(points = {{-25.6, -20.1}, {-9.6, -20.1}}, color = {127, 0, 0}));
    connect(primaryHWS, primaryPipe.port_a) annotation(Line(points = {{-90, 30}, {-30, 30}, {-30, -10}}));
    connect(primaryPipe.port_b, valve.port_a) annotation(Line(points = {{-30, -30}, {-30, -70}, {-50, -70}}, color = {0, 127, 255}));
    connect(valve.port_b, primaryHWR) annotation(Line(points = {{-70, -70}, {-90, -70}}, color = {0, 127, 255}));
    connect(pumpMassflow_u, pump.m_flow_set) annotation(Line(points = {{67, -91}, {66, -91}, {66, -64}}, color = {0, 0, 127}));
// ---------- Equations ----------
// ---------- Graphics & Layout ----------
    annotation(Icon(coordinateSystem(grid = {2, 8}, initialScale = 0.1), graphics = {Rectangle(origin = {-19, 16}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Sphere, lineThickness = 2, extent = {{-40, 40}, {40, -40}}), Line(origin = {5.27, 16}, points = {{74.7293, 32}, {-55.2707, 32}, {6.72932, 0}, {-55.2707, -32}, {74.7293, -32}}, thickness = 2), Line(origin = {-24.27, -52}, points = {{-3.72703, 12}, {12.273, 12}, {-3.72703, -12}, {12.273, -12}, {-3.72703, 12}}, thickness = 2), Line(origin = {-20, -32}, points = {{0, 8}, {0, -8}}, thickness = 2), Line(origin = {-60, -72}, points = {{40, 8}, {40, -8}, {-20, -8}}, thickness = 2, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {-60, 68}, points = {{40, -12}, {40, 12}, {-20, 12}}, thickness = 2, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 10), Ellipse(origin = {60, 80}, lineThickness = 2, extent = {{-15, 15}, {15, -15}}, endAngle = 360), Line(origin = {60, 52}, points = {{0, 12}, {0, -4}}, thickness = 2), Text(origin = {60, 80}, lineThickness = 2, extent = {{-10, 10}, {10, -10}}, textString = "TC", fontName = "DejaVu Sans Mono"), Ellipse(origin = {50, -16}, lineThickness = 2, extent = {{-15, 15}, {15, -15}}, endAngle = 360), Line(origin = {45.33, -16}, points = {{4.67075, -15}, {-9.32925, 0}, {4.67075, 15}}, thickness = 2), Text(origin = {0, 125}, lineColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", fontName = "DejaVu Sans Mono"), Line(origin = {54.21, -55.85}, points = {{-4.20711, 24}, {-4.20711, -24}}, pattern = LinePattern.Dash, thickness = 2)}), uses(Modelica(version = "3.2.1")), Diagram(coordinateSystem(grid = {2, 8})), version = "");
  end ETS;

  model DE_ETS
    // ---------- Definitions and Inheritances ----------
    // ---------- Declarations ----------
    replaceable package DE_Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    replaceable package BldgMedium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    parameter Modelica.SIunits.Power spaceHeatCapacity, dhwHeatCapacity, processHeatCapacity annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Modelica.SIunits.Area hexArea annotation(Dialog(tab = "General", group = "Geometry"));
    parameter Modelica.SIunits.Temp_C hexLMTD annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Modelica.SIunits.Pressure hexDeltaP annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Modelica.SIunits.Pressure valveDeltaP annotation(Dialog(tab = "General", group = "Design Conditions"));
    outer parameter Modelica.SIunits.Temp_C globalBldgHWS_temp, globalBldgHWR_temp;
    outer Real heatLoads[3]; //{spaceHeatLoad, dhwHeatLoad, processHeatLoad};
    BldgMedium.ThermodynamicState pumpMediumState;
    Modelica.SIunits.SpecificHeatCapacity bldgMedium_cp=BldgMedium.specificHeatCapacityCp(pumpMediumState);
    // ---------- Components ----------
    DE.ETS deETS(
  	redeclare replaceable package primaryMedium=DE_Medium, 
  	redeclare replaceable package secondaryMedium=BldgMedium) 
  	annotation(Placement(visible = true, transformation(origin = {-40, -5.32907e-15}, extent = {{-60, -60}, {60, 60}}, rotation = 0)));
    
    Modelica.Fluid.Sources.FixedBoundary bldgHWS(
  	redeclare replaceable package Medium = BldgMedium, 
  	T = globalBldgHWS_temp, 
  	nPorts = 1)  
  	annotation(Placement(visible = true, transformation(origin = {90, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  
    Modelica.Fluid.Sources.FixedBoundary bldgHWR(
  	redeclare replaceable package Medium=BldgMedium, 
  	T=globalBldgHWR_temp, 
  	nPorts = 1) 
  	annotation(Placement(visible = true, transformation(origin = {90, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  	
    Modelica.Blocks.Sources.Constant bldgHWS_sp(
  	k=globalBldgHWS_temp)
  	annotation(Placement(visible = true, transformation(origin = {50, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  	
    Modelica.Blocks.Math.MultiSum loadCalc(
  	k = array(spaceHeatCapacity, dhwHeatCapacity, processHeatCapacity), 
  	nu = 3) 
  	annotation(Placement(visible = true, transformation(origin = {70, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  	
    Modelica.Blocks.Math.Division massflowCalc 
  	annotation(Placement(visible = true, transformation(origin = {10, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    
    Modelica.Blocks.Sources.Constant bldgDeltaT(
  	k=(globalBldgHWS_temp-globalBldgHWR_temp)) 
  	annotation(Placement(visible = true, transformation(origin = {70, -90}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  	
    Modelica.Blocks.Math.Product product1 
  	annotation(Placement(visible = true, transformation(origin = {40, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  	
  equation
  // ---------- Connectors ----------
    connect(bldgHWS_sp.y, deETS.HWS_u_s) annotation(Line(points = {{38, 50}, {16, 50}, {16, 50}, {14, 50}}, color = {0, 0, 127}));
    connect(bldgHWR.ports[1], deETS.secondaryHWR) annotation(Line(points = {{80, -10}, {16, -10}, {16, -8}, {14, -8}}, color = {0, 127, 255}));
    connect(deETS.secondaryHWS, bldgHWS.ports[1]) annotation(Line(points = {{14, 30}, {80, 30}}, color = {0, 127, 255}));
    connect(heatLoads, loadCalc.u);
    connect(loadCalc.y, massflowCalc.u1) annotation(Line(points = {{58, -50}, {40, -50}, {40, -64}, {22, -64}}, color = {0, 0, 127}));
    connect(product1.y, massflowCalc.u2) annotation(Line(points = {{30, -80}, {24, -80}, {24, -76}, {22, -76}}, color = {0, 0, 127}));
    connect(bldgDeltaT.y, product1.u2) annotation(Line(points = {{60, -90}, {52, -90}, {52, -86}, {52, -86}}, color = {0, 0, 127}));
    connect(bldgMedium_cp, product1.u1);
    connect(massflowCalc.y, deETS.pumpMassflow_u) annotation(Line(points = {{0, -70}, {-10, -70}, {-10, -52}, {-10, -52}}, color = {0, 0, 127}));
  // ---------- Equations ----------
pumpMediumState = BldgMedium.setState_phX(deETS.secondaryHWR.p, deETS.secondaryHWR.h_outflow, deETS.secondaryHWR.Xi_outflow);
  // ---------- Graphics & Layout ----------
    annotation(Icon, uses(Modelica(version = "3.2.1")), Diagram, version = "");
  end DE_ETS;

  model DE_ETS_2
    // ---------- Definitions ----------
    // ---------- Inheritances ----------
    extends DE.ETS(redeclare replaceable Modelica.Fluid.Sources.FixedBoundary secondaryHWS(redeclare replaceable package Medium = secondaryMedium) constrainedby Modelica.Fluid.Interfaces.FluidPort);
    // ---------- Declarations ----------
    parameter Real hexArea annotation(Dialog(tab = "General", group = "Geometry"));
    parameter Real hexLMTD annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Real hexDeltaP annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Real hexPower annotation(Dialog(tab = "General", group = "Design Conditions"));
    parameter Real valveDeltaP annotation(Dialog(tab = "General", group = "Design Conditions"));
    outer Real loadFactor;
    // ---------- Components ----------
    //outer Modelica.Fluid.Interfaces.FluidPort_b globalBldgHWS(redeclare replaceable package Medium = secondaryMedium);
    //outer Modelica.Fluid.Interfaces.FluidPort_a globalBldgHWR(redeclare replaceable package Medium = secondaryMedium);
    //outer Modelica.Blocks.Interfaces.RealInput globalBldgHWS_u_s;
  equation
// ---------- Connectors ----------
//connect(secondaryHWS, globalBldgHWS);
//connect(secondaryHWR, globalBldgHWR);
//connect(HWS_u_s, globalBldgHWS_u_s);
// ---------- Equations ----------
// ---------- Graphics & Layout ----------
    annotation(Icon, uses(Modelica(version = "3.2.1")), Diagram, version = "");
  end DE_ETS_2;
  annotation(Icon(coordinateSystem(grid = {2, 8}, initialScale = 0.1), graphics = {Bitmap(origin = {3, 24}, extent = {{-93, 24}, {89, -64}}, imageSource = "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")}), Diagram);
end DE;
