package MyGaggia
  extends Modelica.Icons.Package;

  model HW_AcDcCircuit
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {-60, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch powerSwitch annotation(
      Placement(visible = true, transformation(origin = {-60, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage1(V = 230, freqHz = 50) annotation(
      Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.PowerConverters.ACDC.DiodeBridge2Pulse acdcConvertor annotation(
      Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.HeatingResistor protectionThermistor(R_ref = 22, T_ref = 298.15) annotation(
      Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.VariableResistor protectionVaristor annotation(
      Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(
      Placement(visible = true, transformation(origin = {100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {170, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
      Placement(visible = true, transformation(origin = {100, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {170, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 0.000001) annotation(
      Placement(visible = true, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = 0.0001) annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Transformer transformer1 annotation(
      Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_powerSwitch annotation(
      Placement(visible = true, transformation(origin = {-60, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-58, 104}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor vPeaksSensor annotation(
      Placement(visible = true, transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  equation
    connect(vPeaksSensor.n, protectionVaristor.n) annotation(
      Line(points = {{-50, -10}, {-50, -10}, {-50, -20}, {-20, -20}, {-20, -10}, {-20, -10}}, color = {0, 0, 255}));
    connect(protectionVaristor.p, vPeaksSensor.p) annotation(
      Line(points = {{-20, 10}, {-20, 10}, {-20, 20}, {-50, 20}, {-50, 10}, {-50, 10}}, color = {0, 0, 255}));
    connect(vPeaksSensor.v, protectionVaristor.R) annotation(
      Line(points = {{-40, 0}, {-32, 0}, {-32, 0}, {-30, 0}}, color = {0, 0, 127}));
    connect(protectionVaristor.n, ground1.p) annotation(
      Line(points = {{-20, -10}, {-20, -80}, {-60, -80}}, color = {0, 0, 255}));
    connect(powerSwitch.n, protectionVaristor.p) annotation(
      Line(points = {{-50, 82}, {-20, 82}, {-20, 10}}, color = {0, 0, 255}));
    connect(transformer1.p2, acdcConvertor.ac_p) annotation(
      Line(points = {{20, 5}, {21, 5}, {21, 5}, {22, 5}, {22, 7}, {24, 7}, {24, 5}, {30, 5}, {30, 5}, {30, 5}, {30, 5}, {30, 5}}, color = {0, 0, 255}));
    connect(acdcConvertor.ac_n, transformer1.n2) annotation(
      Line(points = {{30, -6}, {20, -6}, {20, -4}, {20, -4}, {20, -6}, {20, -6}, {20, -6}, {20, -6}}, color = {0, 0, 255}));
    connect(transformer1.n1, ground1.p) annotation(
      Line(points = {{0, -5}, {0, -80}, {-60, -80}}, color = {0, 0, 255}));
    connect(protectionThermistor.n, transformer1.p1) annotation(
      Line(points = {{0, 40}, {0, 6}}, color = {0, 0, 255}));
    connect(acdcConvertor.dc_n, capacitor2.n) annotation(
      Line(points = {{50, -6}, {60, -6}, {60, -18}, {100, -18}, {100, -15}, {100, -15}, {100, -10}}, color = {0, 0, 255}));
    connect(acdcConvertor.dc_p, capacitor2.p) annotation(
      Line(points = {{50, 6}, {55, 6}, {55, 8}, {60, 8}, {60, 20}, {100, 20}, {100, 15}, {100, 15}, {100, 12.5}, {100, 12.5}, {100, 10}}, color = {0, 0, 255}));
    connect(acdcConvertor.dc_p, capacitor1.p) annotation(
      Line(points = {{50, 6}, {55, 6}, {55, 8}, {60, 8}, {60, 20}, {80, 20}, {80, 15}, {80, 15}, {80, 10}}, color = {0, 0, 255}));
    connect(capacitor1.n, acdcConvertor.dc_n) annotation(
      Line(points = {{80, -10}, {80, -11.25}, {80, -11.25}, {80, -12.5}, {80, -12.5}, {80, -13}, {80, -13}, {80, -18}, {60, -18}, {60, -4}, {53, -4}, {53, -4}, {48.5, -4}, {48.5, -6}, {49.25, -6}, {49.25, -6}, {50, -6}}, color = {0, 0, 255}));
    connect(powerSwitch.n, protectionThermistor.p) annotation(
      Line(points = {{-50, 82}, {0, 82}, {0, 60}}, color = {0, 0, 255}));
    connect(acdcConvertor.dc_p, pin_p) annotation(
      Line(points = {{50, 6}, {52.5, 6}, {52.5, 6}, {55, 6}, {55, 8}, {60, 8}, {60, 20}, {100, 20}, {100, 90}}, color = {0, 0, 255}));
    connect(acdcConvertor.dc_n, pin_n) annotation(
      Line(points = {{50, -6}, {55, -6}, {55, -4}, {60, -4}, {60, -20}, {100, -20}, {100, -55}, {100, -55}, {100, -72.5}, {100, -72.5}, {100, -90}}, color = {0, 0, 255}));
    connect(u_powerSwitch, powerSwitch.control) annotation(
      Line(points = {{-60, 120}, {-60, 120}, {-60, 90}, {-60, 90}}, color = {255, 0, 255}));
    connect(sineVoltage1.n, ground1.p) annotation(
      Line(points = {{-80, -10}, {-80, -80}, {-60, -80}}, color = {0, 0, 255}));
    connect(sineVoltage1.p, powerSwitch.p) annotation(
      Line(points = {{-80, 10}, {-80, 82}, {-70, 82}}, color = {0, 0, 255}));
    annotation(
      Icon(coordinateSystem(grid = {2, 8})),
      uses(Modelica(version = "3.2.2")),
      Diagram);
  end HW_AcDcCircuit;

  model HW_PowerCircuit
    outer parameter Modelica.Fluid.System system;
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.HeatingResistor heatingResistor1(R_ref = 50, T = system.T_ambient, T_ref = 298.15) annotation(
      Placement(visible = true, transformation(origin = {-40, -60}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
    Modelica.Electrical.Analog.Semiconductors.Diode pumpDiode(T = system.T_ambient) annotation(
      Placement(visible = true, transformation(origin = {80, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch powerSwitch annotation(
      Placement(visible = true, transformation(origin = {-70, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Ideal.ControlledIdealClosingSwitch heaterSSR annotation(
      Placement(visible = true, transformation(origin = {-40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage1(V = 230, freqHz = 50) annotation(
      Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch steamThermostat annotation(
      Placement(visible = true, transformation(origin = {-40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch steamSwitch annotation(
      Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch pumpSwitch annotation(
      Placement(visible = true, transformation(origin = {40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Ideal.ControlledIdealClosingSwitch pumpSSR annotation(
      Placement(visible = true, transformation(origin = {80, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p_heaterSSR annotation(
      Placement(visible = true, transformation(origin = {-20, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_powerSwitch annotation(
      Placement(visible = true, transformation(origin = {-70, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-60, 80}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Interfaces.BooleanInput u_pumpSwitch annotation(
      Placement(visible = true, transformation(origin = {60, 120}, extent = {{20, -20}, {-20, 20}}, rotation = 90), iconTransformation(origin = {60, 80}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Interfaces.BooleanInput u_steamSwitch annotation(
      Placement(visible = true, transformation(origin = {20, 120}, extent = {{20, -20}, {-20, 20}}, rotation = 90), iconTransformation(origin = {0, 80}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p_pumpSSR annotation(
      Placement(visible = true, transformation(origin = {100, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatingElement annotation(
      Placement(visible = true, transformation(origin = {20, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MyLib.Thermal.HeatTransfer.Sensors.Thermostat steamThermosensor(tHigh = 413.15, tLow = 403.15) annotation(
      Placement(visible = true, transformation(origin = {0, -20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.PositivePin pump_p annotation(
      Placement(visible = true, transformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.NegativePin pump_n annotation(
      Placement(visible = true, transformation(origin = {100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(pump_n, pumpDiode.p) annotation(
      Line(points = {{102, -20}, {80, -20}, {80, -40}, {80, -40}}, color = {0, 0, 255}));
    connect(pumpSSR.n, pump_p) annotation(
      Line(points = {{80, 40}, {80, 20}, {100, 20}}, color = {0, 0, 255}));
    connect(pumpSwitch.n, pump_p) annotation(
      Line(points = {{40, 40}, {40, 20}, {100, 20}}, color = {0, 0, 255}));
    connect(pumpDiode.n, ground1.p) annotation(
      Line(points = {{80, -60}, {80, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(heatingElement, steamThermosensor.port) annotation(
      Line(points = {{20, -60}, {20, -20}, {10, -20}}, color = {191, 0, 0}));
    connect(steamThermosensor.y, steamThermostat.control) annotation(
      Line(points = {{-11, -20}, {-33, -20}}, color = {255, 0, 255}));
    connect(steamSwitch.n, steamThermostat.p) annotation(
      Line(points = {{0, 40}, {0, 20}, {-40, 20}, {-40, -10}}, color = {0, 0, 255}));
    connect(heaterSSR.n, steamThermostat.p) annotation(
      Line(points = {{-40, 40}, {-40, -10}}, color = {0, 0, 255}));
    connect(steamThermostat.n, heatingResistor1.p) annotation(
      Line(points = {{-40, -30}, {-40, -50}}, color = {0, 0, 255}));
    connect(sineVoltage1.n, ground1.p) annotation(
      Line(points = {{-90, -10}, {-90, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(sineVoltage1.p, powerSwitch.p) annotation(
      Line(points = {{-90, 10}, {-90, 80}, {-80, 80}}, color = {0, 0, 255}));
    connect(heatingElement, heatingResistor1.heatPort) annotation(
      Line(points = {{20, -60}, {-30, -60}}, color = {191, 0, 0}));
    connect(heatingResistor1.n, ground1.p) annotation(
      Line(points = {{-40, -70}, {-40, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(powerSwitch.n, steamSwitch.p) annotation(
      Line(points = {{-60, 80}, {0, 80}, {0, 60}}, color = {0, 0, 255}));
    connect(u_powerSwitch, powerSwitch.control) annotation(
      Line(points = {{-70, 120}, {-70, 87}}, color = {255, 0, 255}));
    connect(powerSwitch.n, pumpSSR.p) annotation(
      Line(points = {{-60, 80}, {80, 80}, {80, 60}}, color = {0, 0, 255}));
    connect(powerSwitch.n, pumpSwitch.p) annotation(
      Line(points = {{-60, 80}, {40, 80}, {40, 60}}, color = {0, 0, 255}));
    connect(powerSwitch.n, heaterSSR.p) annotation(
      Line(points = {{-60, 80}, {-40, 80}, {-40, 60}}, color = {0, 0, 255}));
    connect(pin_p_pumpSSR, pumpSSR.control) annotation(
      Line(points = {{100, 50}, {90, 50}}, color = {0, 0, 255}));
    connect(u_pumpSwitch, pumpSwitch.control) annotation(
      Line(points = {{60, 120}, {60, 50}, {47, 50}}, color = {255, 0, 255}));
    connect(u_steamSwitch, steamSwitch.control) annotation(
      Line(points = {{20, 120}, {20, 50}, {7, 50}}, color = {255, 0, 255}));
    connect(pin_p_heaterSSR, heaterSSR.control) annotation(
      Line(points = {{-20, 50}, {-30, 50}}, color = {0, 0, 255}));
    annotation(
      Icon(coordinateSystem(grid = {2, 8})),
      uses(Modelica(version = "3.2.2")),
      Diagram);
  end HW_PowerCircuit;

  model HW_HeaterPowerCircuit
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {-80, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.HeatingResistor heatingResistor1(R_ref = 50, T_ref = 298.15, useHeatPort = true) annotation(
      Placement(visible = true, transformation(origin = {-40, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch powerSwitch annotation(
      Placement(visible = true, transformation(origin = {-80, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Ideal.ControlledIdealClosingSwitch heaterSSR(level = 3) annotation(
      Placement(visible = true, transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage1(V = 230, freqHz = 50) annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p_heaterSSR annotation(
      Placement(visible = true, transformation(origin = {-20, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_powerSwitch annotation(
      Placement(visible = true, transformation(origin = {-80, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 80}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatingElement annotation(
      Placement(visible = true, transformation(origin = {-80, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MyLib.Thermal.HeatTransfer.Sensors.Thermostat steamThermosensor(tHigh = 413.15, tLow = 403.15) annotation(
      Placement(visible = true, transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch steamThermostat annotation(
      Placement(visible = true, transformation(origin = {-40, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  equation
    connect(steamThermosensor.y, steamThermostat.control) annotation(
      Line(points = {{-59, -10}, {-47, -10}}, color = {255, 0, 255}));
    connect(steamThermostat.n, heatingResistor1.p) annotation(
      Line(points = {{-40, -20}, {-40, -40}}, color = {0, 0, 255}));
    connect(heaterSSR.n, steamThermostat.p) annotation(
      Line(points = {{-40, 20}, {-40, 0}}, color = {0, 0, 255}));
    connect(steamThermosensor.port, heatingElement) annotation(
      Line(points = {{-80, -10}, {-80, -50}}, color = {191, 0, 0}));
    connect(heatingElement, heatingResistor1.heatPort) annotation(
      Line(points = {{-80, -50}, {-50, -50}}, color = {191, 0, 0}));
    connect(heatingResistor1.n, ground1.p) annotation(
      Line(points = {{-40, -60}, {-40, -90}, {-80, -90}}, color = {0, 0, 255}));
    connect(powerSwitch.n, heaterSSR.p) annotation(
      Line(points = {{-70, 90}, {-40, 90}, {-40, 40}}, color = {0, 0, 255}));
    connect(pin_p_heaterSSR, heaterSSR.control) annotation(
      Line(points = {{-20, 30}, {-30, 30}}, color = {0, 0, 255}));
    connect(u_powerSwitch, powerSwitch.control) annotation(
      Line(points = {{-80, 120}, {-80, 98}}, color = {255, 0, 255}));
    connect(sineVoltage1.p, powerSwitch.p) annotation(
      Line(points = {{-100, 10}, {-100, 30}, {-100, 30}, {-100, 50}, {-100, 50}, {-100, 90}, {-96, 90}, {-96, 90}, {-94, 90}, {-94, 90}, {-90, 90}}, color = {0, 0, 255}));
    connect(sineVoltage1.n, ground1.p) annotation(
      Line(points = {{-100, -10}, {-100, -50}, {-100, -50}, {-100, -90}, {-90, -90}, {-90, -90}, {-80, -90}}, color = {0, 0, 255}));
    annotation(
      Icon(coordinateSystem(grid = {2, 8})),
      uses(Modelica(version = "3.2.2")),
      Diagram);
  end HW_HeaterPowerCircuit;

  model HW_TempCtrlCircuit
    outer parameter Modelica.Fluid.System system;
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage1(V = 5) annotation(
      Placement(visible = true, transformation(origin = {-90, -2}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    MyLib.Electrical.Analog.Basic.SteinhartResistor tempSensor(Beta = 4200, R_ref = 100000, T = system.T_ambient, T_ref = 298.15, use_BetaOnly = true) annotation(
      Placement(visible = true, transformation(origin = {-40, 40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Resistor tempResistor(R = 499, T = system.T_ambient, T_ref = 298.15) annotation(
      Placement(visible = true, transformation(origin = {-40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(visible = true, transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput y_tempSignal annotation(
      Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatToTempSensor annotation(
      Placement(visible = true, transformation(origin = {-70, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
      Placement(visible = true, transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_heaterSSR annotation(
      Placement(visible = true, transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-80, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    MyLib.Electrical.Analog.Ideal.BooleanToDO bToDO_heaterSSR(RsignalToGround = 499) annotation(
      Placement(visible = true, transformation(origin = {60, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MyLib.Electrical.Analog.Ideal.AIToInteger aiToI_tempSignal(enableNoise = true, noiseAmplitude = 0.005, referenceSignal = 1.1, signalResolution = 1023) annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(aiToI_tempSignal.y_signalOut, y_tempSignal) annotation(
      Line(points = {{11, 0}, {40, 0}}, color = {255, 127, 0}));
    connect(bToDO_heaterSSR.n_signalOut, pin_n) annotation(
      Line(points = {{71, 50}, {90, 50}}, color = {0, 0, 255}));
    connect(aiToI_tempSignal.n_ground, ground.p) annotation(
      Line(points = {{0, -11}, {0, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(tempSensor.n, aiToI_tempSignal.p_signalIn) annotation(
      Line(points = {{-40, 30}, {-40, 0}, {-11, 0}}, color = {0, 0, 255}));
    connect(u_heaterSSR, bToDO_heaterSSR.u_signalIn) annotation(
      Line(points = {{30, 50}, {52, 50}}, color = {255, 0, 255}));
    connect(bToDO_heaterSSR.n_ground, ground.p) annotation(
      Line(points = {{60, 39}, {60, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(constantVoltage1.p, bToDO_heaterSSR.p_Vsupply) annotation(
      Line(points = {{-90, 8}, {-90, 80}, {60, 80}, {60, 61}}, color = {0, 0, 255}));
    connect(heatToTempSensor, tempSensor.heatPort) annotation(
      Line(points = {{-70, 40}, {-50, 40}}, color = {191, 0, 0}));
    connect(tempSensor.n, tempResistor.p) annotation(
      Line(points = {{-40, 30}, {-40, -30}}, color = {0, 0, 255}));
    connect(constantVoltage1.p, tempSensor.p) annotation(
      Line(points = {{-90, 8}, {-90, 80}, {-40, 80}, {-40, 50}}, color = {0, 0, 255}));
    connect(tempResistor.n, ground.p) annotation(
      Line(points = {{-40, -50}, {-40, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(ground.p, constantVoltage1.n) annotation(
      Line(points = {{-70, -80}, {-90, -80}, {-90, -12}}, color = {0, 0, 255}));
    annotation(
      Icon(coordinateSystem(grid = {2, 8})));
  end HW_TempCtrlCircuit;

  model HW_VolumeCtrlCircuit
    outer parameter Modelica.Fluid.System system;
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage1(V = 5) annotation(
      Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch doubleShotSwitch annotation(
      Placement(visible = true, transformation(origin = {20, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch singleShotSwitch annotation(
      Placement(visible = true, transformation(origin = {40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Semiconductors.NPN flowMeter(Bf = 8, Phic = 0.6, Phie = 0.6, T = system.T_ambient) annotation(
      Placement(visible = true, transformation(origin = {-40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    // Parameters for transistor, meterResistor, signalResistor to be verified. OpenCollector output signal not yet satisfying.
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch flowMeterContact annotation(
      Placement(visible = true, transformation(origin = {-60, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Resistor flowSignalResistor(R = 4700, T = system.T_ambient, T_ref = 298.15) annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(visible = true, transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Capacitor flowCapacitor(C = 0.0000001) annotation(
      Placement(visible = true, transformation(origin = {-10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Interfaces.BooleanOutput y_singleShot annotation(
      Placement(visible = true, transformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput y_doubleShot annotation(
      Placement(visible = true, transformation(origin = {109, -1}, extent = {{-9, -9}, {9, 9}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput y_flowSignal annotation(
      Placement(visible = true, transformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_flowSignal annotation(
      Placement(visible = true, transformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, -64}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_doubleShotSignal annotation(
      Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_singleShotSignal annotation(
      Placement(visible = true, transformation(origin = {-120, 90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 64}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    MyLib.Electrical.Analog.Ideal.DIToBoolean diToB_singleShot(enableNoise = true, switchingThreshold = 3) annotation(
      Placement(visible = true, transformation(origin = {70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MyLib.Electrical.Analog.Ideal.DIToBoolean diToB_doubleShot(enableNoise = true, switchingThreshold = 3) annotation(
      Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MyLib.Electrical.Analog.Ideal.DIToBoolean diToB_FluidSignal(enableNoise = true, switchingThreshold = 3) annotation(
      Placement(visible = true, transformation(origin = {70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor singleShotResistor(R = 4700, T = system.T_ambient, T_ref = 298.15) annotation(
      Placement(visible = true, transformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Resistor doubleShotResistor(R = 4700, T = system.T_ambient, T_ref = 298.15) annotation(
      Placement(visible = true, transformation(origin = {20, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Resistor flowMeterResistor(R = 47000, T = system.T_ambient, T_ref = 298.15) annotation(
      Placement(visible = true, transformation(origin = {-60, -20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    MyLib.Electrical.Analog.Ideal.BooleanToDO bToDO_pumpSSR annotation(
      Placement(visible = true, transformation(origin = {90, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_pumpSSR annotation(
      Placement(visible = true, transformation(origin = {60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 80}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
      Placement(visible = true, transformation(origin = {120, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(diToB_doubleShot.y_signalOut, y_doubleShot) annotation(
      Line(points = {{81, 0}, {95.5, 0}, {95.5, -1}, {109, -1}}, color = {255, 0, 255}));
    connect(bToDO_pumpSSR.n_signalOut, pin_n) annotation(
      Line(points = {{102, 60}, {118, 60}, {118, 60}, {120, 60}}, color = {0, 0, 255}));
    connect(u_pumpSSR, bToDO_pumpSSR.u_signalIn) annotation(
      Line(points = {{60, 60}, {80, 60}, {80, 60}, {82, 60}}, color = {255, 0, 255}));
    connect(bToDO_pumpSSR.n_ground, ground.p) annotation(
      Line(points = {{90, 49}, {90, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(constantVoltage1.p, bToDO_pumpSSR.p_Vsupply) annotation(
      Line(points = {{-90, 10}, {-90, 80}, {90, 80}, {90, 71}}, color = {0, 0, 255}));
    connect(flowMeterResistor.n, flowMeter.B) annotation(
      Line(points = {{-60, -30}, {-60, -30}, {-60, -40}, {-50, -40}, {-50, -40}}, color = {0, 0, 255}));
    connect(flowSignalResistor.n, flowMeter.C) annotation(
      Line(points = {{-10, 0}, {-10, -35}, {-30, -35}}, color = {0, 0, 255}));
    connect(flowMeter.E, ground.p) annotation(
      Line(points = {{-30, -45}, {-30, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(flowMeterContact.n, flowMeterResistor.p) annotation(
      Line(points = {{-60, 0}, {-60, -10}}, color = {0, 0, 255}));
    connect(u_flowSignal, flowMeterContact.control) annotation(
      Line(points = {{-120, 30}, {-40, 30}, {-40, 10}, {-53, 10}}, color = {255, 0, 255}));
    connect(constantVoltage1.p, flowMeterContact.p) annotation(
      Line(points = {{-90, 10}, {-90, 80}, {-60, 80}, {-60, 20}}, color = {0, 0, 255}));
    connect(ground.p, constantVoltage1.n) annotation(
      Line(points = {{-70, -80}, {-90, -80}, {-90, -10}}, color = {0, 0, 255}));
    connect(constantVoltage1.p, singleShotSwitch.p) annotation(
      Line(points = {{-90, 10}, {-90, 80}, {40, 80}, {40, 60}}, color = {0, 0, 255}));
    connect(constantVoltage1.p, flowSignalResistor.p) annotation(
      Line(points = {{-90, 10}, {-90, 80}, {-10, 80}, {-10, 20}}, color = {0, 0, 255}));
    connect(constantVoltage1.p, doubleShotSwitch.p) annotation(
      Line(points = {{-90, 10}, {-90, 80}, {20, 80}, {20, 40}}, color = {0, 0, 255}));
    connect(singleShotSwitch.n, diToB_singleShot.p_signalIn) annotation(
      Line(points = {{40, 40}, {40, 30}, {59, 30}}, color = {0, 0, 255}));
    connect(diToB_singleShot.n_ground, ground.p) annotation(
      Line(points = {{70, 19}, {70, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(diToB_singleShot.y_signalOut, y_singleShot) annotation(
      Line(points = {{81, 30}, {101, 30}, {101, 30}, {109, 30}}, color = {255, 0, 255}));
    connect(diToB_doubleShot.n_ground, ground.p) annotation(
      Line(points = {{70, -11}, {70, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(doubleShotSwitch.n, diToB_doubleShot.p_signalIn) annotation(
      Line(points = {{20, 20}, {20, 0}, {59, 0}}, color = {0, 0, 255}));
    connect(diToB_FluidSignal.y_signalOut, y_flowSignal) annotation(
      Line(points = {{81, -30}, {110, -30}}, color = {255, 0, 255}));
    connect(diToB_FluidSignal.n_ground, ground.p) annotation(
      Line(points = {{70, -41}, {70, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(flowCapacitor.p, diToB_FluidSignal.p_signalIn) annotation(
      Line(points = {{-10, -50}, {-10, -30}, {59, -30}}, color = {0, 0, 255}));
    connect(doubleShotResistor.n, ground.p) annotation(
      Line(points = {{20, -70}, {20, -70}, {20, -80}, {-70, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(doubleShotSwitch.n, doubleShotResistor.p) annotation(
      Line(points = {{20, 20}, {20, 20}, {20, -50}, {20, -50}}, color = {0, 0, 255}));
    connect(singleShotResistor.n, ground.p) annotation(
      Line(points = {{40, -70}, {40, -70}, {40, -80}, {-70, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(singleShotSwitch.n, singleShotResistor.p) annotation(
      Line(points = {{40, 40}, {40, 40}, {40, -50}, {40, -50}}, color = {0, 0, 255}));
    connect(u_doubleShotSignal, doubleShotSwitch.control) annotation(
      Line(points = {{-120, 60}, {30, 60}, {30, 30}, {28, 30}, {28, 30}}, color = {255, 0, 255}));
    connect(flowSignalResistor.n, flowCapacitor.p) annotation(
      Line(points = {{-10, 0}, {-10, -50}}, color = {0, 0, 255}));
    connect(flowCapacitor.n, ground.p) annotation(
      Line(points = {{-10, -70}, {-10, -80}, {-70, -80}}, color = {0, 0, 255}));
    connect(u_singleShotSignal, singleShotSwitch.control) annotation(
      Line(points = {{-120, 90}, {50, 90}, {50, 50}, {47, 50}}, color = {255, 0, 255}));
    annotation(
      Icon(coordinateSystem(grid = {2, 8})),
      experiment(StartTime = 0, StopTime = 60, Tolerance = 0.0001, Interval = 0.1),
      __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "optimization", lv = "LOG_STATS"));
  end HW_VolumeCtrlCircuit;

  model HW_Hydraulics
    outer Modelica.Fluid.System system;
    replaceable package Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
    parameter Modelica.SIunits.Temperature boiler_T_start = system.T_ambient;
    Modelica.Fluid.Vessels.OpenTank waterTank(redeclare replaceable package Medium = Medium, crossArea = 0.2 * 0.1, height = 0.15, level_start = 0.12, nPorts = 1, portsData = {Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter = 0.005)}) annotation(
      Placement(visible = true, transformation(origin = {-40, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Fluid.Pipes.DynamicPipe pipeTankToPump(redeclare replaceable package Medium = Medium, diameter = 0.005, length = 0.3, modelStructure = Modelica.Fluid.Types.ModelStructure.a_vb, nNodes = 1) annotation(
      Placement(visible = true, transformation(origin = {-80, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    MyLib.Fluid.Assemblies.SolenoidPump solenoidPump(redeclare replaceable package Medium = Medium, R_Resistor = 165, c_spring = 6000, k_EMF = 250, pistonCrossArea = 0.006 ^ 2 * Modelica.Constants.pi, pistonLength = 0.02, pistonMass = 0.02, portDia = 0.005, springLength = 0.02) annotation(
      Placement(visible = true, transformation(origin = {-50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Pipes.DynamicPipe pipePumpToBoiler(redeclare replaceable package Medium = Medium, diameter = 0.005, length = 0.2, modelStructure = Modelica.Fluid.Types.ModelStructure.av_b, nNodes = 1) annotation(
      Placement(visible = true, transformation(origin = {10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Vessels.ClosedVolume boiler(redeclare replaceable package Medium = Medium, T_start = boiler_T_start, V = 250e-06, nPorts = 2, use_HeatTransfer = true, portsData = {Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter = 0.005), Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter = 0.005)}) annotation(
      Placement(visible = true, transformation(origin = {60, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a a_boiler annotation(
      Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MyLib.Fluid.Valves.CheckValve filterBasket(redeclare replaceable package Medium = Medium, diameter = 0.005, dp_open = 500000, zeta = 2.5) annotation(
      Placement(visible = true, transformation(origin = {60, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Fluid.Vessels.OpenTank coffeeCup(redeclare replaceable package Medium = Medium, crossArea = 0.02 ^ 2 * Modelica.Constants.pi, height = 0.08, level_start = 1e-06, nPorts = 1, portsData = {Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter = 0.005, height = coffeeCup.height)}) annotation(
      Placement(visible = true, transformation(extent = {{40, -80}, {80, -40}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.PositivePin pump_p annotation(
      Placement(visible = true, transformation(origin = {-80, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.NegativePin pump_n annotation(
      Placement(visible = true, transformation(origin = {-20, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature ambient(T = system.T_ambient) annotation(
      Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Components.Convection boilerSurface annotation(
      Placement(visible = true, transformation(origin = {60, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant heatConductance(k = 0.02) annotation(
      Placement(visible = true, transformation(origin = {90, 90}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(heatConductance.y, boilerSurface.Gc) annotation(
      Line(points = {{78, 90}, {60, 90}, {60, 80}, {60, 80}}, color = {0, 0, 127}));
    connect(boilerSurface.solid, boiler.heatPort) annotation(
      Line(points = {{70, 70}, {80, 70}, {80, 28}, {75, 28}, {75, 30}, {70, 30}}, color = {191, 0, 0}));
    connect(ambient.port, boilerSurface.fluid) annotation(
      Line(points = {{40, 70}, {50, 70}}, color = {191, 0, 0}));
    connect(pipePumpToBoiler.port_b, boiler.ports[1]) annotation(
      Line(points = {{20, 20}, {60, 20}}, color = {0, 127, 255}));
    connect(boiler.ports[2], filterBasket.port_a) annotation(
      Line(points = {{60, 20}, {60, 0}}, color = {0, 127, 255}, thickness = 0.5));
    connect(a_boiler, boiler.heatPort) annotation(
      Line(points = {{90, 30}, {70, 30}}, color = {191, 0, 0}));
    connect(solenoidPump.port_b, pipePumpToBoiler.port_a) annotation(
      Line(points = {{-40, 20}, {0, 20}}, color = {0, 127, 255}));
    connect(pump_p, solenoidPump.p) annotation(
      Line(points = {{-80, 50}, {-80, 38}, {-80, 38}, {-80, 24}, {-70, 24}, {-70, 26}, {-60, 26}}, color = {0, 0, 255}));
    connect(solenoidPump.n, pump_n) annotation(
      Line(points = {{-40, 26}, {-20, 26}, {-20, 50}}, color = {0, 0, 255}));
    connect(pipeTankToPump.port_b, solenoidPump.port_a) annotation(
      Line(points = {{-80, 0}, {-80, 20}, {-60, 20}}, color = {0, 127, 255}));
    connect(filterBasket.port_b, coffeeCup.ports[1]) annotation(
      Line(points = {{60, -20}, {60, -80}}, color = {0, 127, 255}));
    connect(waterTank.ports[1], pipeTankToPump.port_a) annotation(
      Line(points = {{-40, -80}, {-80, -80}, {-80, -20}}, color = {0, 127, 255}, thickness = 0.5));
    annotation(
      Icon(coordinateSystem(grid = {2, 8})),
      experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-05, Interval = 0.01),
      __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS"));
  end HW_Hydraulics;



  model SW_TempCtrl
    parameter Modelica.SIunits.Temperature temp_SP;
    parameter Real PID_PropGain, PID_IntTime;
    Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM1(f = 5, useConstantDutyCycle = false) annotation(
      Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MyGaggia.Utilities.NTC_DigitalToTemp nTC_DigitalToTemp1(Beta = 4200, R_pullup = 499, R_ref = 100000, scaleMaxValue = 1.1, signalResolution = 1023, use_BetaOnly = true, v_supply = 5) annotation(
      Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID PID_tempCtrl(Td = 0, Ti = PID_IntTime, initType = Modelica.Blocks.Types.InitPID.InitialOutput, k = PID_PropGain, yMax = 1, yMin = 0, y_start = 1) annotation(
      Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerInput u_tempSignal annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput y_heaterSSR annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.RealExpression tempSetPoint(y = temp_SP) annotation(
      Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(tempSetPoint.y, PID_tempCtrl.u_s) annotation(
      Line(points = {{-18, 0}, {-2, 0}, {-2, 0}, {-2, 0}}, color = {0, 0, 127}));
    connect(PID_tempCtrl.y, signalPWM1.dutyCycle) annotation(
      Line(points = {{21, 0}, {38, 0}}, color = {0, 0, 127}));
    connect(nTC_DigitalToTemp1.y, PID_tempCtrl.u_m) annotation(
      Line(points = {{-59, 0}, {-48.5, 0}, {-48.5, -40}, {8.75, -40}, {8.75, -12}, {10, -12}}, color = {0, 0, 127}));
    connect(signalPWM1.fire, y_heaterSSR) annotation(
      Line(points = {{44, 11}, {44, 40}, {80, 40}, {80, 0}, {110, 0}}, color = {255, 0, 255}));
    connect(u_tempSignal, nTC_DigitalToTemp1.u) annotation(
      Line(points = {{-120, 0}, {-82, 0}}, color = {255, 127, 0}));
    annotation(
      Icon(coordinateSystem(grid = {2, 8})));
  end SW_TempCtrl;

  model SW_VolumeCtrl
    parameter Real singleShotVolume, doubleShotVolume;
    parameter Real volumeFlowPerPulse;
    parameter Integer pumpPowerStage;
    Modelica.Blocks.Interfaces.BooleanInput u_Pulse annotation(
      Placement(visible = true, transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, -64}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput y_SSR annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_SingleShot annotation(
      Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 64}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput u_DoubleShot annotation(
      Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.MathInteger.MultiSwitch multiSwitch1(expr = {sShotPulseCount, dShotPulseCount}, nu = 2, use_pre_as_default = true) annotation(
      Placement(visible = true, transformation(origin = {-40.0778, 40}, extent = {{-5.18231, -10}, {15.5469, 10}}, rotation = 0)));
    MyGaggia.Utilities.DispenseShot dispenseShot annotation(
      Placement(visible = true, transformation(origin = {40, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Logical.FallingEdge feSingleShot annotation(
      Placement(visible = true, transformation(origin = {-80, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Logical.FallingEdge feDoubleShot annotation(
      Placement(visible = true, transformation(origin = {-80, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Logical.Or orSingleDouble annotation(
      Placement(visible = true, transformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM powerModulationPWM(f = 5, useConstantDutyCycle = false) annotation(
      Placement(visible = true, transformation(origin = {60, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.IntegerExpression pumpStage(y = pumpPowerStage) annotation(
      Placement(visible = true, transformation(origin = {-80, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain dutyCycleCalc(k = 0.1) annotation(
      Placement(visible = true, transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.MathBoolean.And dutyCycleOverlay(nu = 2) annotation(
      Placement(visible = true, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.IntegerToReal iToR_dutyCycle annotation(
      Placement(visible = true, transformation(origin = {-40, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    Integer sShotPulseCount = integer(singleShotVolume / volumeFlowPerPulse);
    Integer dShotPulseCount = integer(doubleShotVolume / volumeFlowPerPulse);
  equation
    connect(iToR_dutyCycle.y, dutyCycleCalc.u) annotation(
      Line(points = {{-29, -70}, {-10, -70}}, color = {0, 0, 127}));
    connect(dutyCycleCalc.y, powerModulationPWM.dutyCycle) annotation(
      Line(points = {{13, -70}, {48, -70}}, color = {0, 0, 127}));
    connect(pumpStage.y, iToR_dutyCycle.u) annotation(
      Line(points = {{-69, -70}, {-50, -70}}, color = {255, 127, 0}));
    connect(powerModulationPWM.fire, dutyCycleOverlay.u[2]) annotation(
      Line(points = {{52, -59}, {52, -4}, {70, -4}, {70, 0}}, color = {255, 0, 255}));
    connect(dutyCycleOverlay.y, y_SSR) annotation(
      Line(points = {{92, 0}, {102, 0}, {102, 0}, {110, 0}}, color = {255, 0, 255}));
    connect(dispenseShot.y, dutyCycleOverlay.u[1]) annotation(
      Line(points = {{52, 40}, {60, 40}, {60, 4}, {70, 4}, {70, 0}}, color = {255, 0, 255}));
    connect(feDoubleShot.y, orSingleDouble.u2) annotation(
      Line(points = {{-69, 20}, {-20, 20}, {-20, 62}, {-12, 62}}, color = {255, 0, 255}));
    connect(feSingleShot.y, orSingleDouble.u1) annotation(
      Line(points = {{-68, 80}, {-20, 80}, {-20, 70}, {-12, 70}}, color = {255, 0, 255}));
    connect(orSingleDouble.y, dispenseShot.u_ShotRequest) annotation(
      Line(points = {{11, 70}, {20, 70}, {20, 48}, {31, 48}}, color = {255, 0, 255}));
    connect(u_Pulse, dispenseShot.u_FlowMeterInput) annotation(
      Line(points = {{-120, -20}, {20, -20}, {20, 32.5}, {32, 32.5}, {32, 33}}, color = {255, 0, 255}));
    connect(multiSwitch1.y, dispenseShot.u_TargetVolume) annotation(
      Line(points = {{-29, 40}, {30, 40}}, color = {255, 127, 0}));
    connect(feDoubleShot.y, multiSwitch1.u[2]) annotation(
      Line(points = {{-69, 20}, {-60, 20}, {-60, 40}, {-52, 40}}, color = {255, 0, 255}));
    connect(feSingleShot.y, multiSwitch1.u[1]) annotation(
      Line(points = {{-68, 80}, {-60, 80}, {-60, 40}, {-52, 40}}, color = {255, 0, 255}));
    connect(u_DoubleShot, feDoubleShot.u) annotation(
      Line(points = {{-120, 20}, {-94, 20}, {-94, 20}, {-92, 20}}, color = {255, 0, 255}));
    connect(u_SingleShot, feSingleShot.u) annotation(
      Line(points = {{-120, 80}, {-92, 80}, {-92, 80}, {-92, 80}}, color = {255, 0, 255}));
    annotation(
      Icon(coordinateSystem(grid = {2, 8})));
  end SW_VolumeCtrl;

  package Utilities
    model DispenseShot
      Modelica.Blocks.Interfaces.BooleanInput u_ShotRequest annotation(
        Placement(visible = true, transformation(origin = {-120, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 72}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.IntegerInput u_TargetVolume annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanInput u_FlowMeterInput annotation(
        Placement(visible = true, transformation(origin = {-120, -70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, -72}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanOutput y annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.Utilities.PulseCounter shotCounter annotation(
        Placement(visible = true, transformation(origin = {50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop1 annotation(
        Placement(visible = true, transformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.And and1 annotation(
        Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.And and11 annotation(
        Placement(visible = true, transformation(origin = {-70, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Pre pre1 annotation(
        Placement(visible = true, transformation(origin = {10, 90}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Or or1 annotation(
        Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.FallingEdge fallingEdge1 annotation(
        Placement(visible = true, transformation(origin = {-70, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Pre pre11 annotation(
        Placement(visible = true, transformation(origin = {50, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    equation
      connect(shotCounter.y, y) annotation(
        Line(points = {{61, -10}, {79.5, -10}, {79.5, 0}, {110, 0}}, color = {255, 0, 255}));
      connect(shotCounter.y, pre11.u) annotation(
        Line(points = {{61, -10}, {79.5, -10}, {79.5, -50}, {62, -50}}, color = {255, 0, 255}));
      connect(pre11.y, fallingEdge1.u) annotation(
        Line(points = {{39, -50}, {-90, -50}, {-90, -20}, {-82, -20}}, color = {255, 0, 255}));
      connect(u_FlowMeterInput, shotCounter.u_Pulse) annotation(
        Line(points = {{-120, -70}, {30, -70}, {30, -16}, {42, -16}}, color = {255, 0, 255}));
      connect(u_TargetVolume, shotCounter.u_CounterPeriod) annotation(
        Line(points = {{-120, 0}, {29, 0}, {29, -4}, {42, -4}}, color = {255, 127, 0}));
      connect(rSFlipFlop1.Q, shotCounter.u_Enable) annotation(
        Line(points = {{21, 46}, {50, 46}, {50, -2}}, color = {255, 0, 255}));
      connect(fallingEdge1.y, or1.u2) annotation(
        Line(points = {{-59, -20}, {-50, -20}, {-50, 22}, {-42, 22}}, color = {255, 0, 255}));
      connect(u_ShotRequest, and1.u1) annotation(
        Line(points = {{-120, 70}, {-90, 70}, {-90, 50}, {-82, 50}, {-82, 50}}, color = {255, 0, 255}));
      connect(rSFlipFlop1.QI, and1.u2) annotation(
        Line(points = {{21, 34}, {29.5, 34}, {29.5, 10}, {-90, 10}, {-90, 42}, {-82, 42}}, color = {255, 0, 255}));
      connect(rSFlipFlop1.Q, pre1.u) annotation(
        Line(points = {{21, 46}, {50, 46}, {50, 90}, {22, 90}}, color = {255, 0, 255}));
      connect(and1.y, rSFlipFlop1.S) annotation(
        Line(points = {{-59, 50}, {-10, 50}, {-10, 46}, {-2, 46}}, color = {255, 0, 255}));
      connect(or1.y, rSFlipFlop1.R) annotation(
        Line(points = {{-19, 30}, {-10.5, 30}, {-10.5, 34}, {-2, 34}}, color = {255, 0, 255}));
      connect(pre1.y, and11.u1) annotation(
        Line(points = {{-1, 90}, {-90, 90}, {-90, 78}, {-82, 78}}, color = {255, 0, 255}));
      connect(and11.y, or1.u1) annotation(
        Line(points = {{-59, 78}, {-50, 78}, {-50, 30}, {-42, 30}}, color = {255, 0, 255}));
      connect(u_ShotRequest, and11.u2) annotation(
        Line(points = {{-120, 70}, {-82, 70}}, color = {255, 0, 255}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end DispenseShot;

    block PulseCounter
      extends Modelica.Blocks.Icons.Block;
      Modelica.Blocks.Interfaces.IntegerInput u_CounterPeriod annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 64}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanInput u_Pulse annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, -64}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanOutput y annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanInput u_Enable annotation(
        Placement(visible = true, transformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 80}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    protected
      Integer i(min = 0, start = 0, fixed = true);
    algorithm
      when not u_Enable then
        i := 0;
        y := false;
      end when;
      when u_Pulse then
        if i < u_CounterPeriod and u_Enable then
          y := true;
          i := pre(i) + 1;
        else
          y := false;
        end if;
      end when;
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end PulseCounter;

    block NTC_DigitalToTemp
      extends Modelica.Blocks.Interfaces.SO;
      parameter Modelica.SIunits.Voltage scaleMaxValue "Reference voltage of measurement input signal (upper limit)";
      parameter Real signalResolution "Digital resolution of input signal";
      parameter Modelica.SIunits.Voltage v_supply "Voltage of circuit power supply";
      parameter Modelica.SIunits.Resistance R_pullup "Resistance of measurement pullup resistor";
      parameter Modelica.SIunits.Resistance R_ref "Resistance of Steinhart Resistor at T_ref";
      parameter Modelica.SIunits.Temperature T_ref = 298.15 "Reference temperature of Steinhart Resistor";
      parameter Boolean use_BetaOnly = false "= true to use Beta coeff. and a simplified Steinhart equation";
      parameter Real A = 1 / R_ref, B = 0, C = 0, D = 0 if not use_BetaOnly;
      parameter Real Beta = -1e12 if use_BetaOnly;
      Modelica.Blocks.Interfaces.IntegerInput u annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.SIunits.Resistance R;
    protected
      parameter Real k = scaleMaxValue / signalResolution;
    equation
      u * k * (R + R_pullup) = v_supply * R_pullup;
      if not use_BetaOnly then
        1 / y = A + B * log(R / R_ref) + C * log(R / R_ref) ^ 2 + D * log(R / R_ref) ^ 3;
      elseif use_BetaOnly then
        1 / y = 1 / T_ref + 1 / Beta * log(R / R_ref);
      end if;
    end NTC_DigitalToTemp;
    annotation(
      Icon(coordinateSystem(grid = {2, 8})));
  end Utilities;

  package Testblocks
    model Test_SW_TempCtrl
      //  extends MyGaggia.SW_TempCtrl(PID_tempCtrl.initType = Modelica.Blocks.Types.InitPID.InitialOutput, PID_tempCtrl.controllerType = Modelica.Blocks.Types.SimpleController.PI);
      Modelica.Blocks.Sources.IntegerStep integerStep1(height = 600, offset = 50, startTime = 1) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interaction.Show.BooleanValue booleanValue1 annotation(
        Placement(visible = true, transformation(origin = {88, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.SW_TempCtrl sW_TempCtrl1(PID_IntTime = 5, PID_PropGain = 0.01, temp_SP = 368.15) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Noise.UniformNoise uniformNoise1(enableNoise = true, samplePeriod = 0.5, useAutomaticLocalSeed = true, useGlobalSeed = true, y_max = 2, y_min = -2) annotation(
        Placement(visible = true, transformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
        Placement(visible = true, transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.MathInteger.Sum sum1(nu = 2) annotation(
        Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner Modelica.Blocks.Noise.GlobalSeed globalSeed(enableNoise = true, useAutomaticSeed = true) annotation(
        Placement(visible = true, transformation(origin = {-88, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(sum1.y, sW_TempCtrl1.u_tempSignal) annotation(
        Line(points = {{-18, 0}, {-8, 0}, {-8, 0}, {-8, 0}}, color = {255, 127, 0}));
      connect(integerStep1.y, sum1.u[1]) annotation(
        Line(points = {{-58, 0}, {-42, 0}, {-42, 0}, {-40, 0}}, color = {255, 127, 0}));
      connect(realToInteger1.y, sum1.u[2]) annotation(
        Line(points = {{-58, -50}, {-50, -50}, {-50, 0}, {-40, 0}, {-40, 0}}, color = {255, 127, 0}));
      connect(uniformNoise1.y, realToInteger1.u) annotation(
        Line(points = {{-98, -50}, {-82, -50}, {-82, -50}, {-82, -50}}, color = {0, 0, 127}));
      connect(sW_TempCtrl1.y_heaterSSR, booleanValue1.activePort) annotation(
        Line(points = {{11, 0}, {76, 0}}, color = {255, 0, 255}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})),
        experiment(StartTime = 0, StopTime = 300, Tolerance = 0.0001, Interval = 0.5),
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS"));
    end Test_SW_TempCtrl;

    model Test_NTC
      MyGaggia.Utilities.NTC_DigitalToTemp nTC_DigitalToTemp1(Beta = 4200, R_pullup = 499, R_ref = 100000, scaleMaxValue = 1.1, signalResolution = 1023, use_BetaOnly = true, v_supply = 5) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.IntegerStep integerStep1(height = 600, offset = 50, startTime = 1) annotation(
        Placement(visible = true, transformation(origin = {-52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interaction.Show.RealValue realValue1 annotation(
        Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(nTC_DigitalToTemp1.y, realValue1.numberPort) annotation(
        Line(points = {{12, 0}, {38, 0}, {38, 0}, {38, 0}}, color = {0, 0, 127}));
      connect(integerStep1.y, nTC_DigitalToTemp1.u) annotation(
        Line(points = {{-40, 0}, {-12, 0}, {-12, 0}, {-12, 0}}, color = {255, 127, 0}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})),
        experiment(StartTime = 0, StopTime = 10, Tolerance = 0.0001, Interval = 0.1),
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS"));
    end Test_NTC;

    model Test_DispenseShot
      Modelica.Blocks.Sources.BooleanPulse booleanPulse2(period = 20, startTime = 11, width = 5) annotation(
        Placement(visible = true, transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interaction.Show.BooleanValue booleanValue1 annotation(
        Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.IntegerConstant integerConstant1(k = 10) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse1(period = 1, startTime = 14) annotation(
        Placement(visible = true, transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.Utilities.DispenseShot dispenseShot1 annotation(
        Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.FallingEdge fallingEdge1 annotation(
        Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(booleanPulse1.y, dispenseShot1.u_FlowMeterInput) annotation(
        Line(points = {{-58, -30}, {0, -30}, {0, -7}, {22, -7}}, color = {255, 0, 255}));
      connect(fallingEdge1.y, dispenseShot1.u_ShotRequest) annotation(
        Line(points = {{-18, 30}, {0, 30}, {0, 7}, {22, 7}}, color = {255, 0, 255}));
      connect(integerConstant1.y, dispenseShot1.u_TargetVolume) annotation(
        Line(points = {{-58, 0}, {22, 0}}, color = {255, 127, 0}));
      connect(dispenseShot1.y, booleanValue1.activePort) annotation(
        Line(points = {{41, 0}, {78, 0}}, color = {255, 0, 255}));
      connect(booleanPulse2.y, fallingEdge1.u) annotation(
        Line(points = {{-58, 30}, {-44, 30}, {-44, 30}, {-42, 30}}, color = {255, 0, 255}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})));
    end Test_DispenseShot;

    model Test_SW_VolumeCtrl
      SW_VolumeCtrl sW_VolumeCtrl1(pumpPowerStage = 8) annotation(
        Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanConstant booleanConstant1(k = false) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse1(period = 30, startTime = 4, width = 5) annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse2(period = 1, startTime = 8) annotation(
        Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interaction.Show.BooleanValue booleanValue1 annotation(
        Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(booleanConstant1.y, sW_VolumeCtrl1.u_DoubleShot) annotation(
        Line(points = {{-58, 0}, {2, 0}, {2, 0}, {2, 0}}, color = {255, 0, 255}));
      connect(booleanPulse1.y, sW_VolumeCtrl1.u_SingleShot) annotation(
        Line(points = {{-58, 70}, {-20, 70}, {-20, 6}, {2, 6}, {2, 6}}, color = {255, 0, 255}));
      connect(booleanPulse2.y, sW_VolumeCtrl1.u_Pulse) annotation(
        Line(points = {{-58, -70}, {-20, -70}, {-20, -6}, {2, -6}, {2, -6}}, color = {255, 0, 255}));
      connect(sW_VolumeCtrl1.y_SSR, booleanValue1.activePort) annotation(
        Line(points = {{22, 0}, {78, 0}, {78, 0}, {78, 0}}, color = {255, 0, 255}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})),
        experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-06, Interval = 0.12),
        __OpenModelica_simulationFlags(jacobian = "", s = "dassl", lv = "LOG_STATS"));
    end Test_SW_VolumeCtrl;

    model Test_HeaterPowerCircuit
      MyGaggia.HW_TempCtrlCircuit hW_TempCtrlCircuit1 annotation(
        Placement(visible = true, transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.SW_TempCtrl sW_TempCtrl1(PID_IntTime = 5, PID_PropGain = 0.2, temp_SP = 368.15) annotation(
        Placement(visible = true, transformation(origin = {10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.HW_HeaterPowerCircuit hW_PowerCircuit1 annotation(
        Placement(visible = true, transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanExpression powerSwitchState(y = true) annotation(
        Placement(visible = true, transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor1(C = 1050) annotation(
        Placement(visible = true, transformation(origin = {90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow fixedHeatFlow1(Q_flow = -50) annotation(
        Placement(visible = true, transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    equation
      connect(fixedHeatFlow1.port, heatCapacitor1.port) annotation(
        Line(points = {{70, 40}, {70, 40}, {70, 10}, {80, 10}, {80, 10}}, color = {191, 0, 0}));
      connect(sW_TempCtrl1.y_heaterSSR, hW_TempCtrlCircuit1.u_heaterSSR) annotation(
        Line(points = {{21, -50}, {40, -50}, {40, -20}, {-79, -20}, {-79, -42}, {-59, -42}}, color = {255, 0, 255}));
      connect(hW_TempCtrlCircuit1.pin_n, hW_PowerCircuit1.pin_p_heaterSSR) annotation(
        Line(points = {{-39, -42}, {-20, -42}, {-20, 10}, {21, 10}}, color = {0, 0, 255}));
      connect(hW_PowerCircuit1.heatingElement, heatCapacitor1.port) annotation(
        Line(points = {{39, 10}, {80, 10}}, color = {191, 0, 0}));
      connect(hW_PowerCircuit1.heatingElement, hW_TempCtrlCircuit1.heatToTempSensor) annotation(
        Line(points = {{40, 10}, {60, 10}, {60, -80}, {-80, -80}, {-80, -50}, {-58, -50}, {-58, -50}}, color = {191, 0, 0}));
      connect(hW_TempCtrlCircuit1.y_tempSignal, sW_TempCtrl1.u_tempSignal) annotation(
        Line(points = {{-39, -50}, {2, -50}}, color = {255, 127, 0}));
      connect(powerSwitchState.y, hW_PowerCircuit1.u_powerSwitch) annotation(
        Line(points = {{21, 50}, {30, 50}, {30, 18}}, color = {255, 0, 255}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})),
        experiment(StartTime = 0, StopTime = 400, Tolerance = 0.0001, Interval = 0.333333),
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS"));
    end Test_HeaterPowerCircuit;

    model Test_Hydraulics
      inner Modelica.Fluid.System system(energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, m_flow_nominal = 0.004, m_flow_small = 1e-4, p_ambient(displayUnit = "bar") = 100000, use_eps_Re = true) annotation(
        Placement(visible = true, transformation(extent = {{80, 80}, {100, 100}}, rotation = 0)));
      MyGaggia.HW_Hydraulics hW_Hydraulics1 annotation(
        Placement(visible = true, transformation(origin = {10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Sources.SineVoltage powerSource(V = 230, freqHz = 50) annotation(
        Placement(visible = true, transformation(origin = {10, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(visible = true, transformation(origin = {40, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Semiconductors.Diode idealDiode(T = system.T_ambient) annotation(
        Placement(visible = true, transformation(origin = {-20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow heaterFlow(Q_flow = 1000, T_ref = 393.15) annotation(
        Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(heaterFlow.port, hW_Hydraulics1.a_boiler) annotation(
        Line(points = {{-40, 10}, {0, 10}}, color = {191, 0, 0}));
      connect(ground1.p, powerSource.n) annotation(
        Line(points = {{40, 0}, {40, 0}, {40, 60}, {20, 60}, {20, 60}}, color = {0, 0, 255}));
      connect(hW_Hydraulics1.pump_n, ground1.p) annotation(
        Line(points = {{22, 16}, {40, 16}, {40, 0}, {40, 0}}, color = {0, 0, 255}));
      connect(idealDiode.n, hW_Hydraulics1.pump_p) annotation(
        Line(points = {{-20, 30}, {-20, 30}, {-20, 16}, {0, 16}, {0, 16}}, color = {0, 0, 255}));
      connect(powerSource.p, idealDiode.p) annotation(
        Line(points = {{0, 60}, {-20, 60}, {-20, 50}, {-20, 50}}, color = {0, 0, 255}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})),
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-05, Interval = 0.01),
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS"));
    end Test_Hydraulics;

    model Test_VolumeCtrlCircuit
      MyGaggia.HW_VolumeCtrlCircuit hW_VolumeCtrlCircuit annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse1(period = 4, startTime = 12) annotation(
        Placement(visible = true, transformation(origin = {-72, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanConstant booleanConstant(k = false) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse(period = 30, startTime = 5, width = 10) annotation(
        Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.SW_VolumeCtrl sW_VolumeCtrl(pumpPowerStage = 10) annotation(
        Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(sW_VolumeCtrl.y_SSR, hW_VolumeCtrlCircuit.u_pumpSSR) annotation(
        Line(points = {{52, 0}, {80, 0}, {80, 20}, {0, 20}, {0, 8}, {0, 8}}, color = {255, 0, 255}));
      connect(hW_VolumeCtrlCircuit.y_flowSignal, sW_VolumeCtrl.u_Pulse) annotation(
        Line(points = {{12, -6}, {30, -6}, {30, -6}, {32, -6}}, color = {255, 0, 255}));
      connect(hW_VolumeCtrlCircuit.y_doubleShot, sW_VolumeCtrl.u_DoubleShot) annotation(
        Line(points = {{12, 0}, {32, 0}, {32, 0}, {32, 0}}, color = {255, 0, 255}));
      connect(hW_VolumeCtrlCircuit.y_singleShot, sW_VolumeCtrl.u_SingleShot) annotation(
        Line(points = {{12, 6}, {30, 6}, {30, 6}, {32, 6}}, color = {255, 0, 255}));
      connect(booleanPulse.y, hW_VolumeCtrlCircuit.u_singleShotSignal) annotation(
        Line(points = {{-58, 50}, {-40, 50}, {-40, 6}, {-8, 6}, {-8, 6}}, color = {255, 0, 255}));
      connect(booleanConstant.y, hW_VolumeCtrlCircuit.u_doubleShotSignal) annotation(
        Line(points = {{-58, 0}, {-10, 0}, {-10, 0}, {-8, 0}}, color = {255, 0, 255}));
      connect(booleanPulse1.y, hW_VolumeCtrlCircuit.u_flowSignal) annotation(
        Line(points = {{-60, -54}, {-40, -54}, {-40, -6}, {-8, -6}, {-8, -6}}, color = {255, 0, 255}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})),
        experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-06, Interval = 0.12),
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS", clock = "RT"));
    end Test_VolumeCtrlCircuit;

    model Test_PowerCircuit
      inner Modelica.Fluid.System system(energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, m_flow_nominal = 0.002, m_flow_small = 1e-04, p_ambient(displayUnit = "bar") = 100000, use_eps_Re = true) annotation(
        Placement(visible = true, transformation(extent = {{80, 80}, {100, 100}}, rotation = 0)));
      inner Modelica.Blocks.Noise.GlobalSeed globalSeed(enableNoise = true, useAutomaticSeed = false) annotation(
        Placement(visible = true, transformation(origin = {70, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.HW_TempCtrlCircuit hW_TempCtrlCircuit1 annotation(
        Placement(visible = true, transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.SW_TempCtrl sW_TempCtrl1(PID_IntTime = 5, PID_PropGain = 0.2, temp_SP = 366.15) annotation(
        Placement(visible = true, transformation(origin = {10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.HW_PowerCircuit hW_PowerCircuit1 annotation(
        Placement(visible = true, transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanExpression powerSwitchState(y = true) annotation(
        Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanExpression steamSwitchState annotation(
        Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanExpression pumpSwitchState annotation(
        Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.HW_VolumeCtrlCircuit hW_VolumeCtrlCircuit annotation(
        Placement(visible = true, transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.SW_VolumeCtrl sW_VolumeCtrl(doubleShotVolume = 60, pumpPowerStage = 10, singleShotVolume = 30, volumeFlowPerPulse = 3) annotation(
        Placement(visible = true, transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse1(period = 2, startTime = 3) annotation(
        Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanConstant booleanConstant(k = false) annotation(
        Placement(visible = true, transformation(origin = {-110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse(period = 30, startTime = 2, width = 1) annotation(
        Placement(visible = true, transformation(origin = {-110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyGaggia.HW_Hydraulics hW_Hydraulics(boiler_T_start = system.T_ambient + 70) annotation(
        Placement(visible = true, transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(hW_PowerCircuit1.pump_p, hW_Hydraulics.pump_p) annotation(
        Line(points = {{40, 16}, {60, 16}, {60, -4}, {80, -4}, {80, -4}}, color = {0, 0, 255}));
      connect(hW_Hydraulics.a_boiler, hW_TempCtrlCircuit1.heatToTempSensor) annotation(
        Line(points = {{80, -10}, {-90, -10}, {-90, -50}, {-58, -50}, {-58, -50}}, color = {191, 0, 0}));
      connect(hW_PowerCircuit1.heatingElement, hW_Hydraulics.a_boiler) annotation(
        Line(points = {{30, 2}, {30, 2}, {30, -10}, {80, -10}, {80, -10}}, color = {191, 0, 0}));
      connect(hW_PowerCircuit1.pump_n, hW_Hydraulics.pump_n) annotation(
        Line(points = {{40, 4}, {120, 4}, {120, -4}, {102, -4}, {102, -4}}, color = {0, 0, 255}));
      connect(hW_TempCtrlCircuit1.pin_n, hW_PowerCircuit1.pin_p_heaterSSR) annotation(
        Line(points = {{-38, -42}, {-10, -42}, {-10, 18}, {21, 18}}, color = {0, 0, 255}));
      connect(pumpSwitchState.y, hW_PowerCircuit1.u_pumpSwitch) annotation(
        Line(points = {{11, 80}, {36, 80}, {36, 20}}, color = {255, 0, 255}));
      connect(steamSwitchState.y, hW_PowerCircuit1.u_steamSwitch) annotation(
        Line(points = {{11, 60}, {30, 60}, {30, 20}}, color = {255, 0, 255}));
      connect(powerSwitchState.y, hW_PowerCircuit1.u_powerSwitch) annotation(
        Line(points = {{11, 40}, {24, 40}, {24, 20}}, color = {255, 0, 255}));
      connect(hW_VolumeCtrlCircuit.pin_n, hW_PowerCircuit1.pin_p_pumpSSR) annotation(
        Line(points = {{-70, 19}, {-70, 7}, {21, 7}}, color = {0, 0, 255}));
      connect(booleanPulse.y, hW_VolumeCtrlCircuit.u_singleShotSignal) annotation(
        Line(points = {{-99, 60}, {-94, 60}, {-94, 60}, {-89, 60}, {-89, 36}, {-79, 36}, {-79, 36}}, color = {255, 0, 255}));
      connect(booleanConstant.y, hW_VolumeCtrlCircuit.u_doubleShotSignal) annotation(
        Line(points = {{-99, 30}, {-81, 30}, {-81, 30}, {-79, 30}}, color = {255, 0, 255}));
      connect(booleanPulse1.y, hW_VolumeCtrlCircuit.u_flowSignal) annotation(
        Line(points = {{-99, 0}, {-89, 0}, {-89, 24}, {-79, 24}, {-79, 24}}, color = {255, 0, 255}));
      connect(hW_VolumeCtrlCircuit.y_singleShot, sW_VolumeCtrl.u_SingleShot) annotation(
        Line(points = {{-59, 36.4}, {-54, 36.4}, {-54, 36.4}, {-49, 36.4}, {-49, 36.4}, {-49, 36.4}, {-49, 36.4}, {-49, 36.4}}, color = {255, 0, 255}));
      connect(hW_VolumeCtrlCircuit.y_doubleShot, sW_VolumeCtrl.u_DoubleShot) annotation(
        Line(points = {{-59, 30}, {-49, 30}, {-49, 30}, {-49, 30}}, color = {255, 0, 255}));
      connect(hW_VolumeCtrlCircuit.y_flowSignal, sW_VolumeCtrl.u_Pulse) annotation(
        Line(points = {{-59, 23.6}, {-55, 23.6}, {-55, 23.6}, {-51, 23.6}, {-51, 23.6}, {-50, 23.6}, {-50, 23.6}, {-49, 23.6}}, color = {255, 0, 255}));
      connect(sW_VolumeCtrl.y_SSR, hW_VolumeCtrlCircuit.u_pumpSSR) annotation(
        Line(points = {{-29, 30}, {-21, 30}, {-21, 48}, {-71, 48}, {-71, 38}, {-71, 38}}, color = {255, 0, 255}));
      connect(sW_TempCtrl1.y_heaterSSR, hW_TempCtrlCircuit1.u_heaterSSR) annotation(
        Line(points = {{21, -50}, {40, -50}, {40, -20}, {-79, -20}, {-79, -42}, {-59, -42}}, color = {255, 0, 255}));
      connect(hW_TempCtrlCircuit1.y_tempSignal, sW_TempCtrl1.u_tempSignal) annotation(
        Line(points = {{-39, -50}, {2, -50}}, color = {255, 127, 0}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})),
        experiment(StartTime = 0, StopTime = 30, Tolerance = 1e-05, Interval = 0.01),
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS"));
    end Test_PowerCircuit;

    model Test_SolenoidPump
      replaceable package Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
      Modelica.Fluid.Sources.FixedBoundary boundaryA(redeclare replaceable package Medium = Medium, nPorts = 2, p = system.p_ambient) annotation(
        Placement(visible = true, transformation(origin = {-50, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner Modelica.Fluid.System system(energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, m_flow_nominal = 0.004, m_flow_small = 1e-4, p_ambient = 100000, use_eps_Re = true) annotation(
        Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyLib.Fluid.Assemblies.SolenoidPump solenoidPump(redeclare replaceable package Medium = Medium, R_Resistor = 165, c_spring = 6000, k_EMF = 250, pistonCrossArea = 0.006 ^ 2 * Modelica.Constants.pi, pistonLength = 0.02, pistonMass = 0.02, portDia = 0.005, springLength = 0.02) annotation(
        Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Fluid.Vessels.ClosedVolume boiler(redeclare replaceable package Medium = Medium, V = 250e-06, nPorts = 2, use_HeatTransfer = true, portsData = {Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter = 0.005), Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter = 0.005)}) annotation(
        Placement(visible = true, transformation(origin = {50, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      MyLib.Fluid.Valves.CheckValve filterBasket(redeclare replaceable package Medium = Medium, diameter = 0.005, dp_open = 500000, zeta = 2) annotation(
        Placement(visible = true, transformation(origin = {20, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Electrical.Analog.Sources.SineVoltage powerSource(V = 230, freqHz = 50) annotation(
        Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(visible = true, transformation(origin = {40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Semiconductors.Diode pumpDiode(T = system.T_ambient) annotation(
        Placement(visible = true, transformation(origin = {-20, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    equation
      connect(boiler.ports[2], filterBasket.port_a) annotation(
        Line(points = {{50, -20}, {20, -20}, {20, -40}, {20, -40}}, color = {0, 127, 255}, thickness = 0.5));
      connect(solenoidPump.port_b, boiler.ports[1]) annotation(
        Line(points = {{10, -10}, {20, -10}, {20, -20}, {50, -20}, {50, -20}}, color = {0, 127, 255}));
      connect(filterBasket.port_b, boundaryA.ports[2]) annotation(
        Line(points = {{20, -60}, {20, -80}, {-40, -80}}, color = {0, 127, 255}));
      connect(boundaryA.ports[1], solenoidPump.port_a) annotation(
        Line(points = {{-40, -80}, {-20, -80}, {-20, -10}, {-10, -10}}, color = {0, 127, 255}, thickness = 0.5));
      connect(pumpDiode.n, solenoidPump.p) annotation(
        Line(points = {{-20, 20}, {-20, 20}, {-20, -4}, {-10, -4}, {-10, -4}}, color = {0, 0, 255}));
      connect(powerSource.p, pumpDiode.n) annotation(
        Line(points = {{-10, 60}, {-20, 60}, {-20, 20}, {-20, 20}}, color = {0, 0, 255}));
      connect(ground1.p, powerSource.n) annotation(
        Line(points = {{40, 60}, {10, 60}}, color = {0, 0, 255}));
      connect(solenoidPump.n, ground1.p) annotation(
        Line(points = {{10, -4}, {20, -4}, {20, 60}, {40, 60}}, color = {0, 0, 255}));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})),
        experiment(StartTime = 0, StopTime = 5, Tolerance = 0.0001, Interval = 0.001),
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS"));
    end Test_SolenoidPump;





    model Test_CheckValve
      replaceable package Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
      Modelica.Fluid.Sources.FixedBoundary boundaryB(redeclare replaceable package Medium = Medium, T = system.T_ambient, nPorts = 1, p = system.p_ambient) annotation(
        Placement(visible = true, transformation(origin = {70, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Fluid.Sources.FixedBoundary boundaryA(redeclare replaceable package Medium = Medium, T = system.T_ambient, nPorts = 1, p = system.p_ambient + 900000) annotation(
        Placement(visible = true, transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner Modelica.Fluid.System system(use_eps_Re = true) annotation(
        Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyLib.Fluid.Valves.CheckValve checkValve(redeclare replaceable package Medium = Medium, diameter = 0.0008, dp_open = 500000, zeta = 2) annotation(
        Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(checkValve.port_b, boundaryB.ports[1]) annotation(
        Line(points = {{10, -30}, {60, -30}, {60, -30}, {60, -30}}, color = {0, 127, 255}));
      connect(boundaryA.ports[1], checkValve.port_a) annotation(
        Line(points = {{-60, -30}, {-10, -30}, {-10, -30}, {-10, -30}}, color = {0, 127, 255}, thickness = 0.5));
      annotation(
        Icon(coordinateSystem(grid = {2, 8})),
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-05, Interval = 0.01),
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS"));
    end Test_CheckValve;

    model Test_PulseFlowMeter
      replaceable package Medium = Modelica.Media.Water.StandardWaterOnePhase constrainedby Modelica.Media.Interfaces.PartialMedium;
      Modelica.Fluid.Sources.FixedBoundary boundaryB(redeclare replaceable package Medium = Medium, T = system.T_ambient, nPorts = 1, p = system.p_ambient) annotation(
        Placement(visible = true, transformation(origin = {70, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Fluid.Sources.FixedBoundary boundaryA(redeclare replaceable package Medium = Medium, T = system.T_ambient, nPorts = 1, p = system.p_ambient + 10000) annotation(
        Placement(visible = true, transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner Modelica.Fluid.System system(momentumDynamics = Modelica.Fluid.Types.Dynamics.DynamicFreeInitial) annotation(
        Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyLib.Fluid.Valves.CheckValve checkValve(redeclare replaceable package Medium = Medium, diameter = 0.005, zeta = 1) annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MyLib.Fluid.Sensors.FlowTransmitter flowTransmitter1(redeclare replaceable package Medium = Medium, volumeFlowPerPulse = 0.0001) annotation(
        Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interaction.Show.BooleanValue booleanValue1 annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(flowTransmitter1.V_flowPulseSignal, booleanValue1.activePort) annotation(
        Line(points = {{-30, -18}, {-30, 0}, {-10, 0}}, color = {255, 0, 255}));
      connect(flowTransmitter1.port_b, checkValve.port_a) annotation(
        Line(points = {{-20, -30}, {20, -30}, {20, -30}, {20, -30}}, color = {0, 127, 255}));
      connect(boundaryA.ports[1], flowTransmitter1.port_a) annotation(
        Line(points = {{-60, -30}, {-40, -30}, {-40, -30}, {-40, -30}}, color = {0, 127, 255}, thickness = 0.5));
      connect(checkValve.port_b, boundaryB.ports[1]) annotation(
        Line(points = {{40, -30}, {60, -30}}, color = {0, 127, 255}));
      annotation(
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-05, Interval = 0.01),
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS"));
    end Test_PulseFlowMeter;
    annotation(
      Icon(coordinateSystem(grid = {2, 8})));
  end Testblocks;
  annotation(
    uses(Modelica(version = "3.2.2"), BuildingSystems(version = "0.1")),
    Icon(graphics = {Bitmap(extent = {{-80, 80}, {80, -80}}, imageSource = "iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAMAAACdt4HsAAAAhFBMVEXw8PDYACdtpUT////bHT99rln09PT5+/j5+fn89vf39/d2qlDeN1axzpzoeI3aEDXF2rbsnauMt2zxs77wq7jO4MHgRmKUvHb42+Dq8eTZCi/xvMbeMVCItWfjWXKfwoTof5LX5c330tj55Ojv9Ov87/G40qXqiJrhTmmYvn3lZ36mx48d3yLKAAABXElEQVRYhaXXXXeCMAwG4CQFCuIENoXpNj82xbn9//83JvNMKNWWNxdckeecQtskpGwRx5MoioiaxySOra+RJbvJ7EZkMQYAnVf97DbKInEAdB1KOAwEHBwP94A8FbEDzFlxE9ifRG4DzOWHHVikch/gbG4DahEXgHk7DHyLK8BfA4A+iTvAn4kBbMQH4HUfqMUP4GMXWIgvwE/XwDL1B7LnK6ASf4DLf6C7AFeA5xdAh+OAIPkDHmQcwLMW0OlYIEvOQN7Pdwa4OAOP44HpL7A08t0Bfm8A4xP6ALMGMFfgAUwVaTPfA1gltIMAfqEaA7a0wYA1VRhQ0hsGvFL/JHoCARknyQ9Y4QC8BPgjwr8R3kjwVoYPE3yc4QsFv9LgSxW+1vHCApc2uLji5R1uMPAWB2+y4DYPbzTxVhdvtuF2X+EDBz7y4EOXgsc+hQ+eCh5924CG74vhNv7/AKX5I11MVViAAAAAAElFTkSuQmCC")}));
end MyGaggia;