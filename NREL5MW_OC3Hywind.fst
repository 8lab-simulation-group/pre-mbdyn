--------------------------------------------------------------------------------
------- FAST INPUT FILE --------------------------------------------------------
NREL 5.0 MW Baseline Wind Turbine for Use in Offshore Analysis.
Properties from Dutch Offshore Wind Energy Converter (DOWEC) 6MW Pre-Design (10046_009.pdf) and REpower 5M 5MW (5m_uk.pdf); Compatible with FAST v6.0.
---------------------- SIMULATION CONTROL --------------------------------------
False       Echo        - Echo input data to "echo.out" (flag)
   2        ADAMSPrep   - ADAMS preprocessor mode {1: Run FAST, 2: use FAST as a preprocessor to create an ADAMS model, 3: do both} (switch)
   1        AnalMode    - Analysis mode {1: Run a time-marching simulation, 2: create a periodic linearized model} (switch)
   3        NumBl       - Number of blades (-)
 300.0  630.0      TMax        - Total run time (s)
   0.0125   DT          - Integration time step (s)
---------------------- TURBINE CONTROL -----------------------------------------
   0        YCMode      - Yaw control mode {0: none, 1: user-defined from routine UserYawCont, 2: user-defined from Simulink} (switch)
9999.9      TYCOn       - Time to enable active yaw control (s) [unused when YCMode=0]
   0 1        PCMode      - Pitch control mode {0: none, 1: user-defined from routine PitchCntrl, 2: user-defined from Simulink} (switch)
   0.0      TPCOn       - Time to enable active pitch control (s) [unused when PCMode=0]
   2        VSContrl    - Variable-speed control mode {0: none, 1: simple VS, 2: user-defined from routine UserVSCont, 3: user-defined from Simulink} (switch)
9999.9      VS_RtGnSp   - Rated generator speed for simple variable-speed generator control (HSS side) (rpm) [used only when VSContrl=1]
9999.9      VS_RtTq     - Rated generator torque/constant generator torque in Region 3 for simple variable-speed generator control (HSS side) (N-m) [used only when VSContrl=1]
9999.9      VS_Rgn2K    - Generator torque constant in Region 2 for simple variable-speed generator control (HSS side) (N-m/rpm^2) [used only when VSContrl=1]
9999.9      VS_SlPc     - Rated generator slip percentage in Region 2 1/2 for simple variable-speed generator control (%) [used only when VSContrl=1]
   2        GenModel    - Generator model {1: simple, 2: Thevenin, 3: user-defined from routine UserGen} (switch) [used only when VSContrl=0]
True        GenTiStr    - Method to start the generator {T: timed using TimGenOn, F: generator speed using SpdGenOn} (flag)
True        GenTiStp    - Method to stop the generator {T: timed using TimGenOf, F: when generator power = 0} (flag)
9999.9      SpdGenOn    - Generator speed to turn on the generator for a startup (HSS speed) (rpm) [used only when GenTiStr=False]
   0.0      TimGenOn    - Time to turn on the generator for a startup (s) [used only when GenTiStr=True]
9999.9      TimGenOf    - Time to turn off the generator (s) [used only when GenTiStp=True]
   1        HSSBrMode   - HSS brake model {1: simple, 2: user-defined from routine UserHSSBr} (switch)
9999.9      THSSBrDp    - Time to initiate deployment of the HSS brake (s)
9999.9      TiDynBrk    - Time to initiate deployment of the dynamic generator brake [CURRENTLY IGNORED] (s)
9999.9      TTpBrDp(1)  - Time to initiate deployment of tip brake 1 (s)
9999.9      TTpBrDp(2)  - Time to initiate deployment of tip brake 2 (s)
9999.9      TTpBrDp(3)  - Time to initiate deployment of tip brake 3 (s) [unused for 2 blades]
9999.9      TBDepISp(1) - Deployment-initiation speed for the tip brake on blade 1 (rpm)
9999.9      TBDepISp(2) - Deployment-initiation speed for the tip brake on blade 2 (rpm)
9999.9      TBDepISp(3) - Deployment-initiation speed for the tip brake on blade 3 (rpm) [unused for 2 blades]
9999.9      TYawManS    - Time to start override yaw maneuver and end standard yaw control (s)
9999.9      TYawManE    - Time at which override yaw maneuver reaches final yaw angle (s)
   0.0      NacYawF     - Final yaw angle for yaw maneuvers (degrees)
9999.9      TPitManS(1) - Time to start override pitch maneuver for blade 1 and end standard pitch control (s)
9999.9      TPitManS(2) - Time to start override pitch maneuver for blade 2 and end standard pitch control (s)
9999.9      TPitManS(3) - Time to start override pitch maneuver for blade 3 and end standard pitch control (s) [unused for 2 blades]
9999.9      TPitManE(1) - Time at which override pitch maneuver for blade 1 reaches final pitch (s)
9999.9      TPitManE(2) - Time at which override pitch maneuver for blade 2 reaches final pitch (s)
9999.9      TPitManE(3) - Time at which override pitch maneuver for blade 3 reaches final pitch (s) [unused for 2 blades]
   0.0      BlPitch(1)  - Blade 1 initial pitch (degrees)
   0.0      BlPitch(2)  - Blade 2 initial pitch (degrees)
   0.0      BlPitch(3)  - Blade 3 initial pitch (degrees) [unused for 2 blades]
   0.0      B1PitchF(1) - Blade 1 final pitch for pitch maneuvers (degrees)
   0.0      B1PitchF(2) - Blade 2 final pitch for pitch maneuvers (degrees)
   0.0      B1PitchF(3) - Blade 3 final pitch for pitch maneuvers (degrees) [unused for 2 blades]
---------------------- ENVIRONMENTAL CONDITIONS --------------------------------
   9.80665  Gravity     - Gravitational acceleration (m/s^2)
---------------------- FEATURE FLAGS -------------------------------------------
True        FlapDOF1    - First flapwise blade mode DOF (flag)
True        FlapDOF2    - Second flapwise blade mode DOF (flag)
True        EdgeDOF     - First edgewise blade mode DOF (flag)
False       TeetDOF     - Rotor-teeter DOF (flag) [unused for 3 blades]
True        DrTrDOF     - Drivetrain rotational-flexibility DOF (flag)
True        GenDOF      - Generator DOF (flag)
True        YawDOF      - Yaw DOF (flag)
True        TwFADOF1    - First fore-aft tower bending-mode DOF (flag)
True        TwFADOF2    - Second fore-aft tower bending-mode DOF (flag)
True        TwSSDOF1    - First side-to-side tower bending-mode DOF (flag)
True        TwSSDOF2    - Second side-to-side tower bending-mode DOF (flag)
True        CompAero    - Compute aerodynamic forces (flag)
False       CompNoise   - Compute aerodynamic noise (flag)
---------------------- INITIAL CONDITIONS --------------------------------------
   0.0      OoPDefl     - Initial out-of-plane blade-tip displacement (meters)
   0.0      IPDefl      - Initial in-plane blade-tip deflection (meters)
   0.0      TeetDefl    - Initial or fixed teeter angle (degrees) [unused for 3 blades]
   0.0      Azimuth     - Initial azimuth angle for blade 1 (degrees)
  12.1      RotSpeed    - Initial or fixed rotor speed (rpm)
   0.0      NacYaw      - Initial or fixed nacelle-yaw angle (degrees)
   0.0      TTDspFA     - Initial fore-aft tower-top displacement (meters)
   0.0      TTDspSS     - Initial side-to-side tower-top displacement (meters)
---------------------- TURBINE CONFIGURATION -----------------------------------
  63.0      TipRad      - The distance from the rotor apex to the blade tip (meters)
   1.5      HubRad      - The distance from the rotor apex to the blade root (meters)
   1        PSpnElN     - Number of the innermost blade element which is still part of the pitchable portion of the blade for partial-span pitch control [1 to BldNodes] [CURRENTLY IGNORED] (-)
   0.0      UndSling    - Undersling length [distance from teeter pin to the rotor apex] (meters) [unused for 3 blades]
   0.0      HubCM       - Distance from rotor apex to hub mass [positive downwind] (meters)
  -5.01910  OverHang    - Distance from yaw axis to rotor apex [3 blades] or teeter pin [2 blades] (meters)
   1.9      NacCMxn     - Downwind distance from the tower-top to the nacelle CM (meters)
   0.0      NacCMyn     - Lateral  distance from the tower-top to the nacelle CM (meters)
   1.75     NacCMzn     - Vertical distance from the tower-top to the nacelle CM (meters)
  87.6      TowerHt     - Height of tower above ground level [onshore] or MSL [offshore] (meters)
   1.96256  Twr2Shft    - Vertical distance from the tower-top to the rotor shaft (meters)
   0.0      TwrRBHt     - Tower rigid base height (meters)
  -5.0      ShftTilt    - Rotor shaft tilt angle (degrees)
   0.0      Delta3      - Delta-3 angle for teetering rotors (degrees) [unused for 3 blades]
  -2.5      PreCone(1)  - Blade 1 cone angle (degrees)
  -2.5      PreCone(2)  - Blade 2 cone angle (degrees)
  -2.5      PreCone(3)  - Blade 3 cone angle (degrees) [unused for 2 blades]
   0.0      AzimB1Up    - Azimuth value to use for I/O when blade 1 points up (degrees)
---------------------- MASS AND INERTIA ----------------------------------------
   0.0      YawBrMass   - Yaw bearing mass (kg)
 240.00E3   NacMass     - Nacelle mass (kg)
  56.78E3   HubMass     - Hub mass (kg)
   0.0      TipMass(1)  - Tip-brake mass, blade 1 (kg)
   0.0      TipMass(2)  - Tip-brake mass, blade 2 (kg)
   0.0      TipMass(3)  - Tip-brake mass, blade 3 (kg) [unused for 2 blades]
2607.89E3   NacYIner    - Nacelle inertia about yaw axis (kg m^2)
 534.116    GenIner     - Generator inertia about HSS (kg m^2)
 115.926E3  HubIner     - Hub inertia about rotor axis [3 blades] or teeter axis [2 blades] (kg m^2)
---------------------- DRIVETRAIN ----------------------------------------------
 100.0      GBoxEff     - Gearbox efficiency (%)
  94.4      GenEff      - Generator efficiency [ignored by the Thevenin and user-defined generator models] (%)
  97.0      GBRatio     - Gearbox ratio (-)
False       GBRevers    - Gearbox reversal {T: if rotor and generator rotate in opposite directions} (flag)
  28.1162E3 HSSBrTqF    - Fully deployed HSS-brake torque (N-m)
   0.6      HSSBrDT     - Time for HSS-brake to reach full deployment once initiated (sec) [used only when HSSBrMode=1]
            DynBrkFi    - File containing a mech-gen-torque vs HSS-speed curve for a dynamic brake [CURRENTLY IGNORED] (quoted string)
 867.637E6  DTTorSpr    - Drivetrain torsional spring (N-m/rad)
   6.215E6  DTTorDmp    - Drivetrain torsional damper (N-m/(rad/s))
---------------------- SIMPLE INDUCTION GENERATOR ------------------------------
9999.9      SIG_SlPc    - Rated generator slip percentage (%) [used only when VSContrl=0 and GenModel=1]
9999.9      SIG_SySp    - Synchronous (zero-torque) generator speed (rpm) [used only when VSContrl=0 and GenModel=1]
9999.9      SIG_RtTq    - Rated torque (N-m) [used only when VSContrl=0 and GenModel=1]
9999.9      SIG_PORt    - Pull-out ratio (Tpullout/Trated) (-) [used only when VSContrl=0 and GenModel=1]
---------------------- THEVENIN-EQUIVALENT INDUCTION GENERATOR -----------------
9999.9      TEC_Freq    - Line frequency [50 or 60] (Hz) [used only when VSContrl=0 and GenModel=2]
9998        TEC_NPol    - Number of poles [even integer > 0] (-) [used only when VSContrl=0 and GenModel=2]
9999.9      TEC_SRes    - Stator resistance (ohms) [used only when VSContrl=0 and GenModel=2]
9999.9      TEC_RRes    - Rotor resistance (ohms) [used only when VSContrl=0 and GenModel=2]
9999.9      TEC_VLL     - Line-to-line RMS voltage (volts) [used only when VSContrl=0 and GenModel=2]
9999.9      TEC_SLR     - Stator leakage reactance (ohms) [used only when VSContrl=0 and GenModel=2]
9999.9      TEC_RLR     - Rotor leakage reactance (ohms) [used only when VSContrl=0 and GenModel=2]
9999.9      TEC_MR      - Magnetizing reactance (ohms) [used only when VSContrl=0 and GenModel=2]
---------------------- PLATFORM ------------------------------------------------
   3        PtfmModel   - Platform model {0: none, 1: onshore, 2: fixed bottom offshore, 3: floating offshore} (switch)
"NREL5MW_OC3Hywind_Ptfm.dat"     PtfmFile    - Name of file containing platform properties (quoted string) [unused when PtfmModel=0]
---------------------- TOWER ---------------------------------------------------
  20        TwrNodes    - Number of tower nodes used for analysis (-)
"NRELOffshrBsline5MW_Tower_OC3Hywind.dat"        TwrFile     - Name of file containing tower properties (quoted string)
---------------------- NACELLE-YAW ---------------------------------------------
9028.32E6   YawSpr      - Nacelle-yaw spring constant (N-m/rad)
  19.16E6   YawDamp     - Nacelle-yaw damping constant (N-m/(rad/s))
   0.0      YawNeut     - Neutral yaw position--yaw spring force is zero at this yaw (degrees)
---------------------- FURLING -------------------------------------------------
False       Furling     - Read in additional model properties for furling turbine (flag)
            FurlFile    - Name of file containing furling properties (quoted string) [unused when Furling=False]
---------------------- ROTOR-TEETER --------------------------------------------
   0        TeetMod     - Rotor-teeter spring/damper model {0: none, 1: standard, 2: user-defined from routine UserTeet} (switch) [unused for 3 blades]
   0.0      TeetDmpP    - Rotor-teeter damper position (degrees) [used only for 2 blades and when TeetMod=1]
   0.0      TeetDmp     - Rotor-teeter damping constant (N-m/(rad/s)) [used only for 2 blades and when TeetMod=1]
   0.0      TeetCDmp    - Rotor-teeter rate-independent Coulomb-damping moment (N-m) [used only for 2 blades and when TeetMod=1]
   0.0      TeetSStP    - Rotor-teeter soft-stop position (degrees) [used only for 2 blades and when TeetMod=1]
   0.0      TeetHStP    - Rotor-teeter hard-stop position (degrees) [used only for 2 blades and when TeetMod=1]
   0.0      TeetSSSp    - Rotor-teeter soft-stop linear-spring constant (N-m/rad) [used only for 2 blades and when TeetMod=1]
   0.0      TeetHSSp    - Rotor-teeter hard-stop linear-spring constant (N-m/rad) [used only for 2 blades and when TeetMod=1]
---------------------- TIP-BRAKE -----------------------------------------------
   0.0      TBDrConN    - Tip-brake drag constant during normal operation, Cd*Area (m^2)
   0.0      TBDrConD    - Tip-brake drag constant during fully-deployed operation, Cd*Area (m^2)
   0.0      TpBrDT      - Time for tip-brake to reach full deployment once released (sec)
---------------------- BLADE ---------------------------------------------------
"NRELOffshrBsline5MW_Blade.dat"                  BldFile(1)  - Name of file containing properties for blade 1 (quoted string)
"NRELOffshrBsline5MW_Blade.dat"                  BldFile(2)  - Name of file containing properties for blade 2 (quoted string)
"NRELOffshrBsline5MW_Blade.dat"                  BldFile(3)  - Name of file containing properties for blade 3 (quoted string) [unused for 2 blades]
---------------------- AERODYN -------------------------------------------------
"NRELOffshrBsline5MW_AeroDyn.ipt"                ADFile      - Name of file containing AeroDyn input parameters (quoted string)
---------------------- NOISE ---------------------------------------------------
            NoiseFile   - Name of file containing aerodynamic noise input parameters (quoted string) [used only when CompNoise=True]
---------------------- ADAMS ---------------------------------------------------
"NRELOffshrBsline5MW_ADAMSSpecific.dat"          ADAMSFile   - Name of file containing ADAMS-specific input parameters (quoted string) [unused when ADAMSPrep=1]
---------------------- LINEARIZATION CONTROL -----------------------------------
"NRELOffshrBsline5MW_Linear.dat"                 LinFile     - Name of file containing FAST linearization parameters (quoted string) [unused when AnalMode=1]
---------------------- OUTPUT --------------------------------------------------
True        SumPrint    - Print summary data to "<RootName>.fsm" (flag)
3           OutFileFmt  - Format for tabular (time-marching) output file(s) (1: text file [<RootName>.out], 2: binary file [<RootName>.outb], 3: both) (switch)
True        TabDelim    - Generate a tab-delimited tabular output file. (flag)
"ES10.3E2"  OutFmt      - Format used for tabular output except time.  Resulting field should be 10 characters. (quoted string)  [not checked for validity!]
  0.0  30.0      TStart      - Time to begin tabular output (s)
   1        DecFact     - Decimation factor for tabular output {1: output every time step} (-)
   1.0      SttsTime    - Amount of time between screen status messages (sec)
  -3.09528  NcIMUxn     - Downwind distance from the tower-top to the nacelle IMU (meters)
   0.0      NcIMUyn     - Lateral  distance from the tower-top to the nacelle IMU (meters)
   2.23336  NcIMUzn     - Vertical distance from the tower-top to the nacelle IMU (meters)
   1.912    ShftGagL    - Distance from rotor apex [3 blades] or teeter pin [2 blades] to shaft strain gages [positive for upwind rotors] (meters)
   0        NTwGages    - Number of tower nodes that have strain gages for output [0 to 9] (-)
  10        TwrGagNd    - List of tower nodes that have strain gages [1 to TwrNodes] (-) [unused if NTwGages=0]
   1        NBlGages    - Number of blade nodes that have strain gages for output [0 to 9] (-)
   9        BldGagNd    - List of blade nodes that have strain gages [1 to BldNodes] (-) [unused if NBlGages=0]
            OutList     - The next line(s) contains a list of output parameters.  See OutList.txt for a listing of available output channels, (-)
"WindVxi  , WindVyi  , WindVzi"                              - Longitudinal, lateral, and vertical wind speeds
"WaveElev"                                                   - Wave elevation at the platform reference point
"Wave1Vxi , Wave1Vyi , Wave1Vzi"                             - Longitudinal, lateral, and vertical wave particle velocities at platform node 1 (approx. platform reference point)
"Wave1Axi , Wave1Ayi , Wave1Azi"                             - Longitudinal, lateral, and vertical wave particle accelerations at platform node 1 (approx. platform reference point)
"GenPwr   , GenTq"                                           - Electrical generator power and torque
"HSSBrTq"                                                    - High-speed shaft brake torque
"BldPitch1, BldPitch2, BldPitch3"                            - Pitch angles for blades 1, 2, and 3
"Azimuth"                                                    - Blade 1 azimuth angle
"RotSpeed , GenSpeed"                                        - Low-speed shaft and high-speed shaft speeds
"NacYaw   , NacYawErr"                                       - Nacelle yaw angle and nacelle yaw error estimate
"OoPDefl1 , IPDefl1  , TwstDefl1"                            - Blade 1 out-of-plane and in-plane deflections and tip twist
"OoPDefl2 , IPDefl2  , TwstDefl2"                            - Blade 2 out-of-plane and in-plane deflections and tip twist
"OoPDefl3 , IPDefl3  , TwstDefl3"                            - Blade 3 out-of-plane and in-plane deflections and tip twist
"TwrClrnc1, TwrClrnc2, TwrClrnc3"                            - Tip-to-tower clearance estimate for blades 1, 2, and 3
"NcIMUTAxs, NcIMUTAys, NcIMUTAzs"                            - Nacelle IMU translational accelerations (absolute) in the nonrotating, shaft coordinate system
"TTDspFA  , TTDspSS  , TTDspTwst"                            - Tower fore-aft and side-to-side displacements and top twist
"PtfmSurge, PtfmSway , PtfmHeave"                            - Platform translational surge, sway, and heave displacements
"PtfmRoll , PtfmPitch, PtfmYaw"                              - Platform rotational roll, pitch and yaw displacements
"PtfmTAxt , PtfmTAyt , PtfmTAzt"                             - Platform translation accelerations (absolute) in the tower-base coordinate system
"RootFxc1 , RootFyc1 , RootFzc1"                             - Out-of-plane shear, in-plane shear, and axial forces at the root of blade 1
"RootMxc1 , RootMyc1 , RootMzc1"                             - In-plane bending, out-of-plane bending, and pitching moments at the root of blade 1
"RootFxc2 , RootFyc2 , RootFzc2"                             - Out-of-plane shear, in-plane shear, and axial forces at the root of blade 2
"RootMxc2 , RootMyc2 , RootMzc2"                             - In-plane bending, out-of-plane bending, and pitching moments at the root of blade 2
"RootFxc3 , RootFyc3 , RootFzc3"                             - Out-of-plane shear, in-plane shear, and axial forces at the root of blade 3
"RootMxc3 , RootMyc3 , RootMzc3"                             - In-plane bending, out-of-plane bending, and pitching moments at the root of blade 3
"Spn1MLxb1, Spn1MLyb1, Spn1MLzb1"                            - Blade 1 local edgewise bending, flapwise bending, and pitching moments at span station 1 (approx. 50% span)
"Spn1MLxb2, Spn1MLyb2, Spn1MLzb2"                            - Blade 2 local edgewise bending, flapwise bending, and pitching moments at span station 1 (approx. 50% span)
"Spn1MLxb3, Spn1MLyb3, Spn1MLzb3"                            - Blade 3 local edgewise bending, flapwise bending, and pitching moments at span station 1 (approx. 50% span)
"RotThrust, LSSGagFya, LSSGagFza"                            - Rotor thrust and low-speed shaft 0- and 90-rotating shear forces at the main bearing
"RotTorq  , LSSGagMya, LSSGagMza"                            - Rotor torque and low-speed shaft 0- and 90-rotating bending moments at the main bearing
"YawBrFxp , YawBrFyp , YawBrFzp"                             - Fore-aft shear, side-to-side shear, and vertical forces at the top of the tower (not rotating with nacelle yaw)
"YawBrMxp , YawBrMyp , YawBrMzp"                             - Side-to-side bending, fore-aft bending, and yaw moments at the top of the tower (not rotating with nacelle yaw)
"TwrBsFxt , TwrBsFyt , TwrBsFzt"                             - Fore-aft shear, side-to-side shear, and vertical forces at the base of the tower (platform)
"TwrBsMxt , TwrBsMyt , TwrBsMzt"                             - Side-to-side bending, fore-aft bending, and yaw moments at the base of the tower (platform)
"TwHt1MLxt, TwHt1MLyt, TwHt1MLzt"                            - Local side-to-side bending, fore-aft bending, and yaw moments at tower gage 1 (approx. 50% elevation)
"Fair1Ten , Fair1Ang , Anch1Ten , Anch1Ang"                  - Line 1 fairlead and anchor effective tensions and vertical angles
"Fair2Ten , Fair2Ang , Anch2Ten , Anch2Ang"                  - Line 2 fairlead and anchor effective tensions and vertical angles
"Fair3Ten , Fair3Ang , Anch3Ten , Anch3Ang"                  - Line 3 fairlead and anchor effective tensions and vertical angles
"TipSpdRat, RotCp    , RotCt    , RotCq"                     - Rotor tip speed ratio and power, thrust, and torque coefficients
END of FAST input file (the word "END" must appear in the first 3 columns of this last line).
--------------------------------------------------------------------------------


