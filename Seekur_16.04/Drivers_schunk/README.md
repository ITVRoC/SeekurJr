# Schunk drivers  
The following folders allow the configuration and monitoring of the schunk arm. Each folder works with a specific communication 
protocol: CanOpen and SMP (Schunk motion protocol)

## CanopenTools

The CanopenTools folder has executables for calibration, movement, specifications and information about the Schunk arm 
and its joints. The executables in this folder use the canopen protocol. 

**OBS:** If some joint is not available, confirm that the communication
protocol for that joint is correct. 

### Important files:
- LWA_tool.exe - Graphical interface that allows movement, temperature control, data visualization, arm status and exchange the communication protocol.
- COT_ReferenceTool.exe - Executable for calibration of arm joints.
- COT_GetErrorDetails.exe - Show detailed error messages.
- COT_ResetToSMP.exe - Choose the joint and change your communication protocol to SMP.
- SMPT_ResetToCANopen - Choose the joint and change your communication protocol to CanOpen.

## MTS

In this folder there is an executable for calibration and configuration of the robotic arm as well as manuals for the gripper and how to use the MTS.

**OBS:** If the reference change does not work with the CanOpen protocol, it must be performed with the SMP
