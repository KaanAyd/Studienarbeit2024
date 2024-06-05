# Open TruckMaker 11.0.1
IPC::System::Simple::system("'C:\ProgramData\Microsoft\Windows\Start Menu\Programs\IPG\CarMaker 11.0.1'");

# Wait for TruckMaker to start
sleep(5);

# Set the path to the TestRun
my $testRunPath = "C:\CM_Projects\Studienarbeit\Data\TestRun\Test1";

# Select the TestRun
IPC::System::Simple::system("cm_gui_testrun_open $testRunPath");

# Start the simulation
IPC::System::Simple::system("cm_gui_simulation_start");