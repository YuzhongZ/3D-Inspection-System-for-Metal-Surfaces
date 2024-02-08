#include <iostream>
#include <iomanip>
#include <memory>


#include "Point.h"


using namespace Pylon;
using namespace BlazeCameraParams_Params;
using namespace std;
using namespace pcl;


int main(int, char*[])
{
	int exitCode = EXIT_SUCCESS;

	try
	{
		
		PylonInitialize();
		CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
		camera.RegisterConfiguration(new CBlazeDefaultConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
		/*camera.StopGrabbing();
		camera.Close();*/
		Sample processing;
		
		exitCode = processing.run();
		//exitCode = processing.run1();
	}
	catch (GenICam::GenericException& e)
	{
		std::cerr << "Exception occurred: " << std::endl << e.GetDescription() << std::endl;
		exitCode = EXIT_FAILURE;
	}

	PylonTerminate();

	std::cout << std::endl << "Press Enter to exit." << std::endl;
	while (std::cin.get() != '\n')
		;

	return exitCode;
}



