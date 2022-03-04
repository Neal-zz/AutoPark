#include "ComCaspTest.h"


ComCaspTest::ComCaspTest()
	: focusVoltage(24), comStatus(false)
{

}

void ComCaspTest::comConnect()
{
	// connect to the caspian.
	if (eCaspSuccess == Casp_OpenCOM())
	{
		std::cout << "caspian successfully connected!\n";
		comStatus = true;
	}
	else
	{
		//CString str;
		//str = Casp_GetErrorMsg(Casp_Status);
		std::cout << "fail to connect to the caspian!" << std::endl;
		comStatus = false;
	}
	return;
}


void ComCaspTest::setFocusVoltage(const float& focusVoltageIn)
{
	if (focusVoltage == focusVoltageIn || comStatus == false) {
		return;
	}
		
	focusVoltage = focusVoltageIn;
	// change focus lens.
	eCOMCaspErr Casp_Status;
	Casp_Status = Casp_SetFocus(static_cast<double>(focusVoltage));
	if (eCaspSuccess != Casp_Status)
	{
		//CString str;
		//str = Casp_GetErrorMsg(Casp_Status);
		std::cout << "fail to set focus voltage!" << std::endl;
	}
	return;
}

void ComCaspTest::comClose()
{
	// close the caspian.
	Casp_CloseCOM();
	comStatus = false;
	focusVoltage = 24;
	return;
}




