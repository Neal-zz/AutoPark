#include "ComCaspTest.h"


ComCaspTest::ComCaspTest()
	: focusNum(0), comStatus(false)
{

}

void ComCaspTest::comConnect()
{
	// connect to the caspian.
	eCOMCaspErr Casp_Status;
	Casp_Status = Casp_OpenCOM();
	if (eCaspSuccess == Casp_Status)
	{
		std::cout << "successfully connect!" << std::endl;
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


void ComCaspTest::setFocusNum(const double& focusNumIn)
{
	if (focusNum == focusNumIn || comStatus==false)
		return;
	focusNum = focusNumIn;
	// change focus lens.
	eCOMCaspErr Casp_Status;
	Casp_Status = Casp_SetFocus(static_cast<double>(focusNum));
	if (eCaspSuccess != Casp_Status)
	{
		//CString str;
		//str = Casp_GetErrorMsg(Casp_Status);
		std::cout << "fail to set focus num!" << std::endl;
	}
	return;
}

void ComCaspTest::comClose()
{
	// close the caspian.
	Casp_CloseCOM();
	comStatus = false;
	return;
}






