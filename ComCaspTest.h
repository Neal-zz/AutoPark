// change focus of camera lens.

#pragma once

#include "ComCasp.h"

#include <iostream>
//#include <afx.h>

class ComCaspTest
{
public:
	ComCaspTest();
	void comConnect();
	void setFocusNum(const double& focusNumIn); // 24-70
	void comClose();
	double getFocusNum() const { return focusNum; };

private:
	double focusNum; // focus lens voltage
	bool comStatus; // true: connected.
};