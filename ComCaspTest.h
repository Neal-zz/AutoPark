// change focus of camera lens.

#pragma once

#include "ComCasp.h"

#include <iostream>

class ComCaspTest
{
public:
	ComCaspTest();
	void comConnect();
	void comClose();

	void setFocusNum(const double& focusNumIn); // 24-70
	double getFocusNum() const { return focusNum; };

	bool getComStatus() const { return comStatus; };

private:
	double focusNum; // focus lens voltage
	bool comStatus; // true: connected.
};