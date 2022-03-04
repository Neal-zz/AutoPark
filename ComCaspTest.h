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

	void setFocusVoltage(const float& focusVoltageIn); // 24-70
	float getFocusVoltage() const { return focusVoltage; };

	bool getComStatus() const { return comStatus; };

private:
	float focusVoltage; // focus lens voltage
	bool comStatus; // true: connected.
};