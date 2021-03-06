#pragma once

#include "GalaxyIncludes.h"

#include <opencv2/opencv.hpp>
#include <direct.h>
#include <io.h>
#include <thread>
#include <mutex>


class GxCamTest
{
	//---------------------------------------------------------------------------------
	/**
	\brief   用户继承采集事件处理类
	*/
	//----------------------------------------------------------------------------------
	class CSampleCaptureEventHandler :public ICaptureEventHandler
	{
		//---------------------------------------------------------------------------------
		/**
		\brief   采集回调函数
		\param   objImageDataPointer      图像处理参数
		\param   pUserFrame                   用户参数
		\return  无
		*/
		//----------------------------------------------------------------------------------
		void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam)
		{

			try
			{
				GxCamTest* pGxCam = (GxCamTest*)pUserParam;

				// update image at pGxCam.imgFlow
				pGxCam->writeImgFlow(objImageDataPointer);

				//判断是否需要保存图像
				if (pGxCam->m_bCheckSaveBmp == TRUE)
				{
					pGxCam->SavePicture(objImageDataPointer);
				}
			}
			catch (CGalaxyException)
			{
				//do nothing
			}
			catch (std::exception)
			{
				//do nothing
			}
		}
	};

public:
	GxCamTest();
	bool openDevice();
	void closeDevice();
	void startSnap();
	void stopSnap();

	//void softTrigger(); // unabled
	void writeImgFlow(CImageDataPointer& objImageDataPointer);
	void setCheckSaveBmp() { m_bCheckSaveBmp = true; };

	cv::Mat getImgFlow();
private:

	void __InitParam();

	void SavePicture(CImageDataPointer& objImageDataPointer);

	CGXDevicePointer m_objDevicePtr;
	CGXStreamPointer m_objStreamPtr;
	CGXFeatureControlPointer m_objFeatureControlPtr;
	CGXFeatureControlPointer m_objStreamFeatureControlPtr;
	CSampleCaptureEventHandler* m_pSampleCaptureEventHandle; // 回调函数指针。

	double m_dEditShutterValue;
	double m_dEditGainValue;
	double m_dEditBalanceRatioValue;
	bool m_bCheckSaveBmp;

	bool m_bIsOpen;
	bool m_bIsSnap;
	bool m_bColorFilter;
	bool m_bTriggerMode;
	bool m_bTriggerSource;
	bool m_bTriggerActive; // 是否支持触发极性。
	bool m_bBalanceWhiteAuto; // 是否支持自动白平衡。
	bool m_bBalanceWhiteRatioSelect; // 是否支持白平衡通道选择。
	double m_dShutterValueMax; // 曝光时间最大值。
	double m_dShutterValueMin; // 曝光时间最小值。
	double m_dGainValueMax; // 增益最大值。
	double m_dGainValueMin; // 增益最小值。
	double m_dBalanceWhiteRatioMax; // 自动白平衡系数最大值。
	double m_dBalanceWhiteRatioMin; // 自动白平衡系数最小值。
	gxstring m_strBalanceWhiteAutoMode; // 记录自动白平衡方式。
	std::string m_strSavePath; // 图像保存路径。

	int m_iImgNum;
	cv::Mat imgFlow;
	std::mutex img_mu;
	//std::chrono::system_clock::time_point startGx;
};