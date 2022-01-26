#pragma once

#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>


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
			cv::Mat img;
			try
			{
				GxCamTest* pGxCam = (GxCamTest*)pUserParam;

				//显示图像
				img.create(objImageDataPointer->GetHeight(), objImageDataPointer->GetWidth(), CV_8UC1); // TODO: 8 bit, unsigned, 1 channel.
				void *pRaw8Buffer = NULL;
				pRaw8Buffer = objImageDataPointer->ConvertToRaw8(GX_BIT_4_11);
				memcpy(img.data, pRaw8Buffer, (objImageDataPointer->GetHeight()) * (objImageDataPointer->GetWidth()));
				cv::flip(img, img, 0);
				cv::imshow("sss", img);
				cv::waitKey(1);

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
	BOOL openDevice();
	void closeDevice();
	void startSnap();
	void stopSnap();
	
	void softTrigger();



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
};



