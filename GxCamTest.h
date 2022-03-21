#pragma once

#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>
#include <direct.h>
#include <io.h>

typedef struct ImgMutex {

	ImgMutex()
		: mutex(1)
	{

	}

	bool lock() {
		if (mutex == 0) { 
			//std::cout << "threads collide, i'm waiting...\n";
			return false; 
		}
		mutex = 0;
		return true;
	}
	bool unlock() {
		if (mutex == 1) {
			std::cout << "imgMutex Error!!!" << std::endl;
		}
		mutex = 1;
		return true;
	}

	int mutex;
	cv::Mat imgFlow;
}ImgMutex;

class GxCamTest
{
	//---------------------------------------------------------------------------------
	/**
	\brief   �û��̳вɼ��¼�������
	*/
	//----------------------------------------------------------------------------------
	class CSampleCaptureEventHandler :public ICaptureEventHandler
	{
		//---------------------------------------------------------------------------------
		/**
		\brief   �ɼ��ص�����
		\param   objImageDataPointer      ͼ�������
		\param   pUserFrame                   �û�����
		\return  ��
		*/
		//----------------------------------------------------------------------------------
		void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam)
		{

			try
			{
				GxCamTest* pGxCam = (GxCamTest*)pUserParam;

				// update image at pGxCam.imgFlow
				pGxCam->writeImgFlow(objImageDataPointer);

				//�ж��Ƿ���Ҫ����ͼ��
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
	CSampleCaptureEventHandler* m_pSampleCaptureEventHandle; // �ص�����ָ�롣

	double m_dEditShutterValue;
	double m_dEditGainValue;
	double m_dEditBalanceRatioValue;
	bool m_bCheckSaveBmp;

	bool m_bIsOpen;
	bool m_bIsSnap;
	bool m_bColorFilter;
	bool m_bTriggerMode;
	bool m_bTriggerSource;
	bool m_bTriggerActive; // �Ƿ�֧�ִ������ԡ�
	bool m_bBalanceWhiteAuto; // �Ƿ�֧���Զ���ƽ�⡣
	bool m_bBalanceWhiteRatioSelect; // �Ƿ�֧�ְ�ƽ��ͨ��ѡ��
	double m_dShutterValueMax; // �ع�ʱ�����ֵ��
	double m_dShutterValueMin; // �ع�ʱ����Сֵ��
	double m_dGainValueMax; // �������ֵ��
	double m_dGainValueMin; // ������Сֵ��
	double m_dBalanceWhiteRatioMax; // �Զ���ƽ��ϵ�����ֵ��
	double m_dBalanceWhiteRatioMin; // �Զ���ƽ��ϵ����Сֵ��
	gxstring m_strBalanceWhiteAutoMode; // ��¼�Զ���ƽ�ⷽʽ��
	std::string m_strSavePath; // ͼ�񱣴�·����

	int m_iImgNum;
	ImgMutex imgMutex;
	//std::chrono::system_clock::time_point startGx;
};