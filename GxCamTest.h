#pragma once

#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>


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
			cv::Mat img;
			try
			{
				GxCamTest* pGxCam = (GxCamTest*)pUserParam;

				//��ʾͼ��
				img.create(objImageDataPointer->GetHeight(), objImageDataPointer->GetWidth(), CV_8UC1); // TODO: 8 bit, unsigned, 1 channel.
				void *pRaw8Buffer = NULL;
				pRaw8Buffer = objImageDataPointer->ConvertToRaw8(GX_BIT_4_11);
				memcpy(img.data, pRaw8Buffer, (objImageDataPointer->GetHeight()) * (objImageDataPointer->GetWidth()));
				cv::flip(img, img, 0);
				cv::imshow("sss", img);
				cv::waitKey(1);

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
};



