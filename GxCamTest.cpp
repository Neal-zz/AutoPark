#include "GxCamTest.h"

GxCamTest::GxCamTest()
	: m_bIsOpen(false)
	, m_bIsSnap(false)
	, m_bColorFilter(false)
	, m_bTriggerMode(false) // unabled
	, m_bTriggerSource(false) // default: Software
	, m_bTriggerActive(false) // default: FallingEdge
	, m_bBalanceWhiteAuto(false)
	, m_bBalanceWhiteRatioSelect(false)
	, m_strSavePath("E:\\sjtu\\autoPark\\code\\autoPark\\autoPark\\imgGrasp")
	, m_strBalanceWhiteAutoMode("Off")
	, m_pSampleCaptureEventHandle(NULL)
	, m_bCheckSaveBmp(false)
	, m_dEditShutterValue(20000) // �ع� 20~1e6 // 20000 �Ƚϲ�����ģ��
	, m_dEditGainValue(10) // ���� 0~23.9  // ����Խ������Խ��
	, m_dEditBalanceRatioValue(0)
	, m_dShutterValueMax(0)
	, m_dShutterValueMin(0)
	, m_dGainValueMax(0)
	, m_dGainValueMin(0)
	, m_dBalanceWhiteRatioMax(0)
	, m_dBalanceWhiteRatioMin(0)
	, m_iImgNum(0)
{
	if (_access(m_strSavePath.c_str(), 0) == -1) {
		_mkdir(m_strSavePath.c_str()); // if save path doesn't exist, create one.
	}

	//startGx = std::chrono::system_clock::now();
	//imgFlow.create(2048, 2448, CV_8UC1); // img width; height; 8 bit, unsigned, 1 channel.
}

bool GxCamTest::openDevice()
{
	/*OnInitDialog()*/

	try
	{
		// initiate device.
		IGXFactory::GetInstance().Init();

		m_pSampleCaptureEventHandle = new CSampleCaptureEventHandler();

	}
	catch (CGalaxyException& e)
	{
		if (m_pSampleCaptureEventHandle != NULL)
		{
			delete m_pSampleCaptureEventHandle;
			m_pSampleCaptureEventHandle = NULL;
		}

		std::cout << e.what() << std::endl;
		return false;

	}
	catch (std::exception& e)
	{
		if (m_pSampleCaptureEventHandle != NULL)
		{
			delete m_pSampleCaptureEventHandle;
			m_pSampleCaptureEventHandle = NULL;
		}

		std::cout << e.what() << std::endl;
		return false;

	}

	/*OnBnClickedBtnOpenDevice()*/

	bool bIsDeviceOpen = false;
	bool bIsStreamOpen = false;

	try
	{
		// enumarate devices.
		GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;
		IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);

		// if no device finded.
		if (vectorDeviceInfo.size() <= 0)
		{
			std::cout << "find no device!" << std::endl;
			return false;
		}

		// open device.
		m_objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);
		bIsDeviceOpen = true;
		m_objFeatureControlPtr = m_objDevicePtr->GetRemoteFeatureControl();

		////�жϻ�ͼ�����Ƿ�Ϊ��
		//if (m_pBitmap != NULL)
		//{
		//	delete m_pBitmap;
		//	m_pBitmap = NULL;
		//}
		//
		//Ϊ��ͼ��������ڴ�
		//m_pBitmap = new CGXBitmap(m_objDevicePtr, m_pWnd);

		// if device stream count > 0, then open stream.
		int nStreamCount = m_objDevicePtr->GetStreamCount();

		if (nStreamCount > 0)
		{
			m_objStreamPtr = m_objDevicePtr->OpenStream(0);
			m_objStreamFeatureControlPtr = m_objStreamPtr->GetFeatureControl();
			bIsStreamOpen = true;
		}
		else
		{
			throw std::exception("find no device stream!");
		}

		// �����û��ڴ��������֮�󣬸��ݵ�ǰ���绷�������������ͨ������ֵ��
		// �������������Ĳɼ�����,���÷����ο����´��롣
		GX_DEVICE_CLASS_LIST objDeviceClass = m_objDevicePtr->GetDeviceInfo().GetDeviceClass();
		if (GX_DEVICE_CLASS_GEV == objDeviceClass)
		{
			// �ж��豸�Ƿ�֧����ͨ�����ݰ�����
			if (true == m_objFeatureControlPtr->IsImplemented("GevSCPSPacketSize"))
			{
				// ��ȡ��ǰ���绷�������Ű���ֵ
				int nPacketSize = m_objStreamPtr->GetOptimalPacketSize();
				// �����Ű���ֵ����Ϊ��ǰ�豸����ͨ������ֵ
				m_objFeatureControlPtr->GetIntFeature("GevSCPSPacketSize")->SetValue(nPacketSize);
			}
		}

		// initiate device parameter.
		__InitParam();

		m_bIsOpen = true;

	}
	catch (CGalaxyException& e)
	{
		//�ж��豸���Ƿ��Ѵ�
		if (bIsStreamOpen)
		{
			m_objStreamPtr->Close();
		}

		//�ж��豸�Ƿ��Ѵ�
		if (bIsDeviceOpen)
		{
			m_objDevicePtr->Close();
		}

		//if (m_pBitmap != NULL)
		//{
		//	delete m_pBitmap;
		//	m_pBitmap = NULL;
		//}

		std::cout << e.what() << std::endl;
		return false; // ԭ����TRUE
	}
	catch (std::exception& e)
	{
		//�ж��豸���Ƿ��Ѵ�
		if (bIsStreamOpen)
		{
			m_objStreamPtr->Close();
		}

		//�ж��豸�Ƿ��Ѵ�
		if (bIsDeviceOpen)
		{
			m_objDevicePtr->Close();
		}

		//if (m_pBitmap != NULL)
		//{
		//	delete m_pBitmap;
		//	m_pBitmap = NULL;
		//}

		std::cout << e.what() << std::endl;
		return false; // ԭ����TRUE
	}
	std::cout << "camera successfully connected!\n";
	return true;
}

void GxCamTest::closeDevice()
{
	/*OnBnClickedBtnCloseDevice()*/

	//ʧȥ����
	//SetFocus();

	try
	{
		//�ж��Ƿ���ֹͣ�ɼ�
		if (m_bIsSnap)
		{
			//����ͣ������
			m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

			//�ر�����ɼ�
			m_objStreamPtr->StopGrab();

			//ע���ɼ��ص�
			m_objStreamPtr->UnregisterCaptureCallback();
		}
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	try
	{
		//�ر�������
		m_objStreamPtr->Close();

	}
	catch (CGalaxyException)
	{
		//do noting
	}
	try
	{
		//�ر��豸
		m_objDevicePtr->Close();
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	m_bIsOpen = false;
	m_bIsSnap = false;

	//���½���
	//__UpdateUI();
	//if (m_pBitmap != NULL)
	//{
	//	delete m_pBitmap;
	//	m_pBitmap = NULL;
	//}


	/*OnClose*/
	try
	{
		//�ͷ��豸��Դ
		IGXFactory::GetInstance().Uninit();
	}
	catch (CGalaxyException)
	{
		//do noting
	}
	catch (std::exception)
	{
		//do noting
	}

	if (m_pSampleCaptureEventHandle != NULL)
	{
		delete m_pSampleCaptureEventHandle;
		m_pSampleCaptureEventHandle = NULL;
	}

	//if (m_pBitmap != NULL)
	//{
	//	delete m_pBitmap;
	//	m_pBitmap = NULL;
	//}

}

void GxCamTest::startSnap()
{
	/*OnBnClickedBtnStartSnap()*/

	try
	{
		try
		{
			//����Buffer����ģʽ
			m_objStreamFeatureControlPtr->GetEnumFeature("StreamBufferHandlingMode")->SetValue("OldestFirst");
		}
		catch (...)
		{
		}

		//ע��ص�����
		m_objStreamPtr->RegisterCaptureCallback(m_pSampleCaptureEventHandle, this);

		//��������ͨ��
		m_objStreamPtr->StartGrab();

		//���Ϳ�������
		m_objFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();
		m_bIsSnap = true;

		//���½���
		//__UpdateUI();
	}
	catch (CGalaxyException& e)
	{
		std::cout << e.what() << std::endl;
		return;
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return;
	}
}

void GxCamTest::stopSnap()
{

	try
	{
		//����ͣ������
		m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

		//�ر�����ͨ��
		m_objStreamPtr->StopGrab();

		//ע���ɼ��ص�
		m_objStreamPtr->UnregisterCaptureCallback();
		m_bIsSnap = false;

		//���½���
		//__UpdateUI();
	}
	catch (CGalaxyException& e)
	{
		std::cout << e.what() << std::endl;
		return;
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return;
	}
}

//void GxCamTest::softTrigger()
//{
//	try
//	{
//		// send softTrigger command, and the callback function "m_pSampleCaptureEventHandle" is triggered.
//		m_objFeatureControlPtr->GetCommandFeature("TriggerSoftware")->Execute();
//	}
//	catch (CGalaxyException& e)
//	{
//		std::cout << e.what() << std::endl;
//		return;
//	}
//	catch (std::exception& e)
//	{
//		std::cout << e.what() << std::endl;
//		return;
//	}
//}

void GxCamTest::writeImgFlow(CImageDataPointer& objImageDataPointer)
{
	if (objImageDataPointer->GetStatus() == GX_FRAME_STATUS_SUCCESS) {
		cv::Mat imgTemp(objImageDataPointer->GetHeight(), objImageDataPointer->GetWidth(), CV_8UC1);
		void* pRaw8Buffer = NULL;
		pRaw8Buffer = objImageDataPointer->ConvertToRaw8(GX_BIT_0_7);
		memcpy(imgTemp.data, pRaw8Buffer, (objImageDataPointer->GetHeight()) * (objImageDataPointer->GetWidth()));
		//std::cout << imgTemp.size() << std::endl;

		{
			std::lock_guard<std::mutex> locker(img_mu);
			imgFlow = imgTemp.clone();
		}

		cv::resize(imgTemp, imgTemp, cv::Size(), 0.3, 0.3);
		cv::imshow("raw", imgTemp);
		cv::waitKey(1);

		//std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
		//std::chrono::duration<double, std::milli> duration_ms = end-startGx;
		//std::cout << '\t' << duration_ms.count() << "ms\n";
	}
	
	return;
}

void GxCamTest::SavePicture(CImageDataPointer& objImageDataPointer)
{
	try
	{
		//SYSTEMTIME   sysTime;                   ///< ϵͳʱ��
		std::string strFilePath = ""; // img save path.
		strFilePath = m_strSavePath.c_str();
		std::string strFileName = ""; // img save name.
		strFileName = strFilePath + "/" + std::to_string(m_iImgNum) + ".bmp";
			
		//����ͼ��ΪBMP
		//cv::Mat img;
		//img.create(objImageDataPointer->GetHeight(), objImageDataPointer->GetWidth(), CV_8UC1);
		//void* pRaw8Buffer = NULL;
		//pRaw8Buffer = objImageDataPointer->ConvertToRaw8(GX_BIT_0_7);
		//memcpy(img.data, pRaw8Buffer, (objImageDataPointer->GetHeight()) * (objImageDataPointer->GetWidth()));
		//cv::flip(img, img, 0);
		// 
		//cv::imwrite(strFileName, imgFlow);

		m_iImgNum += 1;
		m_bCheckSaveBmp = false;
	}
	catch (std::exception)
	{
		//���ڴ�ͼ�����̻߳ص���ʵ�ֵģ������ͼ�׳��쳣���ɼ��߳̽���ֹ��Ϊ�˱����߳���ֹ,��ͼ�����쳣��������
		return;

	}

}

cv::Mat GxCamTest::getImgFlow() {
	cv::Mat imgTemp;
	{
		std::lock_guard<std::mutex> locker(img_mu);
		imgTemp = imgFlow.clone();
	}
	return imgTemp;
}

void GxCamTest::__InitParam()
{

	bool bBalanceWhiteAutoRead = false;         ///< ��ƽ���Ƿ�ɶ�

	//���òɼ�ģʽΪ�����ɼ�ģʽ
	m_objFeatureControlPtr->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");

	//�Ƿ�֧�ִ���ģʽѡ��
	//m_bTriggerMode = m_objFeatureControlPtr->IsImplemented("TriggerMode");
	m_objFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("Off"); // set trigger mode: Off. In order to get continuous pictures.

	//�Ƿ�֧��Bayer��ʽ
	m_bColorFilter = m_objFeatureControlPtr->IsImplemented("PixelColorFilter");

	//�Ƿ�֧�ִ���Դѡ��
	m_bTriggerSource = m_objFeatureControlPtr->IsImplemented("TriggerSource");
	if (m_bTriggerSource)
	{
		// set trigger source: Software.
		m_objFeatureControlPtr->GetEnumFeature("TriggerSource")->SetValue("Software");
	}

	//�Ƿ�֧�ִ�������ѡ��
	m_bTriggerActive = m_objFeatureControlPtr->IsImplemented("TriggerActivation");
	if (m_bTriggerActive)
	{
		// set trigger active: FallingEdge.
		m_objFeatureControlPtr->GetEnumFeature("TriggerActivation")->SetValue("FallingEdge");
	}

	//�Ƿ�֧���Զ���ƽ��
	m_bBalanceWhiteAuto = m_objFeatureControlPtr->IsImplemented("BalanceWhiteAuto");

	//��ƽ���Ƿ�ɶ�
	bBalanceWhiteAutoRead = m_objFeatureControlPtr->IsReadable("BalanceWhiteAuto");

	//���֧���ҿɶ������ȡ�豸��ǰ��ƽ��ģʽ
	if (m_bBalanceWhiteAuto)
	{
		if (bBalanceWhiteAutoRead)
		{
			m_strBalanceWhiteAutoMode = m_objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")
				->GetValue();
		}
	}

	//�Ƿ�֧���Զ���ƽ��ͨ��ѡ��
	m_bBalanceWhiteRatioSelect = m_objFeatureControlPtr->IsImplemented("BalanceRatioSelector");

	//��ȡ�ع�ʱ�䡢���漰�Զ���ƽ��ϵ�������ֵ����Сֵ
	m_dShutterValueMax = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMax();
	m_dShutterValueMin = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMin();
	m_dGainValueMax = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMax();
	m_dGainValueMin = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMin();
	m_dBalanceWhiteRatioMax = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMax();
	m_dBalanceWhiteRatioMin = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMin();

	double dShutterValueOld = m_dEditShutterValue;
	try
	{
		//�ж�����ֵ�Ƿ����ع�ʱ�䷶Χ�ڣ����������������������ı߽�ֵ
		if (m_dEditShutterValue > m_dShutterValueMax)
		{
			m_dEditShutterValue = m_dShutterValueMax;
		}
		if (m_dEditShutterValue < m_dShutterValueMin)
		{
			m_dEditShutterValue = m_dShutterValueMin;
		}

		m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(m_dEditShutterValue);
	}
	catch (CGalaxyException& e)
	{
		m_dEditShutterValue = dShutterValueOld;
		std::cout << e.what() << std::endl;
	}
	catch (std::exception& e)
	{
		m_dEditShutterValue = dShutterValueOld;
		std::cout << e.what() << std::endl;
	}

	double dGainValueOld = m_dEditGainValue;
	try
	{
		//�ж�����ֵ�Ƿ����ع�ʱ�䷶Χ�ڣ����������������������ı߽�ֵ
		if (m_dEditGainValue > m_dGainValueMax)
		{
			m_dEditGainValue = m_dGainValueMax;
		}
		if (m_dEditGainValue < m_dGainValueMin)
		{
			m_dEditGainValue = m_dGainValueMin;
		}

		m_objFeatureControlPtr->GetFloatFeature("Gain")->SetValue(m_dEditGainValue);
	}
	catch (CGalaxyException& e)
	{
		m_dEditGainValue = dGainValueOld;
		std::cout << e.what() << std::endl;
	}
	catch (std::exception& e)
	{
		m_dEditGainValue = dGainValueOld;
		std::cout << e.what() << std::endl;
	}
}


