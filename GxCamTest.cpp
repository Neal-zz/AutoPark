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
	, m_dEditShutterValue(20000) // 曝光
	, m_dEditGainValue(0) // 增益
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

	imgFlow.create(2048, 2448, CV_8UC1); // img width; height; 8 bit, unsigned, 1 channel.
}

BOOL GxCamTest::openDevice()
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
		return FALSE;

	}
	catch (std::exception& e)
	{
		if (m_pSampleCaptureEventHandle != NULL)
		{
			delete m_pSampleCaptureEventHandle;
			m_pSampleCaptureEventHandle = NULL;
		}

		std::cout << e.what() << std::endl;
		return FALSE;

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
			return FALSE;
		}

		// open device.
		m_objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);
		bIsDeviceOpen = true;
		m_objFeatureControlPtr = m_objDevicePtr->GetRemoteFeatureControl();

		////判断画图对象是否为空
		//if (m_pBitmap != NULL)
		//{
		//	delete m_pBitmap;
		//	m_pBitmap = NULL;
		//}
		//
		//为画图对象分配内存
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

		// 建议用户在打开网络相机之后，根据当前网络环境设置相机的流通道包长值，
		// 以提高网络相机的采集性能,设置方法参考以下代码。
		GX_DEVICE_CLASS_LIST objDeviceClass = m_objDevicePtr->GetDeviceInfo().GetDeviceClass();
		if (GX_DEVICE_CLASS_GEV == objDeviceClass)
		{
			// 判断设备是否支持流通道数据包功能
			if (true == m_objFeatureControlPtr->IsImplemented("GevSCPSPacketSize"))
			{
				// 获取当前网络环境的最优包长值
				int nPacketSize = m_objStreamPtr->GetOptimalPacketSize();
				// 将最优包长值设置为当前设备的流通道包长值
				m_objFeatureControlPtr->GetIntFeature("GevSCPSPacketSize")->SetValue(nPacketSize);
			}
		}

		// initiate device parameter.
		__InitParam();

		m_bIsOpen = true;

	}
	catch (CGalaxyException& e)
	{
		//判断设备流是否已打开
		if (bIsStreamOpen)
		{
			m_objStreamPtr->Close();
		}

		//判断设备是否已打开
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
		return TRUE;
	}
	catch (std::exception& e)
	{
		//判断设备流是否已打开
		if (bIsStreamOpen)
		{
			m_objStreamPtr->Close();
		}

		//判断设备是否已打开
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
		return TRUE;
	}
}

void GxCamTest::closeDevice()
{
	/*OnBnClickedBtnCloseDevice()*/

	//失去焦点
	//SetFocus();

	try
	{
		//判断是否已停止采集
		if (m_bIsSnap)
		{
			//发送停采命令
			m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

			//关闭流层采集
			m_objStreamPtr->StopGrab();

			//注销采集回调
			m_objStreamPtr->UnregisterCaptureCallback();
		}
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	try
	{
		//关闭流对象
		m_objStreamPtr->Close();

	}
	catch (CGalaxyException)
	{
		//do noting
	}
	try
	{
		//关闭设备
		m_objDevicePtr->Close();
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	m_bIsOpen = false;
	m_bIsSnap = false;

	//更新界面
	//__UpdateUI();
	//if (m_pBitmap != NULL)
	//{
	//	delete m_pBitmap;
	//	m_pBitmap = NULL;
	//}


	/*OnClose*/
	try
	{
		//释放设备资源
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
			//设置Buffer处理模式
			m_objStreamFeatureControlPtr->GetEnumFeature("StreamBufferHandlingMode")->SetValue("OldestFirst");
		}
		catch (...)
		{
		}

		//注册回调函数
		m_objStreamPtr->RegisterCaptureCallback(m_pSampleCaptureEventHandle, this);

		//开启流层通道
		m_objStreamPtr->StartGrab();

		//发送开采命令
		m_objFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();
		m_bIsSnap = true;

		//更新界面
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
		//发送停采命令
		m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

		//关闭流层通道
		m_objStreamPtr->StopGrab();

		//注销采集回调
		m_objStreamPtr->UnregisterCaptureCallback();
		m_bIsSnap = false;

		//更新界面
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
	void* pRaw8Buffer = NULL;
	pRaw8Buffer = objImageDataPointer->ConvertToRaw8(GX_BIT_0_7);
	memcpy(imgFlow.data, pRaw8Buffer, (objImageDataPointer->GetHeight()) * (objImageDataPointer->GetWidth()));
	cv::flip(imgFlow, imgFlow, 0);

	//cv::resize(imgFlow, imgFlow, cv::Size(), 0.3, 0.3);
	//cv::imshow("test", imgFlow);
	//cv::waitKey(1);
	return;
}

void GxCamTest::setCheckSaveBmp()
{
	m_bCheckSaveBmp = true;
}

void GxCamTest::SavePicture(CImageDataPointer& objImageDataPointer)
{
	try
	{
		//SYSTEMTIME   sysTime;                   ///< 系统时间
		std::string strFilePath = ""; // img save path.
		strFilePath = m_strSavePath.c_str();
		std::string strFileName = ""; // img save name.
		strFileName = strFilePath + "/" + std::to_string(m_iImgNum) + ".bmp";
			

		//保存图像为BMP
		cv::Mat img;
		img.create(objImageDataPointer->GetHeight(), objImageDataPointer->GetWidth(), CV_8UC1); // 8 bit, unsigned, 3 channel.
		void* pRaw8Buffer = NULL;
		pRaw8Buffer = objImageDataPointer->ConvertToRaw8(GX_BIT_0_7);
		memcpy(img.data, pRaw8Buffer, (objImageDataPointer->GetHeight()) * (objImageDataPointer->GetWidth()));
		cv::flip(img, img, 0);
		cv::imwrite(strFileName, img);

		m_iImgNum += 1;
		m_bCheckSaveBmp = false;
	}
	catch (std::exception)
	{
		//由于存图是在线程回调中实现的，如果存图抛出异常。采集线程将终止，为了避免线程终止,存图将对异常不做处理
		return;

	}

}

void GxCamTest::__InitParam()
{

	bool bBalanceWhiteAutoRead = false;         ///< 白平衡是否可读

	//设置采集模式为连续采集模式
	m_objFeatureControlPtr->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");

	//是否支持触发模式选择
	//m_bTriggerMode = m_objFeatureControlPtr->IsImplemented("TriggerMode");
	m_objFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("Off"); // set trigger mode: Off. In order to get continuous pictures.

	//是否支持Bayer格式
	m_bColorFilter = m_objFeatureControlPtr->IsImplemented("PixelColorFilter");

	//是否支持触发源选择
	m_bTriggerSource = m_objFeatureControlPtr->IsImplemented("TriggerSource");
	if (m_bTriggerSource)
	{
		// set trigger source: Software.
		m_objFeatureControlPtr->GetEnumFeature("TriggerSource")->SetValue("Software");
	}

	//是否支持触发极性选择
	m_bTriggerActive = m_objFeatureControlPtr->IsImplemented("TriggerActivation");
	if (m_bTriggerActive)
	{
		// set trigger active: FallingEdge.
		m_objFeatureControlPtr->GetEnumFeature("TriggerActivation")->SetValue("FallingEdge");
	}

	//是否支持自动白平衡
	m_bBalanceWhiteAuto = m_objFeatureControlPtr->IsImplemented("BalanceWhiteAuto");

	//白平衡是否可读
	bBalanceWhiteAutoRead = m_objFeatureControlPtr->IsReadable("BalanceWhiteAuto");

	//如果支持且可读，则获取设备当前白平衡模式
	if (m_bBalanceWhiteAuto)
	{
		if (bBalanceWhiteAutoRead)
		{
			m_strBalanceWhiteAutoMode = m_objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")
				->GetValue();
		}
	}

	//是否支持自动白平衡通道选择
	m_bBalanceWhiteRatioSelect = m_objFeatureControlPtr->IsImplemented("BalanceRatioSelector");

	//获取曝光时间、增益及自动白平衡系数的最大值和最小值
	m_dShutterValueMax = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMax();
	m_dShutterValueMin = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMin();
	m_dGainValueMax = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMax();
	m_dGainValueMin = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMin();
	m_dBalanceWhiteRatioMax = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMax();
	m_dBalanceWhiteRatioMin = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMin();
}
