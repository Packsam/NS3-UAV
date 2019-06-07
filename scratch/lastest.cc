#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/netanim-module.h"
#include <string>
#include <cmath>

using namespace ns3;
void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds()
            << context
            << " UE IMSI " << imsi
            << ": connected to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << Simulator::Now ().GetSeconds()
            << context
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds()
            << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds()
            << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << Simulator::Now ().GetSeconds()
            << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds()
            << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}
int
main()
{
/*******************************************************************************
模块日志
*******************************************************************************/
 //LogComponentEnable("A3RsrpHandoverAlgorithm", LOG_LEVEL_LOGIC);
 //LogComponentEnable("MultiModelSpectrumChannel", LOG_LEVEL_LOGIC);
 //LogComponentEnable("LteAnr", LOG_LEVEL_LOGIC);
/*******************************************************************************
参数列表
*******************************************************************************/
 double   simTime         = 10; //仿真时间
 uint16_t nThreeSectorEnb = 19;  //三个扇区的基站数
 uint16_t nMaxRow         = 3;   //每行最多放置几个基站
 double   enbDistance     = 200; //基站间距离
 double   enbXOffset      = 1.5*enbDistance;  //基站整体沿X轴偏置
 double   enbYOffset      = enbDistance/2;  //基站整体沿Y轴偏置
 double   enbTxPower      = 41.0;//基站发射功率
 uint16_t bandwidth       = 50; //50 for 10MHz
 uint16_t nUe             = 1;   //终端UE数量
 uint16_t nAerialUe       = 1;
 uint16_t nBearersPerUe   = 0;   //如果不想启动业务可以直接设置为0
 bool     useUdp          = true;
 bool     epcDl           = false;
 bool     epcUl           = true;
 bool     generateRem     = false;//是否生成Radio environment map
 std::string folderName   = "./Dir15";
/*******************************************************************************
开始设置
*******************************************************************************/
/*默认参数更改*/
 Config::SetDefault ("ns3::DirectionalAntennaModel::DoubleAntennaEnabled", BooleanValue (true));
 Config::SetDefault ("ns3::LteEnbMac::NumberOfRaPreambles", UintegerValue (32));
 //Config::SetDefault ("ns3::LteEnbRrc::RsrpFilterCoefficient", UintegerValue (4));

 Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (1)));
 Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (1000000));
 Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (10 * 1024));
 //Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));
 Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbTxPower));
 Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (80));
 Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (23.0));
 Config::SetDefault ("ns3::RrFfMacScheduler::HarqEnabled",BooleanValue(false));
 
 //Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));
 Config::SetDefault ("ns3::LteUePowerControl::ClosedLoop", BooleanValue (false));
 //Config::SetDefault ("ns3::LteUePowerControl::PoNominalPusch", IntegerValue (-80));
 //Config::SetDefault ("ns3::LteUePowerControl::Alpha", DoubleValue (0.9));
 Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (100));

 Config::SetDefault ("ns3::RadioBearerStatsCalculator::DlRlcOutputFilename", StringValue(folderName+"/DlRlcStats.txt"));
 Config::SetDefault ("ns3::RadioBearerStatsCalculator::UlRlcOutputFilename", StringValue(folderName+"/UlRlcStats.txt"));
 Config::SetDefault ("ns3::RadioBearerStatsCalculator::DlPdcpOutputFilename", StringValue(folderName+"/DlPdcpStats.txt"));
 Config::SetDefault ("ns3::RadioBearerStatsCalculator::UlPdcpOutputFilename", StringValue(folderName+"/UlPdcpStats.txt"));

 Config::SetDefault ("ns3::MacStatsCalculator::DlOutputFilename",  StringValue(folderName+"/DlMacStats.txt"));
 Config::SetDefault ("ns3::MacStatsCalculator::UlOutputFilename",  StringValue(folderName+"/UlMacStats.txt"));

 Config::SetDefault ("ns3::PhyStatsCalculator::DlRsrpSinrFilename",     StringValue(folderName+"/DlRsrpSinrStats.txt"));
 Config::SetDefault ("ns3::PhyStatsCalculator::UlSinrFilename",         StringValue(folderName+"/UlSinrStats.txt"));
 Config::SetDefault ("ns3::PhyStatsCalculator::UlInterferenceFilename", StringValue(folderName+"/UlInterferenceStats.txt"));
 Config::SetDefault ("ns3::PhyTxStatsCalculator::DlTxOutputFilename",   StringValue(folderName+"/DlTxPhyStats.txt"));
 Config::SetDefault ("ns3::PhyTxStatsCalculator::UlTxOutputFilename",   StringValue(folderName+"/UlTxPhyStats.txt"));
 Config::SetDefault ("ns3::PhyRxStatsCalculator::DlRxOutputFilename",   StringValue(folderName+"/DlRxPhyStats.txt"));
 Config::SetDefault ("ns3::PhyRxStatsCalculator::UlRxOutputFilename",   StringValue(folderName+"/UlRxPhyStats.txt"));
 
 Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
 lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
 lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::UavUmiAvPropagationLossModel"));
 //lteHelper->SetPathlossModelAttribute("Frequency",DoubleValue (2.0e9));
 //lteHelper->SetPathlossModelAttribute("BSAntennaHeight",DoubleValue (10.0));
 
 Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
 lteHelper->SetEpcHelper (epcHelper);

 lteHelper->SetHandoverAlgorithmType ("ns3::A2A4RsrqHandoverAlgorithm");
 lteHelper->SetHandoverAlgorithmAttribute ("ServingCellThreshold", UintegerValue (25));
 lteHelper->SetHandoverAlgorithmAttribute ("NeighbourCellOffset",  UintegerValue (0));


 
/*lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
 lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis", DoubleValue (0));//0
 lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger", TimeValue (MilliSeconds (20)));//50
*/
 Ptr<Node> pgw = epcHelper->GetPgwNode ();
// Create a single RemoteHost
 NodeContainer remoteHostContainer;
 remoteHostContainer.Create (1);
 Ptr<Node> remoteHost = remoteHostContainer.Get (0);
 InternetStackHelper internet;
 internet.Install (remoteHostContainer);
// Create the Internet
 PointToPointHelper p2ph;
 p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
 p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
 p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
 NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
 Ipv4AddressHelper ipv4h;
 ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
 Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
 Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
// Routing of the Internet Host (towards the LTE network)
 Ipv4StaticRoutingHelper ipv4RoutingHelper;
 Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
// interface 0 is localhost, 1 is the p2p device
 remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

 NodeContainer enbNodes;
 NodeContainer enbNodestmp;
 NodeContainer enbNodestmp2;
 NodeContainer ueNodes;
 NodeContainer aerialUeNodes;
 NodeContainer terrestrialUeNodes;
 NetDeviceContainer enbDevs;
 NetDeviceContainer ueDevs;

 enbNodes.Create(3*nThreeSectorEnb);
 for (int i=0;i<3*17;i++)
 {
     enbNodestmp.Add(enbNodes.Get (i));
 }
 
 for (int i=3*17;i<3*nThreeSectorEnb;i++)
 {
     enbNodestmp2.Add(enbNodes.Get (i));
 }
//基站移动模型
 MobilityHelper mobility;
 mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
 mobility.Install(enbNodes);
 Ptr<LteHexGridEnbTopologyHelper> lteHexGridEnbTopologyHelper = CreateObject<LteHexGridEnbTopologyHelper> ();
 lteHexGridEnbTopologyHelper->SetLteHelper (lteHelper);
 lteHexGridEnbTopologyHelper->SetAttribute ("InterSiteDistance", DoubleValue (enbDistance));
 lteHexGridEnbTopologyHelper->SetAttribute ("MinX", DoubleValue (enbXOffset));
 lteHexGridEnbTopologyHelper->SetAttribute ("MinY", DoubleValue (enbYOffset));
 lteHexGridEnbTopologyHelper->SetAttribute ("SiteHeight", DoubleValue (10.0));
 lteHexGridEnbTopologyHelper->SetAttribute ("GridWidth", UintegerValue (nMaxRow));
 lteHexGridEnbTopologyHelper->SetAttribute ("SectorOffset", DoubleValue (0.5));
/*Antenna Model 1*/
 lteHelper->SetEnbAntennaModelType ("ns3::DirectionalAntennaModel");

/*Antenna Model 2
  lteHelper->SetEnbAntennaModelType ("ns3::ParabolicAntennaModel");
  lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (65));//70
  lteHelper->SetEnbAntennaModelAttribute ("MaxAttenuation",     DoubleValue (30.0));//20
*/
/*Antenna Model 3
  lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
  lteHelper->SetEnbAntennaModelAttribute ("Gain", DoubleValue (0));
*/
//lteHelper->SetEnbDeviceAttribute ("DlEarfcn", UintegerValue (macroEnbDlEarfcn));
//lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (macroEnbDlEarfcn + 18000));
  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (bandwidth));
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (bandwidth));
  enbDevs = lteHexGridEnbTopologyHelper->SetPositionAndInstallEnbDevice (enbNodestmp);
  
  lteHexGridEnbTopologyHelper->SetAttribute ("InterSiteDistance", DoubleValue (4*enbDistance));
  lteHexGridEnbTopologyHelper->SetAttribute ("MinX", DoubleValue (enbDistance/2));
  lteHexGridEnbTopologyHelper->SetAttribute ("MinY", DoubleValue (enbDistance/2+2*enbDistance*sqrt(0.75)));
  lteHexGridEnbTopologyHelper->SetAttribute ("GridWidth", UintegerValue (2));
  enbDevs.Add(lteHexGridEnbTopologyHelper->SetPositionAndInstallEnbDevice (enbNodestmp2));
//UE移动模型
  MobilityHelper ueMobility;
//无人机移动模型
  if (nAerialUe!=0)
  {
      aerialUeNodes.Create(nAerialUe);
      ueMobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
                                       "X", StringValue ("ns3::UniformRandomVariable[Min=352|Max=382]"),
                                       "Y", StringValue ("ns3::UniformRandomVariable[Min=201|Max=231]"),
                                       "Z", StringValue ("ns3::ConstantRandomVariable[Constant=60]"));
      /*ueMobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
                                       "X", StringValue ("ns3::UniformRandomVariable[Min=300|Max=330]"),
                                       "Y", StringValue ("ns3::UniformRandomVariable[Min=100|Max=130]"),
                                       "Z", StringValue ("ns3::ConstantRandomVariable[Constant=120]"));*/
    /*  ueMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Mode", StringValue ("Time"),
                              "Time", StringValue ("0.1s"),
                              "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=20]"),
                              "Bounds", StringValue ("352|382|201|231"));
      ueMobility.Install (aerialUeNodes);
      ueNodes.Add(aerialUeNodes);*/
       ueMobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
       ueMobility.Install (aerialUeNodes);
      ueNodes.Add(aerialUeNodes);
             //ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (400, 0, 120));
      for(int i=0;i<nAerialUe;i++){
       ueNodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (14,14,0));
      }
  }
//地面用户移动模型
  if (nUe - nAerialUe>0)
  {
       terrestrialUeNodes.Create(nUe - nAerialUe);
       ueMobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
                                        "X", StringValue ("ns3::UniformRandomVariable[Min=385|Max=415]"),
                                        "Y", StringValue ("ns3::UniformRandomVariable[Min=143|Max=173]"),
                                        "Z", StringValue ("ns3::ConstantRandomVariable[Constant=1.5]"));
       ueMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                  "Mode", StringValue ("Time"),
                                  "Time", StringValue ("0.1s"),
                                  "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=40]"),
                                  "Bounds", StringValue ("370|430|120|180"));
       ueMobility.Install (terrestrialUeNodes);
       ueNodes.Add(terrestrialUeNodes);
      /* ueMobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
       ueMobility.Install (terrestrialUeNodes);
       ueNodes.Add(terrestrialUeNodes);
              ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,400, 1.5));
       ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (40, 0, 0));*/
   }

  ueDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIfaces;
  ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

  lteHelper->AttachToClosestEnb (ueDevs,enbDevs);
   //lteHelper->Attach (ueDevs, enbDevs.Get (34));
  // Install and start applications on UEs and remote host
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;

  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  if (useUdp)
  {
      startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
      startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));
  }
  else
  {
      // TCP needs to be started late enough so that all UEs are connected
      // otherwise TCP SYN packets will get lost
      startTimeSeconds->SetAttribute ("Min", DoubleValue (0.100));
      startTimeSeconds->SetAttribute ("Max", DoubleValue (0.110));
  }

  for (uint32_t u = 0; u < nUe; ++u)//numberOfUes
  {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < nBearersPerUe; ++b)//numBearersPerUe
      {
          ++dlPort;
          ++ulPort;

          ApplicationContainer clientApps;
          ApplicationContainer serverApps;

          if (useUdp)
          {
              if (epcDl)
              {
                  //NS_LOG_LOGIC ("installing UDP DL app for UE " << u);
                  UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
                  clientApps.Add (dlClientHelper.Install (remoteHost));
                  PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",InetSocketAddress (Ipv4Address::GetAny (), dlPort));
                  serverApps.Add (dlPacketSinkHelper.Install (ue));
              }
              if (epcUl)
              {
     	           //NS_LOG_LOGIC ("installing UDP UL app for UE " << u);
                   UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
                   clientApps.Add (ulClientHelper.Install (ue));
                   PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",InetSocketAddress (Ipv4Address::GetAny (), ulPort));
                   serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
              }
          }
          else // use TCP
          {
              if (epcDl)
              {
                    //NS_LOG_LOGIC ("installing TCP DL app for UE " << u);
                    BulkSendHelper dlClientHelper ("ns3::TcpSocketFactory",InetSocketAddress (ueIpIfaces.GetAddress (u), dlPort));
                    dlClientHelper.SetAttribute ("MaxBytes", UintegerValue (0));
                    clientApps.Add (dlClientHelper.Install (remoteHost));
                    PacketSinkHelper dlPacketSinkHelper ("ns3::TcpSocketFactory",InetSocketAddress (Ipv4Address::GetAny (), dlPort));
                    serverApps.Add (dlPacketSinkHelper.Install (ue));
              }
              if (epcUl)
              {
                     //NS_LOG_LOGIC ("installing TCP UL app for UE " << u);
                     BulkSendHelper ulClientHelper ("ns3::TcpSocketFactory",InetSocketAddress (remoteHostAddr, ulPort));
                     ulClientHelper.SetAttribute ("MaxBytes", UintegerValue (0));
                     clientApps.Add (ulClientHelper.Install (ue));
                     PacketSinkHelper ulPacketSinkHelper ("ns3::TcpSocketFactory",InetSocketAddress (Ipv4Address::GetAny (), ulPort));
                     serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
              }
           } // end if (useUdp)

           Ptr<EpcTft> tft = Create<EpcTft> ();
           if (epcDl)
           {
                 EpcTft::PacketFilter dlpf;
                 dlpf.localPortStart = dlPort;
                 dlpf.localPortEnd = dlPort;
                 tft->Add (dlpf);
           }
           if (epcUl)
           {
                 EpcTft::PacketFilter ulpf;
                 ulpf.remotePortStart = ulPort;
                 ulpf.remotePortEnd = ulPort;
                 tft->Add (ulpf);
           }
           if (epcDl || epcUl)
           {
                 EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
                 lteHelper->ActivateDedicatedEpsBearer (ueDevs.Get (u), bearer, tft);
           }
           Time startTime = Seconds (startTimeSeconds->GetValue ());
           serverApps.Start (startTime);
           clientApps.Start (startTime);
      } // end for b
  }


  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces();
  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  rlcStats->SetAttribute ("StartTime", TimeValue (Seconds (0)));
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (simTime)));
// Add X2 interface
  lteHelper->AddX2Interface (enbNodes);
  // connect custom trace sinks for RRC connection establishment and handover notification
//  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
//                   MakeCallback (&NotifyConnectionEstablishedEnb));
//  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
//                   MakeCallback (&NotifyConnectionEstablishedUe));
//  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
//                  MakeCallback (&NotifyHandoverStartEnb));
//  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
//                  MakeCallback (&NotifyHandoverStartUe));
//  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
//                   MakeCallback (&NotifyHandoverEndOkEnb));
//  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
//                   MakeCallback (&NotifyHandoverEndOkUe));
  Ptr<RadioEnvironmentMapHelper> remHelper = CreateObject<RadioEnvironmentMapHelper> ();
  if (generateRem)
  {
      remHelper->SetAttribute ("ChannelPath", StringValue ("/ChannelList/1"));
      remHelper->SetAttribute ("OutputFile", StringValue ("rem120.out"));
      remHelper->SetAttribute ("XMin", DoubleValue (0.0));
      remHelper->SetAttribute ("XMax", DoubleValue (1000.0));
      remHelper->SetAttribute ("YMin", DoubleValue (0.0));
      remHelper->SetAttribute ("YMax", DoubleValue (893.0));
      remHelper->SetAttribute ("Z", DoubleValue (120));
      remHelper->SetAttribute ("XRes", UintegerValue (1000));
      remHelper->SetAttribute ("YRes", UintegerValue (893));
      //remHelper->SetAttribute ("MaxPointsPerIteration", UintegerValue (20000000));
      remHelper->Install ();
  }
 
  Simulator::Stop(Seconds(simTime));
  //AnimationInterface anim("scratch/lastest.xml");
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
