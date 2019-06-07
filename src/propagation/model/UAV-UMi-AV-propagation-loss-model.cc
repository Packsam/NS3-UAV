
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/mobility-model.h"
#include <cmath>

#include "UAV-UMi-AV-propagation-loss-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("UavUmiAvPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (UavUmiAvPropagationLossModel);


TypeId
UavUmiAvPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UavUmiAvPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<UavUmiAvPropagationLossModel> ()
    .AddAttribute ("UniformRv",
               "Access to the underlying UniformRandomVariable",
               StringValue ("ns3::UniformRandomVariable"),
               MakePointerAccessor (&UavUmiAvPropagationLossModel::m_uniformRandomVariable),
               MakePointerChecker<UniformRandomVariable> ())
    .AddAttribute ("NormalRv",
                "Access to the underlying LogNormalRandomVariable",
                StringValue ("ns3::NormalRandomVariable"),
                MakePointerAccessor (&UavUmiAvPropagationLossModel::m_normalRandomVariable),
                MakePointerChecker<NormalRandomVariable> ())
    .AddAttribute ("Frequency",
               "The Frequency  (default is 2 GHz).",
               DoubleValue (2.0e9),
               MakeDoubleAccessor (&UavUmiAvPropagationLossModel::m_frequency),
               MakeDoubleChecker<double> ())
    .AddAttribute ("BSAntennaHeight",
                "BS Antenna Height (default is 10m).",
                DoubleValue (10.0),
                MakeDoubleAccessor (&UavUmiAvPropagationLossModel::m_BSAntennaHeight),
                MakeDoubleChecker<double> ())
    .AddAttribute ("BSHeight",
                "BS Height (default is 10m).",
                DoubleValue (10.0),
                MakeDoubleAccessor (&UavUmiAvPropagationLossModel::m_BSHeight),
                MakeDoubleChecker<double> ())
    ;
  return tid;
}

UavUmiAvPropagationLossModel::UavUmiAvPropagationLossModel ()
  : PropagationLossModel ()
{
}

UavUmiAvPropagationLossModel::~UavUmiAvPropagationLossModel ()
{
}

void
UavUmiAvPropagationLossModel::SetFrequency(double fc_HZ)
{
    m_frequency = fc_HZ;
}

void
UavUmiAvPropagationLossModel::SetBSAntennaHeight (double height)
{   //BSAntennaHeight must be 10m in this scenario.
    m_BSAntennaHeight = height;
}

double
UavUmiAvPropagationLossModel::GetLosLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b, double hut) const
{   //TODO:中断距离dBP在38.901表7.4.1-1中解释比较复杂,下面简单计算，可能不准确
    double dBP = 4*(m_BSAntennaHeight-1)*(hut-1)*m_frequency/(3.0*pow(10,8));
    double d2D = sqrt(pow((a->GetPosition ().x - b->GetPosition ().x),2) + pow((a->GetPosition ().y - b->GetPosition ().y),2));
    double d3D = a->GetDistanceFrom (b);
    double pathLoss = 0.0;
    double pathLossFreeSpace = 0.0;
    //Change:delete condition:10<=d2D
    if (hut<=22.5 && d2D<=dBP && m_BSAntennaHeight == 10){
        //if 1.5m<=hut<=22.5
        //and 10m<=d2D<=dBP
        //and h_BS=10m
        pathLoss = 32.4 + 21*log10(d3D) + 20*log10(m_frequency/(1.0e9));
        pathLoss += m_normalRandomVariable->GetValue(0,4);
    }else if (hut<=22.5 && dBP<=d2D && d2D<=5.0e3 && m_BSAntennaHeight == 10){
        //if 1.5m<=hut<=22.5
        //and dBP<=d2D<=5km
        //and h_BS=10m
        pathLoss = 32.4 + 40*log10(d3D) + 20*log10(m_frequency/(1.0e9)) - 9.5*log10(pow(dBP,2)+pow(m_BSAntennaHeight-hut,2));
        pathLoss += m_normalRandomVariable->GetValue(0,4);
    }else if (hut<=300 && d2D<=4.0e3){
        //if 22.5m<hut<=300m
        //and d2D<=4km
        pathLossFreeSpace = 32.45 + 20*log10(m_frequency/1.0e6) + 20*log10(d3D/1.0e3);
        pathLoss = 30.9 + (22.25-0.5*log10(hut))*log10(d3D) + 20*log10(m_frequency/(1.0e9));
        pathLoss = fmax(pathLossFreeSpace, pathLoss);
        pathLoss += m_normalRandomVariable->GetValue(0, fmax(5*exp(-0.01*hut), 2));
    }else{
        pathLoss = 1.0e12;
    }
    NS_LOG_LOGIC ("PathLoss (LOS) = " << pathLoss );
    return pathLoss;
}

double
UavUmiAvPropagationLossModel::GetNlosLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b, double hut) const
{
    double d2D = sqrt(pow((a->GetPosition ().x - b->GetPosition ().x),2) + pow((a->GetPosition ().y - b->GetPosition ().y),2));
    double d3D = a->GetDistanceFrom (b);
    double pathLossLOS = 0.0;
    double pathLoss = 0.0;
    //Change:delete condition:10<=d2D
    if (hut<=22.5 && d2D<=5.0e3 && m_BSAntennaHeight == 10){
        //if 1.5m<=hut<=22.5m
        //and 10m<=d2D<=5km
        //and h_BS=10m
        pathLossLOS = GetLosLoss(a, b, hut);
        pathLoss = 35.3*log10(d3D) + 22.4 + 21.3*log10(m_frequency/(1.0e9)) - 0.3*(hut-1.5);
        pathLoss = fmax(pathLossLOS, pathLoss);
        pathLoss += m_normalRandomVariable->GetValue(0,7.82);
    }else if (hut<=300 && d2D<=4.0e3){
        //if 22.5m<hut<=300m
        //and d2D<=4km
        pathLossLOS = GetLosLoss(a, b, hut);
        pathLoss = 32.4 + (43.2-7.6*log10(hut))*log10(d3D) + 20*log10(m_frequency/(1.0e9));
        pathLoss = fmax(pathLossLOS, pathLoss);
        pathLoss += m_normalRandomVariable->GetValue(0,8);
    }else{
        pathLoss = 1.0e12;
    }
    NS_LOG_LOGIC ("PathLoss (NLOS) = " << pathLoss );
    return pathLoss;
}

double
UavUmiAvPropagationLossModel::GetLosProbability (Ptr<MobilityModel> a, Ptr<MobilityModel> b, double hut) const
{
    double d2D = sqrt(pow((a->GetPosition ().x - b->GetPosition ().x),2) + pow((a->GetPosition ().y - b->GetPosition ().y),2));
    double p   = 0.0;
    double d1  = fmax(294.05 * log10(hut) - 432.94, 18);
    double p1  = 233.98 * log10(hut) - 0.95;

    if (hut<=22.5 && d2D<=18){
        //if 1.5m<=hut<=22.5m
        //and d2D<=18
        p = 1.0;
    }else if (hut<=22.5 && 18<d2D){
        //if 1.5m<=hut<=22.5m
        //and 18<d2D
        p = 18/d2D + (1-18/d2D)*exp(-d2D/36);
    }else if (hut<=300 && d2D<=d1){
        //if 22.5m<hut<=300m
        //and d2D<=d1
        p = 1.0;
    }else if (hut<=300 && d1<d2D){
        //if 22.5m<hut<=300m
        //and d1<d2D
        p = d1/d2D + (1-d1/d2D)*exp(-d2D/p1);
    }else{
        //TODO:不可能运行到这里
        NS_ASSERT_MSG (p!=0.0, "In UAV_UMi_AV, function(GetLosProbability) error");
    }
    return p;
}

double
UavUmiAvPropagationLossModel::DoCalcRxPower (double txPowerDbm,
						Ptr<MobilityModel> a,
						Ptr<MobilityModel> b) const
{
  double hut = (a->GetPosition ().z == m_BSHeight ? b->GetPosition ().z : a->GetPosition ().z);
  NS_LOG_LOGIC ("UE Height = " << hut );
  //TODO:如果出现BUG，请首先检测节点的位置值
  NS_ASSERT_MSG ((hut >= 1.5)&& (hut <= 300), "ue's height must be greater then 1.5m and lower then 300m ");
  double LOSProbability = GetLosProbability(a, b, hut);
  double randomValue = m_uniformRandomVariable->GetValue(0,1);
  if (randomValue <= LOSProbability)
    //如果骰子摇的值落在LOS概率内，则选择LOS损耗
    {
        Nlos = false;
        return (txPowerDbm - GetLosLoss (a, b, hut));
    }
  else
    {
    //如果骰子摇的值没落在LOS概率内，则选择NLOS损耗
    //std::cout<<"NLOS"<<std::endl;
    Nlos = true;
    return (txPowerDbm - GetNlosLoss (a, b, hut));
    }
}

int64_t
UavUmiAvPropagationLossModel::DoAssignStreams (int64_t stream)
{
  //TODO:添加随机变量流
  return 0;
}


} // namespace ns3
