#include <ns3/log.h>
#include <ns3/double.h>
#include <ns3/boolean.h>
#include <cmath>
#include <ns3/simulator.h>

#include "antenna-model.h"
#include "directional-antenna-model.h"

namespace ns3{

NS_LOG_COMPONENT_DEFINE ("DirectionalAntennaModel");
NS_OBJECT_ENSURE_REGISTERED (DirectionalAntennaModel);

TypeId
DirectionalAntennaModel::GetTypeId ()
{
    static TypeId tid = TypeId ("ns3::DirectionalAntennaModel")
        .SetParent<AntennaModel> ()
        .SetGroupName("Antenna")
        .AddConstructor<DirectionalAntennaModel> ()
        .AddAttribute ("VMaxGain",
                       "The maximum attenuation (dB) of the antenna vertical radiation pattern.",
                       DoubleValue (30.0),
                       MakeDoubleAccessor (&DirectionalAntennaModel::m_vMaxGain),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("VBeamwidth",
                       "The vertical 3dB beamwidth (degrees)",
                       DoubleValue (25),//测试用代码。原值为10
                       MakeDoubleAccessor (&DirectionalAntennaModel::SetVerticalBeamwidth,
                                           &DirectionalAntennaModel::GetVerticalBeamwidth),
                       MakeDoubleChecker<double> (0, 180))
        .AddAttribute ("downTile",
                       "The angle (degrees) that expresses the downTile of the antenna",
                       DoubleValue (10),//测试用代码原值为15.7
                       MakeDoubleAccessor (&DirectionalAntennaModel::SetDownTile,
                                           &DirectionalAntennaModel::GetDownTile),
                       MakeDoubleChecker<double> (-360, 360))
        .AddAttribute ("HMaxGain",
                       "The maximum attenuation (dB) of the antenna horizontal radiation pattern.",
                       DoubleValue (25.0),
                       MakeDoubleAccessor (&DirectionalAntennaModel::m_hMaxGain),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("HBeamwidth",
                       "The horizontal 3dB beamwidth (degrees)",
                       DoubleValue (60),
                       MakeDoubleAccessor (&DirectionalAntennaModel::SetHorizontalBeamwidth,
                                           &DirectionalAntennaModel::GetHorizontalBeamwidth),
                       MakeDoubleChecker<double> (0, 180))
        .AddAttribute ("Orientation",
                       "The angle (degrees) that expresses the orientation of the antenna on the x-y plane relative to the x axis",
                       DoubleValue (0),
                       MakeDoubleAccessor (&DirectionalAntennaModel::SetOrientation,
                                           &DirectionalAntennaModel::GetOrientation),
                       MakeDoubleChecker<double> (-360, 360))
        .AddAttribute ("MaxGain",
                       "The maximum attenuation (dB) of the antenna radiation pattern.",
                       DoubleValue (0),
                       MakeDoubleAccessor (&DirectionalAntennaModel::m_maxAntennaGain),
                       MakeDoubleChecker<double> ())
	.AddAttribute ("DoubleAntennaEnabled",
                       "DoubleAntennaEnabled? [by default is true].",
                       BooleanValue (true),
                       MakeBooleanAccessor (&DirectionalAntennaModel::m_doubleAntenna),
                       MakeBooleanChecker ())
        
    ;
    return tid;
}

void
DirectionalAntennaModel::SetVerticalBeamwidth (double beamwidthDegrees)
{
    NS_LOG_FUNCTION (this << beamwidthDegrees);
    m_vBeamwidthRadians = DegreesToRadians (beamwidthDegrees);
}
double
DirectionalAntennaModel::GetVerticalBeamwidth () const
{
    return RadiansToDegrees (m_vBeamwidthRadians);
}

void
DirectionalAntennaModel::SetDownTile (double downTileDegrees)
{
    NS_LOG_FUNCTION (this << downTileDegrees);
    m_downTileRadians = DegreesToRadians (downTileDegrees);
}
double
DirectionalAntennaModel::GetDownTile () const
{
    return RadiansToDegrees (m_downTileRadians);
}

void
DirectionalAntennaModel::SetHorizontalBeamwidth (double beamwidthDegrees)
{
    NS_LOG_FUNCTION (this << beamwidthDegrees);
    m_hBeamwidthRadians = DegreesToRadians (beamwidthDegrees);
}
double
DirectionalAntennaModel::GetHorizontalBeamwidth () const
{
    return RadiansToDegrees (m_hBeamwidthRadians);
}

void
DirectionalAntennaModel::SetOrientation (double orientationDegrees)
{
    NS_LOG_FUNCTION (this << orientationDegrees);
    m_orientationRadians = DegreesToRadians (orientationDegrees);
}
double
DirectionalAntennaModel::GetOrientation () const
{
    return RadiansToDegrees (m_orientationRadians);
}

double
DirectionalAntennaModel::GetGainDb (Angles a)
{
    /*Check the parameters*/
  /*NS_LOG_LOGIC("--m_vMaxGain           "<<m_vMaxGain);
    NS_LOG_LOGIC("--m_vBeamwidthRadians  "<<m_vBeamwidthRadians);
    NS_LOG_LOGIC("--m_downTileRadians    "<<m_downTileRadians);
    NS_LOG_LOGIC("--m_hMaxGain           "<<m_hMaxGain);
    NS_LOG_LOGIC("--m_hBeamwidthRadians  "<<m_hBeamwidthRadians);
    NS_LOG_LOGIC("--m_orientationRadians "<<m_orientationRadians);
    NS_LOG_LOGIC("--m_maxAntennaGain     "<<m_maxAntennaGain);
  */
    NS_LOG_FUNCTION (this << a);
    double theta = a.theta - M_PI/2 - m_downTileRadians;
    while (theta <= - M_PI)
    {
        theta += M_PI+M_PI;
    }
    while (theta > M_PI)
    {
        theta -= M_PI+M_PI;
    }
    double vGain = -std::min(12*pow(theta/m_vBeamwidthRadians,2),m_vMaxGain);
    NS_LOG_LOGIC ("theta = " << theta);
    double phi = a.phi - m_orientationRadians;
    while (phi <= -M_PI)
    {
        phi += M_PI+M_PI;
    }
    while (phi > M_PI)
    {
        phi -= M_PI+M_PI;
    }
    if (fabs(phi)>m_hBeamwidthRadians)
    {
    	return -1e3;
    }
    double hGain = -std::min(12*pow(phi/m_hBeamwidthRadians,2),m_hMaxGain);
    NS_LOG_LOGIC ("phi = " << phi);
    //add up-antenna
    if (m_doubleAntenna)
    {
	//theta = a.theta - M_PI/2 + DegreesToRadians(57.7);//57.7
	//------------------------------------------------------测试用代码
    static double midtheta;
    static double DBeamwidthRadians = DegreesToRadians(10);
    static double DBeamK = 1.0;
    
    if((a.theta<DegreesToRadians(45))&&(a.theta>DegreesToRadians(30)))
    {
        midtheta = 37;
        DBeamwidthRadians = DegreesToRadians(7.5);
        ZeroPoint = false;
        DBeamK = 0.5;
    }
    
    if((a.theta<DegreesToRadians(65))&&(a.theta>DegreesToRadians(55)))
    {
        midtheta = 60;
        DBeamwidthRadians = DegreesToRadians(5);
        ZeroPoint = false;
        DBeamK = 0.5;
    }
    else if((a.theta<DegreesToRadians(80))&&(a.theta>DegreesToRadians(70)))
    {
        midtheta = 75;
        DBeamwidthRadians = DegreesToRadians(5);
        ZeroPoint = false;
        DBeamK = 0.5;
    }
    else if((a.theta<DegreesToRadians(105))&&(a.theta>DegreesToRadians(85)))
    {
        midtheta = 95;
        ZeroPoint = false;
    }
    else
    {
       ZeroPoint = true; 
       return -25;
    }
    theta = a.theta-DegreesToRadians(midtheta);//57.7
    //---------------------------------------------------------------
    while (theta <= - M_PI)
	{
	     theta += M_PI+M_PI;
	}
	while (theta > M_PI)
	{
	     theta -= M_PI+M_PI;
	}
    /*if (fabs(theta)>m_vBeamwidthRadians)//测试用代码
    {
        return -1e3;
    }*/
	double vGain2 = (-std::min((DBeamK*12*pow(theta/DBeamwidthRadians,2)+12*(DBeamK-1)),m_vMaxGain));
        vGain = fmax(vGain,vGain2);
   	
    }
    //end
    double gainDb = m_maxAntennaGain-std::min(-(vGain+hGain),m_hMaxGain);
    NS_LOG_LOGIC ("gain = " << gainDb);
    return gainDb;
}

} //namespace ns3
