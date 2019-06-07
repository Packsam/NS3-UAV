#ifndef DIRECTIONAL_ANTENNA_MODEL_H
#define DIRECTIONAL_ANTENNA_MODEL_H

#include <ns3/object.h>
#include <ns3/antenna-model.h>

namespace ns3{
class DirectionalAntennaModel : public AntennaModel
{
public:
    // inherited from Object
    static TypeId GetTypeId ();
    // inherited from AntennaModel
    virtual double GetGainDb (Angles a);
    // other function
    void   SetVerticalBeamwidth (double beamwidthDegrees);
    double GetVerticalBeamwidth () const;
    void   SetDownTile (double downTileDegrees);
    double GetDownTile () const;

    void   SetHorizontalBeamwidth (double beamwidthDegrees);
    double GetHorizontalBeamwidth () const;
    void   SetOrientation (double orientationDegrees);
    double GetOrientation () const;
private:
    double m_vMaxGain;
    double m_vBeamwidthRadians;
    double m_downTileRadians;

    double m_hMaxGain;
    double m_hBeamwidthRadians;
    double m_orientationRadians;

    double m_maxAntennaGain;

    bool   m_doubleAntenna;
};// Diretional Antenna Model
} // namespace ns3

#endif //DIRECTIONAL_ANTENNA_MODEL_H
