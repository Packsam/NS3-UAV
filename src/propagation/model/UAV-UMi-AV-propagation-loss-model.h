#ifndef UAV_UMi_AV_PROPAGATION_LOSS_MODEL_H
#define UAV_UMi_AV_PROPAGATION_LOSS_MODEL_H

#include <ns3/propagation-loss-model.h>

namespace ns3 {

class UavUmiAvPropagationLossModel : public PropagationLossModel
{

public:
  /*
    Usage:
    lteHelper->SetPathlossModelType("UavUmiAvPropagationLossModel");
    lteHelper->SetPathlossModelAttribute("Frequency",DoubleValue (2.0e9));
    lteHelper->SetPathlossModelAttribute("BSAntennaHeight",DoubleValue (10.0));
  */
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  UavUmiAvPropagationLossModel ();
  virtual ~UavUmiAvPropagationLossModel ();

  void   SetFrequency(double fc_HZ);
  void   SetBSAntennaHeight (double height);
  double GetLosLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b, double hut) const;
  double GetNlosLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b, double hut) const;
  double GetLosProbability (Ptr<MobilityModel> a, Ptr<MobilityModel> b, double hut) const;
private:
  /**
   * \brief Copy constructor
   * [4]
   * Defined and unimplemented to avoid misuse
   */
  UavUmiAvPropagationLossModel (const UavUmiAvPropagationLossModel &);
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns
   */
  UavUmiAvPropagationLossModel & operator = (const UavUmiAvPropagationLossModel &);

  virtual double DoCalcRxPower (double txPowerDbm,
                                Ptr<MobilityModel> a,
                                Ptr<MobilityModel> b) const;
  virtual int64_t DoAssignStreams (int64_t stream);

  Ptr<UniformRandomVariable>  m_uniformRandomVariable;
  Ptr<NormalRandomVariable> m_normalRandomVariable;
  double m_frequency;
  double m_BSAntennaHeight;
  double m_BSHeight;

};

} // namespace ns3


#endif // OKUMURA_HATA_PROPAGATION_LOSS_MODEL_H
