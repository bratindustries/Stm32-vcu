/*
 * Charge-interface request handling for CHAdeMO through a Nissan Leaf PDM.
 *
 * The NissanPDM charger class owns all CAN control. This class only reports
 * the PDM-derived DC fast-charge request to the VCU charge-mode state machine.
 */

#ifndef PDMCHADEMO_H
#define PDMCHADEMO_H

#include "chargerint.h"

class PdmChademoInterface : public Chargerint {
public:
  bool DCFCRequest(bool RunCh) override;
  bool ACRequest(bool RunCh) override { return RunCh; }
};

#endif // PDMCHADEMO_H