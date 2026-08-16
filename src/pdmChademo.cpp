/*
 * Charge-interface request handling for CHAdeMO through a Nissan Leaf PDM.
 */

#include "pdmChademo.h"
#include "NissanPDM.h"
#include "params.h"

bool PdmChademoInterface::DCFCRequest(bool RunCh) {
  // This interface is valid only when the Leaf PDM is the selected charger.
  if (Param::GetInt(Param::chargemodes) != ChargeModes::Leaf_PDM)
    return false;

  return RunCh && NissanPDM::DcfcRequestPresent();
}