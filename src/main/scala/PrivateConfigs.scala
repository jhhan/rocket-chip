package rocketchip

import Chisel._
import uncore._
import rocket._
import dma._

class CopyAccelConfig extends ChiselConfig(
  (pname,site,here) => pname match {
    case RoCCMaxTaggedMemXacts => 2
    case DMAMaxXacts => site(TLMaxClientXacts)
    case DMADataBits => 32
    case BuildRoCC => Some(() => (Module(new CopyAccelerator, {
      case CoreName => "CopyAccelerator"
      case TLId => "Network"
    })))
    case NRoCCCSRs => 8
    case NTiles => 2
  }
)

class CopyAccelCPPConfig extends ChiselConfig(
  new CopyAccelConfig ++ new DefaultCPPConfig)

class CopyAccelFPGAConfig extends ChiselConfig(
  new CopyAccelConfig ++ new DefaultFPGAConfig)
