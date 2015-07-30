// See LICENSE for license details.

package rocketchip

import Chisel._
import uncore._

/** RocketChipNetworks combine a TileLink protocol with a particular physical
  * network implementation and chip layout.
  *
  * Specifically, they provide mappings between ClientTileLinkIO/ 
  * ManagerTileLinkIO channels and LogicalNetwork ports (i.e. generic
  * TileLinkIO with networking headers). Channels coming into the network have
  * appropriate networking headers appended and outgoing channels have their
  * headers stripped.
  *
  * @constructor base class constructor for Rocket NoC
  * @param addrToManagerId a mapping from a physical address to the network
  *        id of a coherence manager
  * @param sharerToClientId a mapping from the id of a particular coherent
  *        client (as determined by e.g. the directory) and the network id
  *        of that client
  * @param clientDepths the depths of the queue that should be used to buffer
  *        each channel on the client side of the network
  * @param managerDepths the depths of the queue that should be used to buffer
  *        each channel on the manager side of the network
  */
abstract class RocketChipNetwork(
    addrToManagerId: UInt => UInt,
    sharerToClientId: UInt => UInt,
    clientDepths: TileLinkDepths, 
    managerDepths: TileLinkDepths) extends TLModule {
  val nClients = params(TLNClients)
  val nManagers = params(TLNManagers)
  val io = new Bundle {
    val clients = Vec.fill(nClients){new ClientTileLinkIO}.flip
    val managers = Vec.fill(nManagers){new ManagerTileLinkIO}.flip
  }

  val clients = io.clients.zipWithIndex.map { 
    case (c, i) => {
      val p = Module(new ClientTileLinkNetworkPort(i, addrToManagerId))
      val q = Module(new TileLinkEnqueuer(clientDepths))
      p.io.client <> c
      q.io.client <> p.io.network
      q.io.manager 
    }
  }

  val managers = io.managers.zipWithIndex.map {
    case (m, i) => {
      val p = Module(new ManagerTileLinkNetworkPort(i, sharerToClientId))
      val q = Module(new TileLinkEnqueuer(managerDepths))
      m <> p.io.manager
      p.io.network <> q.io.manager
      q.io.client
    }
  }
}

/** A simple arbiter for each channel that also deals with header-based routing.
  * Assumes a single manager agent. */
class RocketChipTileLinkArbiter(
    sharerToClientId: UInt => UInt = (u: UInt) => u,
    clientDepths: TileLinkDepths = TileLinkDepths(0,0,0,0,0), 
    managerDepths: TileLinkDepths = TileLinkDepths(0,0,0,0,0))
      extends RocketChipNetwork(u => UInt(0), sharerToClientId, clientDepths, managerDepths)
        with TileLinkArbiterLike
        with PassesId {
  val arbN = nClients
  require(nManagers == 1)
  if(arbN > 1) {
    hookupClientSource(clients.map(_.acquire), managers.head.acquire)
    hookupClientSource(clients.map(_.release), managers.head.release)
    hookupFinish(clients.map(_.finish), managers.head.finish)
    hookupManagerSourceWithHeader(clients.map(_.probe), managers.head.probe)
    hookupManagerSourceWithHeader(clients.map(_.grant), managers.head.grant)
  } else {
    managers.head <> clients.head
  }
}

/** Provides a separate physical crossbar for each channel. Assumes multiple manager
  * agents. Managers are assigned to higher physical network port ids than
  * clients, and translations between logical network id and physical crossbar
  * port id are done automatically.
  */
class RocketChipTileLinkCrossbar(
    addrToManagerId: UInt => UInt = u => UInt(0),
    sharerToClientId: UInt => UInt = u => u,
    clientDepths: TileLinkDepths = TileLinkDepths(0,0,0,0,0), 
    managerDepths: TileLinkDepths = TileLinkDepths(0,0,0,0,0))
      extends RocketChipNetwork(addrToManagerId, sharerToClientId, clientDepths, managerDepths) {
  val n = params(LNEndpoints)
  val count = params(TLDataBeats)
  // Actually instantiate the particular networks required for TileLink
  val acqNet = Module(new BasicCrossbar(n, new Acquire, count, Some((a: PhysicalNetworkIO[Acquire]) => a.payload.hasMultibeatData())))
  val relNet = Module(new BasicCrossbar(n, new Release, count, Some((r: PhysicalNetworkIO[Release]) => r.payload.hasMultibeatData())))
  val prbNet = Module(new BasicCrossbar(n, new Probe))
  val gntNet = Module(new BasicCrossbar(n, new Grant, count, Some((g: PhysicalNetworkIO[Grant]) => g.payload.hasMultibeatData())))
  val ackNet = Module(new BasicCrossbar(n, new Finish))

  // Aliases for the various network IO bundle types
  type PNIO[T <: Data] = DecoupledIO[PhysicalNetworkIO[T]]
  type LNIO[T <: Data] = DecoupledIO[LogicalNetworkIO[T]]
  type FromCrossbar[T <: Data] = PNIO[T] => LNIO[T]
  type ToCrossbar[T <: Data] = LNIO[T] => PNIO[T]

  // Shims for converting between logical network IOs and physical network IOs
  def crossbarToManagerShim[T <: Data](in: PNIO[T]): LNIO[T] = {
    val out = DefaultFromPhysicalShim(in)
    out.bits.header.src := in.bits.header.src - UInt(nManagers)
    out
  }
  def crossbarToClientShim[T <: Data](in: PNIO[T]): LNIO[T] = {
    val out = DefaultFromPhysicalShim(in)
    out.bits.header.dst := in.bits.header.dst - UInt(nManagers)
    out
  }
  def managerToCrossbarShim[T <: Data](in: LNIO[T]): PNIO[T] = {
    val out = DefaultToPhysicalShim(n, in)
    out.bits.header.dst := in.bits.header.dst + UInt(nManagers)
    out
  }
  def clientToCrossbarShim[T <: Data](in: LNIO[T]): PNIO[T] = {
    val out = DefaultToPhysicalShim(n, in)
    out.bits.header.src := in.bits.header.src + UInt(nManagers)
    out
  }

  // Make an individual connection between virtual and physical ports using
  // a particular shim. Also pin the unused Decoupled control signal low.
  def doDecoupledInputHookup[T <: Data](phys_in: PNIO[T], phys_out: PNIO[T], log_io: LNIO[T], shim: ToCrossbar[T]) = {
    val s = shim(log_io)
    phys_in.valid := s.valid
    phys_in.bits := s.bits
    s.ready := phys_in.ready
    phys_out.ready := Bool(false)
  }

  def doDecoupledOutputHookup[T <: Data](phys_in: PNIO[T], phys_out: PNIO[T], log_io: LNIO[T], shim: FromCrossbar[T]) = {
    val s = shim(phys_out)
    log_io.valid := s.valid
    log_io.bits := s.bits
    s.ready := log_io.ready
    phys_in.valid := Bool(false)
  }

  //Hookup all instances of a particular subbundle of TileLink
  def doDecoupledHookups[T <: Data](physIO: BasicCrossbarIO[T], getLogIO: TileLinkIO => LNIO[T]) = {
    physIO.in.head.bits.payload match {
      case c: ClientToManagerChannel => {
        managers.zipWithIndex.map { case (i, id) => 
          doDecoupledOutputHookup(physIO.in(id), physIO.out(id), getLogIO(i), crossbarToManagerShim[T])
        }
        clients.zipWithIndex.map { case (i, id) =>
          doDecoupledInputHookup(physIO.in(id+nManagers), physIO.out(id+nManagers), getLogIO(i), clientToCrossbarShim[T])
        }
      }
      case m: ManagerToClientChannel => {
        managers.zipWithIndex.map { case (i, id) =>
          doDecoupledInputHookup(physIO.in(id), physIO.out(id), getLogIO(i), managerToCrossbarShim[T])
        }
        clients.zipWithIndex.map { case (i, id) =>
          doDecoupledOutputHookup(physIO.in(id+nManagers), physIO.out(id+nManagers), getLogIO(i), crossbarToClientShim[T])
        }
      }
    }
  }

  doDecoupledHookups(acqNet.io, (tl: TileLinkIO) => tl.acquire)
  doDecoupledHookups(relNet.io, (tl: TileLinkIO) => tl.release)
  doDecoupledHookups(prbNet.io, (tl: TileLinkIO) => tl.probe)
  doDecoupledHookups(gntNet.io, (tl: TileLinkIO) => tl.grant)
  doDecoupledHookups(ackNet.io, (tl: TileLinkIO) => tl.finish)
}

class IndexPair(n: Int) extends Bundle {
  val fe_idx = UInt(width = log2Up(n))
  val be_idx = UInt(width = log2Up(n))
  override def cloneType = new IndexPair(n).asInstanceOf[this.type]
}

class RocketChipSwitcher(n: Int) extends Module {
  val io = new Bundle {
    val in_req = Vec.fill(n) { Decoupled(new RemoteAddress).flip }
    val out_req = Vec.fill(n) { Decoupled(new RemoteAddress) }
    val cur_addr = Vec.fill(n) { new RemoteAddress().asInput }
    val finish = Vec.fill(n) { Bool(INPUT) }
  }

  val nroutes = n * (n - 1)

  val ctr = Counter(nroutes)
  ctr.inc()

  val idx_table = Vec.fill(nroutes) { new IndexPair(n) }

  var fe_ind = 0
  var be_ind = 1

  for (i <- 0 until nroutes) {
    idx_table(i).fe_idx := UInt(fe_ind)
    idx_table(i).be_idx := UInt(be_ind)

    if (be_ind == n - 1) {
      fe_ind += 1
      be_ind = 0
    } else if (be_ind == fe_ind - 1) {
      be_ind += 2
    } else {
      be_ind += 1
    }
  }

  val fe_idx = idx_table(ctr.value).fe_idx
  val fe_req = io.in_req(fe_idx)

  val be_idx = idx_table(ctr.value).be_idx
  val be_req = io.out_req(be_idx)
  val be_addr = io.cur_addr(be_idx)

  val be_busy = Vec.fill(n) { Reg(init = Bool(false)) }

  val fe_be_map = Vec.fill(n) { Reg(UInt(width = log2Up(n))) }

  for (i <- 0 until n) {
    io.out_req(i).valid := fe_req.valid && be_idx === UInt(i)
    io.out_req(i).bits := fe_req.bits
    io.in_req(i).ready := !be_busy(be_idx) && fe_idx === UInt(i) && be_req.ready

    when (io.finish(i)) {
      be_busy(fe_be_map(i)) := Bool(false)
    }
  }

  when (fe_req.fire()) {
    be_busy(be_idx) := Bool(true)
    fe_be_map(fe_idx) := be_idx
  }
}

class RocketChipNetAdapter[T <: Data](n: Int, dType: T, canSwitch: Boolean)
    extends Module {

  val io = new Bundle {
    val log_in = Decoupled(new RemoteNetworkIO(dType)).flip
    val phys_out = Decoupled(new PhysicalNetworkIO(n, new RemoteNetworkIO(dType)))
    val cur_addr = Vec.fill(n) { new RemoteAddress().asInput }
    val switch_addr = Decoupled(new RemoteAddress)
    val switch_done = Bool(OUTPUT)
    val phys_src = UInt(INPUT, log2Up(n))
    val route_error = Bool(OUTPUT)
  }

  val matching_ports = if (params(AddressRouting)) {
    io.cur_addr.map {
      ra => (io.log_in.bits.header.dst.addr === UInt(0) ||
             io.log_in.bits.header.dst.addr === ra.addr) &&
            (io.log_in.bits.header.dst.port === ra.port)
    }
  } else {
    io.cur_addr.map { ra => (io.log_in.bits.header.dst.port === ra.port) }
  }
  val phys_in_dst = PriorityEncoder(matching_ports)
  val dst_found = Cat(matching_ports).orR

  io.phys_out.bits.header.dst := phys_in_dst
  io.phys_out.bits.header.src := io.phys_src
  io.phys_out.bits.payload := io.log_in.bits
  io.phys_out.valid := io.log_in.valid && dst_found
  io.log_in.ready := io.phys_out.ready

  if (canSwitch) {
    val s_idle :: s_req :: s_wait :: s_finish :: Nil = Enum(Bits(), 4)
    val state = Reg(init = s_idle)

    io.switch_addr.valid := (state === s_req)
    io.switch_addr.bits := io.log_in.bits.header.dst
    io.switch_done := (state === s_finish)

    switch (state) {
      is (s_idle) {
        when (io.log_in.valid && !dst_found) {
          state := s_req
        }
      }
      is (s_req) {
        when (io.switch_addr.ready) {
          state := s_wait
        }
      }
      is (s_wait) {
        when (dst_found) {
          state := s_finish
        }
      }
      is (s_finish) {
        state := s_idle
      }
    }
    io.route_error := Bool(false)
  } else {
    io.route_error := Reg(next = io.log_in.valid && !dst_found)
  }
}

class RouterIO[T <: Data](n: Int, dType: T) extends Bundle {
  val in = Vec.fill(n){Decoupled(new RemoteNetworkIO(dType))}.flip
  val out = Vec.fill(n){Decoupled(new RemoteNetworkIO(dType))}
  val cur_addr = Vec.fill(n){new RemoteAddress().asInput}
  val switch_addr = Vec.fill(n){Decoupled(new RemoteAddress)}
  val route_error = Vec.fill(n) { Bool(OUTPUT) }
}

class RocketChipRouter[T <: Data](n: Int, dType: T, canSwitch: Boolean = false)
    extends Module {
  val io = new RouterIO(n, dType)

  // wrap the LogicalNetworkIO in the PhysicalNetworkIO
  val xbar = Module(new BasicCrossbar(n, new RemoteNetworkIO(dType)))

  val switcher = if (canSwitch) {
    val sw = Module(new RocketChipSwitcher(n))
    sw.io.out_req <> io.switch_addr
    sw.io.cur_addr <> io.cur_addr
    Some(sw)
  } else {
    io.switch_addr.foreach(sr => sr.valid := Bool(false))
    None
  }

  for (i <- 0 until n) {
    val adapter = Module(new RocketChipNetAdapter(n, dType, canSwitch))
    adapter.io.log_in <> io.in(i)
    adapter.io.phys_out <> xbar.io.in(i)
    adapter.io.cur_addr <> io.cur_addr
    adapter.io.phys_src := UInt(i)
    adapter.io.route_error <> io.route_error(i)

    switcher.foreach { sw =>
      sw.io.in_req(i) <> adapter.io.switch_addr
      sw.io.finish(i) <> adapter.io.switch_done
    }

    io.out(i).bits := xbar.io.out(i).bits.payload
    io.out(i).valid := xbar.io.out(i).valid
    xbar.io.out(i).ready := io.out(i).ready
  }
}
