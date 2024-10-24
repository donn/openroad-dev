#include "ScanStitch.hh"

#include <deque>
#include <iostream>

namespace dft {

ScanStitch::ScanStitch(odb::dbDatabase* db,
                       bool per_chain_enable,
                       std::string scan_enable_name_pattern,
                       std::string scan_in_name_pattern,
                       std::string scan_out_name_pattern)
    : db_(db),
      per_chain_enable_(per_chain_enable),
      scan_enable_name_pattern_(scan_enable_name_pattern),
      scan_in_name_pattern_(scan_in_name_pattern),
      scan_out_name_pattern_(scan_out_name_pattern)
{
  odb::dbChip* chip = db_->getChip();
  top_block_ = chip->getBlock();
}

void ScanStitch::Stitch(
    const std::vector<std::unique_ptr<ScanChain>>& scan_chains,
    utl::Logger* logger)
{
  // TODO: For now, we only use one scan enable for all the chains. We may
  // support in the future multiple test modes
  if (scan_chains.empty()) {
    return;
  }

  auto scanEnableName = fmt::format(FMT_RUNTIME(scan_enable_name_pattern_), 0);

  std::optional<ScanDriver> scan_enable = std::nullopt;
  if (!per_chain_enable_) {
    scan_enable = FindOrCreateScanEnable(top_block_, scanEnableName, logger);
  }

  size_t ordinal = 0;
  for (const std::unique_ptr<ScanChain>& scan_chain : scan_chains) {
    Stitch(top_block_, *scan_chain, scan_enable, logger, ordinal++);
  }
}

void ScanStitch::Stitch(odb::dbBlock* block,
                        const ScanChain& scan_chain,
                        const std::optional<ScanDriver>& scan_enable_opt,
                        utl::Logger* logger,
                        size_t ordinal)
{
  auto scan_enable_name
      = fmt::format(FMT_RUNTIME(scan_enable_name_pattern_), ordinal);
  auto scan_enable = scan_enable_opt.value_or(
      FindOrCreateScanEnable(block, scan_enable_name, logger));

  // Let's create the scan in and scan out of the chain
  // TODO: Suport user defined scan signals
  auto scan_in_name = fmt::format(FMT_RUNTIME(scan_in_name_pattern_), ordinal);
  ScanDriver scan_in_port = FindOrCreateScanIn(block, scan_in_name, logger);

  // We need fast pop for front and back
  std::deque<std::reference_wrapper<const std::unique_ptr<ScanCell>>>
      scan_cells;

  const std::vector<std::unique_ptr<ScanCell>>& original_scan_cells
      = scan_chain.getScanCells();

  std::copy(original_scan_cells.cbegin(),
            original_scan_cells.cend(),
            std::back_inserter(scan_cells));

  // All the cells in the scan chain are controlled by the same scan enable
  for (const std::unique_ptr<ScanCell>& scan_cell : scan_cells) {
    scan_cell->connectScanEnable(scan_enable);
  }

  // Lets get the first and last cell
  const std::unique_ptr<ScanCell>& first_scan_cell = *scan_cells.begin();
  const std::unique_ptr<ScanCell>& last_scan_cell = *(scan_cells.end() - 1);

  if (!scan_cells.empty()) {
    scan_cells.pop_front();
  }

  if (!scan_cells.empty()) {
    scan_cells.pop_back();
  }

  for (auto it = scan_cells.begin(); it != scan_cells.end(); ++it) {
    const std::unique_ptr<ScanCell>& current = *it;
    const std::unique_ptr<ScanCell>& next = *(it + 1);
    // Connects current cell scan out to next cell scan in
    next->connectScanIn(current->getScanOut());
  }

  // Let's connect the first cell
  first_scan_cell->connectScanEnable(scan_enable);
  first_scan_cell->connectScanIn(scan_in_port);

  if (!scan_cells.empty()) {
    scan_cells.begin()->get()->connectScanIn(first_scan_cell->getScanOut());
  } else {
    // If last_scan_cell == first_scan_cell, then scan in was already connected
    if (last_scan_cell != first_scan_cell) {
      last_scan_cell->connectScanIn(first_scan_cell->getScanOut());
    }
  }

  // Let's connect the last cell
  auto scan_out_name
      = fmt::format(FMT_RUNTIME(scan_out_name_pattern_), ordinal);
  ScanLoad scan_out_port = FindOrCreateScanOut(
      block, last_scan_cell->getScanOut(), scan_out_name, logger);
  last_scan_cell->connectScanOut(scan_out_port);
}

ScanDriver ScanStitch::FindOrCreateScanEnable(odb::dbBlock* block,
                                              std::string with_name,
                                              utl::Logger* logger)
{
  auto net = block->findNet(with_name.data());
  if (net != nullptr) {
    for (auto bterm : net->getBTerms()) {
      if (bterm->getIoType() == odb::dbIoType::INPUT) {
        return ScanDriver(bterm);
      }
    }
    for (auto iterm : net->getITerms()) {
      if (iterm->getIoType() == odb::dbIoType::OUTPUT) {
        return ScanDriver(iterm);
      }
    }
  }

  return CreateNewPort<ScanDriver>(block, with_name, logger, net);
}

ScanDriver ScanStitch::FindOrCreateScanIn(odb::dbBlock* block,
                                          std::string with_name,
                                          utl::Logger* logger)
{
  auto net = block->findNet(with_name.data());
  if (net != nullptr) {
    for (auto bterm : net->getBTerms()) {
      if (bterm->getIoType() == odb::dbIoType::INPUT) {
        return ScanDriver(bterm);
      }
    }
    for (auto iterm : net->getITerms()) {
      if (iterm->getIoType() == odb::dbIoType::OUTPUT) {
        return ScanDriver(iterm);
      }
    }
  }

  return CreateNewPort<ScanDriver>(block, with_name, logger, net);
}

ScanLoad ScanStitch::FindOrCreateScanOut(odb::dbBlock* block,
                                         const ScanDriver& cell_scan_out,
                                         std::string with_name,
                                         utl::Logger* logger)
{
  // TODO: For now we will create a new scan_out pin at the top level if we need
  // one. We need to support defining DFT signals for scan_out
  auto net = block->findNet(with_name.data());
  if (net != nullptr) {
    for (auto bterm : net->getBTerms()) {
      if (bterm->getIoType() == odb::dbIoType::OUTPUT) {
        return ScanLoad(bterm);
      }
    }
    for (auto iterm : net->getITerms()) {
      if (iterm->getIoType() == odb::dbIoType::INPUT) {
        return ScanLoad(iterm);
      }
    }
  }

  // TODO: Trace forward the scan out net so we can see if it is connected to a
  // top port or to functional logic
  odb::dbNet* scan_out_net = cell_scan_out.getNet();
  if (scan_out_net && top_block_ == scan_out_net->getBlock()) {
    // if the scan_out_net exists, and has an BTerm that is an OUTPUT, then we
    // can reuse that BTerm to act as scan_out, only if the block of the bterm
    // is the top block, otherwise we will punch a new port
    for (odb::dbBTerm* bterm : scan_out_net->getBTerms()) {
      if (bterm->getIoType() == odb::dbIoType::OUTPUT) {
        net = odb::dbNet::create(block, with_name.c_str());
        bterm->connect(net);
        return ScanLoad(bterm);
      }
    }
  }

  return CreateNewPort<ScanLoad>(block, with_name, logger, net);
}

}  // namespace dft
