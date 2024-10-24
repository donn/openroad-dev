///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2023, Google LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "dft/Dft.hh"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>

#include "ClockDomain.hh"
#include "DftConfig.hh"
#include "ScanArchitect.hh"
#include "ScanBoundary.hh"
#include "ScanCell.hh"
#include "ScanCellFactory.hh"
#include "ScanReplace.hh"
#include "ScanStitch.hh"
#include "odb/db.h"
#include "utl/Logger.h"

namespace dft {

Dft::Dft() : dft_config_(std::make_unique<DftConfig>())
{
}

void Dft::init(odb::dbDatabase* db, sta::dbSta* sta, utl::Logger* logger)
{
  db_ = db;
  logger_ = logger;
  sta_ = sta;

  // Just to be sure if we are called twice
  reset();
}

void Dft::reset()
{
  scan_replace_.reset();
  need_to_run_pre_dft_ = true;
}

void Dft::pre_dft()
{
  scan_replace_ = std::make_unique<ScanReplace>(db_, sta_, logger_);
  scan_replace_->collectScanCellAvailable();
  scan_boundary_ = std::make_unique<ScanBoundary>(db_, sta_, logger_);

  // This should always be at the end
  need_to_run_pre_dft_ = false;
}

void Dft::previewDft(bool verbose)
{
  if (need_to_run_pre_dft_) {
    pre_dft();
  }

  std::vector<std::unique_ptr<ScanChain>> scan_chains = scanArchitect();

  logger_->report("***************************");
  logger_->report("Preview DFT Report");
  logger_->report("Number of chains: {:d}", scan_chains.size());
  logger_->report("Clock domain: {:s}",
                  ScanArchitectConfig::ClockMixingName(
                      dft_config_->getScanArchitectConfig().getClockMixing()));
  logger_->report("***************************\n");
  for (const auto& scan_chain : scan_chains) {
    scan_chain->report(logger_, verbose);
  }
  logger_->report("");
}

void Dft::scanReplace(bool keep_pl)
{
  if (need_to_run_pre_dft_) {
    pre_dft();
  }
  scan_replace_->scanReplace(keep_pl);
}

void Dft::insertBoundaryScanRegisters(const char* clock_name,
                                      dft::ClockEdge clock_edge,
                                      const char* reset_name,
                                      dft::ResetActiveEdge reset_active,
                                      dft::MuxInfo &input_mux,
                                      const char* testing_net,
                                      sta::PatternMatch* ignore_ports_rx)
{
  dft::ClockDomain clock_domain(clock_name, clock_edge);
  dft::ResetDomain reset_domain(reset_name, reset_active);
  auto block = db_->getChip()->getBlock();
  scan_boundary_->addBoundaryScanRegisters(block, clock_domain, reset_domain, input_mux, testing_net, ignore_ports_rx);
}

void Dft::writeScanChains(std::string filename)
{
  using boost::property_tree::ptree;
  if (need_to_run_pre_dft_) {
    pre_dft();
  }

  std::ofstream json_file(filename);
  if (!json_file.is_open()) {
    logger_->error(
        utl::DFT, 13, "Failed to open file {} for writing.", filename);
  }
  try {
    ptree root;

    std::vector<std::unique_ptr<ScanChain>> scan_chains = scanArchitect();

    for (auto& chain : scan_chains) {
      ptree current_chain;
      ptree cells;
      auto& scan_cells = chain->getScanCells();
      for (auto& cell : scan_cells) {
        ptree name;
        name.put("", cell->getName());
        cells.push_back(std::make_pair("", name));
      }
      current_chain.add_child("cells", cells);
      root.add_child(chain->getName(), current_chain);
    }

    boost::property_tree::write_json(json_file, root);
  } catch (std::exception& ex) {
    logger_->error(
        utl::DFT, 14, "Failed to write JSON report. Exception: {}", ex.what());
  }
}

void Dft::insertDft(bool per_chain_enable,
                    std::string scan_enable_name_pattern,
                    std::string scan_in_name_pattern,
                    std::string scan_out_name_pattern)
{
  if (need_to_run_pre_dft_) {
    pre_dft();
  }
  std::vector<std::unique_ptr<ScanChain>> scan_chains = scanArchitect();

  ScanStitch stitch(db_,
                    per_chain_enable,
                    scan_enable_name_pattern,
                    scan_in_name_pattern,
                    scan_out_name_pattern);
  stitch.Stitch(scan_chains, logger_);
}

DftConfig* Dft::getMutableDftConfig()
{
  return dft_config_.get();
}

const DftConfig& Dft::getDftConfig() const
{
  return *dft_config_;
}

void Dft::reportDftConfig() const
{
  logger_->report("DFT Config:");
  dft_config_->report(logger_);
}

std::vector<std::unique_ptr<ScanChain>> Dft::scanArchitect()
{
  std::vector<std::unique_ptr<ScanCell>> scan_cells
      = CollectScanCells(db_, sta_, logger_);

  // Scan Architect
  std::unique_ptr<ScanCellsBucket> scan_cells_bucket
      = std::make_unique<ScanCellsBucket>(logger_);
  scan_cells_bucket->init(dft_config_->getScanArchitectConfig(), scan_cells);

  std::unique_ptr<ScanArchitect> scan_architect
      = ScanArchitect::ConstructScanScanArchitect(
          dft_config_->getScanArchitectConfig(),
          std::move(scan_cells_bucket),
          logger_);
  scan_architect->init();
  scan_architect->architect();
  scan_architect->tryUseClockNames();

  return scan_architect->getScanChains();
}

}  // namespace dft
