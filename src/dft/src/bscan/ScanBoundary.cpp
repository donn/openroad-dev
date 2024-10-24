///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2024, Efabless Corporation
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

#include "ScanBoundary.hh"

#include <iostream>
#include <limits>

#include "Utils.hh"
#include "db_sta/dbNetwork.hh"
#include "sta/EquivCells.hh"
#include "sta/FuncExpr.hh"
#include "sta/Liberty.hh"
#include "sta/PatternMatch.hh"
#include "sta/Sequential.hh"

namespace dft {
ScanBoundary::ScanBoundary(odb::dbDatabase* db,
                           sta::dbSta* sta,
                           utl::Logger* logger)
    : db_(db), sta_(sta), logger_(logger)
{
  db_network_ = sta_->getDbNetwork();
}

bool ScanBoundary::IsCandidate(sta::LibertyCell* cell,
                               dft::ClockEdge for_edge,
                               dft::ResetActiveEdge for_reset)
{
  if (!utils::IsScanCell(cell)) {
    return false;
  }

  // Metadata
  auto sequentials = cell->sequentials();
  if (sequentials.size() != 1) {
    return false;  // Complex cell
  }
  auto sequential = sequentials[0];

  // 1. Check if clock matches expected
  auto clock_expr = sequential->clock();
  if (clock_expr == nullptr) {
    return false;
  }
  if ((clock_expr->op() == sta::FuncExpr::op_not)
      != (for_edge == dft::ClockEdge::Falling)) {
    return false;
  }

  // 2. Check if reset matches expected
  auto clear_expr = sequential->clear();
  if (clear_expr == nullptr) {
    return false;
  }
  if ((clear_expr->op() == sta::FuncExpr::op_not)
      != (for_reset == dft::ResetActiveEdge::Low)) {
    return false;
  }

  return true;
}

std::vector<dft::ScanBoundary::ScanFlopInfo> ScanBoundary::GetScanFlops(
    dft::ClockEdge for_edge,
    dft::ResetActiveEdge for_reset)
{
  auto comparison = [](ScanFlopInfo a, ScanFlopInfo b) {
    return a.lib_->area() < b.lib_->area();
  };

  std::vector<ScanFlopInfo> candidates;
  std::make_heap(candidates.begin(), candidates.end(), comparison);

  auto libs = db_->getLibs();
  for (auto lib : libs) {
    for (auto master : lib->getMasters()) {
      if (master->getType() != odb::dbMasterType::CORE) {
        continue;
      }

      auto sta_master = db_network_->dbToSta(master);
      if (sta_master == nullptr) {
        continue;  // Not a timing-related cell, therefore, not a scan cell
      }

      auto sta_lib_master = db_network_->libertyCell(sta_master);
      if (sta_lib_master == nullptr) {
        continue;  // No liberty cell
      }

      if (!IsCandidate(sta_lib_master, for_edge, for_reset)) {
        continue;  // Mismatched request clock/reset
      }

      candidates.push_back(ScanFlopInfo(master, sta_lib_master));

      std::push_heap(candidates.begin(), candidates.end(), comparison);
    }
  }

  std::sort_heap(candidates.begin(), candidates.end(), comparison);

  return candidates;
}

odb::dbNet* ScanBoundary::FindOrCreateTesting(odb::dbBlock* block,
                                              std::string with_name,
                                              utl::Logger* logger)
{
  auto net = block->findNet(with_name.data());
  if (net != nullptr) {
    return net;
  }

  auto port = dft::utils::CreateNewPort(block, with_name, logger, "input");
  
  return port->getNet();
}


void ScanBoundary::addBoundaryScanRegisters(odb::dbBlock* block,
                                            dft::ClockDomain &clock_domain,
                                            dft::ResetDomain &reset_domain,
                                            MuxInfo &input_mux,
                                            std::string testing_net_name,
                                            sta::PatternMatch* ignore_ports_rx)
{
  
  auto scanflops = GetScanFlops(clock_domain.getClockEdge(), reset_domain.getResetActiveEdge());
  if (scanflops.size() == 0) {
    logger_->error(
      utl::DFT,
      26,
      "No candidate scannable flip-flops found within the loaded cell masters. Make sure you have read the requisite lib files."
    );
  }
  
  // 1. Process Scanflop used for Boundary Scan Registers - get port names
  auto scanflop = scanflops[0]; // smallest
  auto test_info = scanflop.lib_->testCell();
  auto seq_info = scanflop.lib_->sequentials()[0]; // we already checked if it has at least one sequential
  
  // Assuming that the expressions are either PORT or !PORT
  auto clock_port_expr = seq_info->clock();
  if (clock_port_expr->op() == sta::FuncExpr::op_not) {
    clock_port_expr = clock_port_expr->left();
  }
  auto reset_port_expr = seq_info->clear();
  if (reset_port_expr->op() == sta::FuncExpr::op_not) {
    reset_port_expr = reset_port_expr->left();
  }
  auto clock_pin_name = clock_port_expr->port()->name();
  auto reset_pin_name = reset_port_expr->port()->name();
  auto data_pin_name = test_info->dataIn()->name();
  auto output_pin_name = test_info->scanOut()->name();
  
  // 2. Get global clock, reset and testing nets
  auto clock_net_name = std::string(clock_domain.getClockName());
  auto reset_net_name = std::string(reset_domain.getResetName());
  
  auto clock_net = block->findNet(clock_net_name.c_str());
  if (clock_net == nullptr) {
    logger_->error(
      utl::DFT,
      28,
      "Clock net {} not found",
      clock_net_name
    );
  }
  auto reset_net = block->findNet(reset_net_name.c_str());
  if (reset_net == nullptr) {
    logger_->error(
      utl::DFT,
      29,
      "Reset net {} not found",
      reset_net_name
    );
  }
  auto testing_net = FindOrCreateTesting(block, testing_net_name, logger_);
  assert(testing_net != nullptr);
  
  // 4. Iterate over bterms and add boundary scan registers
  auto bterms = block->getBTerms();
  for (auto bterm: bterms) {
    if (bterm->getIoType() != odb::dbIoType::INPUT && bterm->getIoType() != odb::dbIoType::OUTPUT) {
      // ignore inouts
      continue;
    }
    auto term_name = bterm->getName();
    auto term_net = bterm->getNet();
    
    if (term_net == testing_net
      || term_net == clock_net
      || term_net == reset_net) {
      continue;
    }
    
    if (ignore_ports_rx->match(term_name)) {
      continue;
    }
    
    auto bsr_name = fmt::format("{}.{}", term_name, "bsr");
    auto bsr = odb::dbInst::create(block, scanflop.db_, bsr_name.c_str());
    
    auto data_iterm = bsr->findITerm(data_pin_name);
    data_iterm->connect(term_net);
    
    auto clock_iterm = bsr->findITerm(clock_pin_name);
    clock_iterm->connect(clock_net);
    
    auto reset_iterm = bsr->findITerm(reset_pin_name);
    reset_iterm->connect(reset_net);
    
    if (bterm->getIoType() == odb::dbIoType::INPUT) {
      auto mkMuxNet = [&](const char* name) {
        auto full_name = fmt::format("{}.bsr_mux_{}", term_name, name);
        auto created_net = odb::dbNet::create(block, full_name.c_str());
        assert(created_net != nullptr);
        return created_net;
      };
      
      auto mux_name = fmt::format("{}.{}", term_name, "bsr_mux");
      auto mux = odb::dbInst::create(block, input_mux.master, mux_name.c_str());
      auto a1 = mkMuxNet("a1");
      auto y = mkMuxNet("y");
      
      #define getMuxITermFromInfo(port)\
        ([&]() {\
          auto _tmp = mux->findITerm(input_mux.port->getName().c_str());\
          assert (_tmp != nullptr);\
          return _tmp;\
        })()
      
      // Replace iterms connected to input with that of the muxed output (y)
      // -- This includes the connection to the BSR itself.
      auto mux_y_iterm = getMuxITermFromInfo(y);
      mux_y_iterm->connect(y);
      std::vector<odb::dbITerm*> current_term_net_iterms(
        term_net->getITerms().begin(), 
        term_net->getITerms().end()
      );
      for (auto iterm: current_term_net_iterms) {
        iterm->disconnect();
        iterm->connect(y);
      }
      
      // Connect testing as a selection net
      auto mux_s_iterm = getMuxITermFromInfo(s);
      mux_s_iterm->connect(testing_net);
      
      // Connect raw value of pin to a0 of multiplexer
      auto mux_a0_iterm = getMuxITermFromInfo(a0);
      mux_a0_iterm->connect(term_net);
      
      // Connect value of flipflop to a1 of multiplexer
      auto mux_a1_iterm = getMuxITermFromInfo(a1);
      mux_a1_iterm->connect(a1);
      auto output_iterm = bsr->findITerm(output_pin_name);
      output_iterm->connect(a1);
      
      #undef getMuxITermFromInfo
    }
  }
}

}// namespace dft
