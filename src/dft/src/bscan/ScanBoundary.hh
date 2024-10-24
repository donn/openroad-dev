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
#pragma once

#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "ClockDomain.hh"
#include "ResetDomain.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "utl/Logger.h"

namespace dft {
  
struct MuxInfo {
  odb::dbMaster *master;
  odb::dbMTerm *s, *a0, *a1, *y;
};

// Performs scan replacement on a OpenROAD's database
class ScanBoundary
{
 public:
  ScanBoundary(odb::dbDatabase* db, sta::dbSta* sta, utl::Logger* logger);

  void addBoundaryScanRegisters(odb::dbBlock* block,
                                              dft::ClockDomain &clock_domain,
                                              dft::ResetDomain &reset_domain,
                                              MuxInfo &input_mux,
                                              std::string testing_net_name,
                                              sta::PatternMatch* ignore_ports_rx = nullptr);

  // supposed to be private, just testing though
 private:
  struct ScanFlopInfo
  {
    odb::dbMaster* db_;
    sta::LibertyCell* lib_;

    ScanFlopInfo(odb::dbMaster* db, sta::LibertyCell* lib) : db_(db), lib_(lib)
    {
    }
  };

  // returns viable scannable flip-flops matching clock and reset edge
  // information, sorted smallest first.
  std::vector<ScanFlopInfo> GetScanFlops(dft::ClockEdge for_edge,
                                         dft::ResetActiveEdge for_reset);
                                         
  odb::dbNet* FindOrCreateTesting(odb::dbBlock* block,
                                                std::string with_name,
                                                utl::Logger* logger);

  // checks if a cell is indeed triggered by that clock edge and reset by that
  // particular reset edge
  bool IsCandidate(sta::LibertyCell* cell,
                   dft::ClockEdge for_edge,
                   dft::ResetActiveEdge for_reset);

  odb::dbDatabase* db_;
  sta::dbSta* sta_;
  utl::Logger* logger_;
  sta::dbNetwork* db_network_;

  std::optional<std::vector<ScanFlopInfo>> scanflops_;
};

}  // namespace dft
