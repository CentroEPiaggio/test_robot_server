// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include "mact_controllers/common/types.hpp"

namespace mact_controllers {

bool adaptationTorqueSourceFromString(
  const std::string & name, AdaptationTorqueSource & source)
{
  if (name == "measured") {
    source = AdaptationTorqueSource::kMeasured;
    return true;
  }
  if (name == "commanded") {
    source = AdaptationTorqueSource::kCommanded;
    return true;
  }
  return false;
}

const char * toString(AdaptationTorqueSource source)
{
  switch (source) {
    case AdaptationTorqueSource::kMeasured:
      return "measured";
    case AdaptationTorqueSource::kCommanded:
      return "commanded";
  }
  return "unknown";
}

}  // namespace mact_controllers
