// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <iostream>

#include "test/integration/map/native_structured_closure.h"

int main(int argc, char** argv) {
  try {
    if (argc != 3 && (argc != 4 || std::string(argv[3]) != "--causal-prefixes"))
      throw std::runtime_error(
          "usage: native_structured_fault_replay FROZEN_BUNDLE NEW_OUTPUT "
          "[--causal-prefixes]");
    openloc::replay::NativeStructuredClosure::ReplayFrozen(argv[1], argv[2], argc == 4);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
