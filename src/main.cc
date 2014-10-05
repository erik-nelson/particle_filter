
#include <cstdlib>
#include <stdio.h>
#include "../include/ParticleFilter.h"

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr
    << "[ERROR: main.cc]: Usage: ./particle_filter.o CONFIG_FILENAME"
    << std::endl;;
    return EXIT_FAILURE;
  }

  // Convert argument(s) to vector of strings
  std::vector<std::string> args(argv, argv + argc);

  // Instantiate and initialize particle filter
  ParticleFilter p;
  if (!p.initialize(args))
  {
    printf("[ERROR: particle_filter.cc]: Failed to initialize ParticleFilter\n");
    return EXIT_FAILURE;
  }

  // Spin and display particle filter output
  p.run();

  return EXIT_SUCCESS;
}
