//! \example tutorial-MHD-sequence-reader.cpp
#include <visp3/ustk_io/usMHDSequenceReader.h>
#include <visp3/ustk_io/usMHDSequenceWriter.h>
int main(int argc, char **argv)
{
  std::string sequenceDirectory;
  if (argc == 1) {
    std::cout << "\nUsage: " << argv[0] << " [--input /path/to/mhd/sequence ] \n" << std::endl;
    return 0;
  }

  for (unsigned int i = 1; i < (unsigned int)argc; i++) {
    if (std::string(argv[i]) == "--input") {
      sequenceDirectory = std::string(argv[i + 1]);
      i = argc;
    } else {
      std::cout << "\nUsage: " << argv[0] << " [--input /path/to/mhd/sequence ] \n" << std::endl;
      return 0;
    }
  }

  usImagePreScan3D<unsigned char> image;
  std::vector<uint64_t> timestamp;

  usMHDSequenceReader reader;
  reader.setSequenceDirectory(sequenceDirectory);

  usMHDSequenceWriter writer;
  writer.setSequenceDirectory("/tmp"); // set here your outpur directory

  uint64_t newTimestamp = 0;
  int inc = 0;
  // reading loop
  while (!reader.end()) {
    reader.acquire(image, timestamp);

    // print your image informations
    std::cout << image;

    // add a timestamp of 100ms between each frame
    for (unsigned int i = 0; i < timestamp.size(); i++) {
      timestamp.at(i) = newTimestamp;
      newTimestamp += 100;
    }
    if (inc % 2 ==
        1) // to fit a virtual probe motor sweeping along Z axis (at every volume, frame order is inverted along Z axis)
      std::reverse(timestamp.begin(), timestamp.end());

    writer.write(image, timestamp);
    inc++;
  }

  return 0;
}
