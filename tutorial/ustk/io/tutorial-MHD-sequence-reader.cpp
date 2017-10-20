#include <visp3/ustk_io/usMHDSequenceReader.h>
#include <visp3/ustk_io/usMHDSequenceWriter.h>
int main(int argc, char** argv)
{
  std::string sequenceDirectory;
  if(argc == 1) {
    std::cout << "\nUsage: " << argv[0] << " [--input /path/to/mhd/sequence ] \n" << std::endl;
    return 0;
  }

  for (unsigned int i=1; i<(unsigned int)argc; i++) {
    if (std::string(argv[i]) == "--input") {
      sequenceDirectory = std::string(argv[i+1]);
      i = argc;
    }
    else {
      std::cout << "\nUsage: " << argv[0] << " [--input /path/to/mhd/sequence ] \n" << std::endl;
      return 0;
    }
  }

  usImagePreScan3D<unsigned char> image;
  uint64_t timestamp;

  usMHDSequenceReader reader;
  reader.setSequenceDirectory(sequenceDirectory);

  usMHDSequenceWriter writer;
  writer.setSequenceDirectory("/tmp");

  //reading loop
  while ( !reader.end()) {
    reader.acquire(image,timestamp);

    std::cout << image;
    std::cout << "timestamp : " << timestamp << std::endl;

    timestamp = 1561565362;
    writer.writeImage(image,timestamp);
  }

  return 0;
}
