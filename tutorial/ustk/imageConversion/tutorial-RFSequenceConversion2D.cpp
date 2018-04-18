#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_FFTW

#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_core/usMHDSequenceReader.h>
#include <visp3/ustk_core/usMHDSequenceWriter.h>

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

  usImageRF2D<short int> imageRF;
  usImagePreScan2D<unsigned char> imagePreScan;
  usMHDSequenceReader reader;
  reader.setSequenceDirectory(sequenceDirectory);

  usMHDSequenceWriter writer;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  writer.setSequenceDirectory("/tmp"); // set here your outpur directory
#elif defined(_WIN32)
  writer.setSequenceDirectory("C:\\temp"); // set here your outpur directory
#endif

  usRFToPreScan2DConverter converter;

  uint64_t timestamp = 0;
  int inc = 0;
  // reading/converting loop
  while (!reader.end()) {
    reader.acquire(imageRF, timestamp);
    converter.convert(imageRF, imagePreScan);
    writer.write(imagePreScan, timestamp);
    std::cout << "image " << inc << " successfully converted\n";
  }

  return 0;
}
#else
#include <iostream>
int main()
{
  std::cout << "this tutorial requirest RF to pre-scan conversion, so you have to install FFTW thirdparty\n";
  return 0;
}
#endif
