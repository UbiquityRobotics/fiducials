#include "stag/Decoder.h"

#include <stdexcept>
#include <fstream>
#include <string>
#include <sstream>

using std::ifstream;
using std::string;

Decoder::Decoder(int hd) {
  const unsigned long long int* rawCodewords = nullptr;

  if (hd == 11) {
    rawCodewords = HD11;
    noOfCodewords = 22309;
  } else if (hd == 13) {
    rawCodewords = HD13;
    noOfCodewords = 2884;
  } else if (hd == 15) {
    rawCodewords = HD15;
    noOfCodewords = 766;
  } else if (hd == 17) {
    rawCodewords = HD17;
    noOfCodewords = 157;
  } else if (hd == 19) {
    rawCodewords = HD19;
    noOfCodewords = 38;
  } else if (hd == 21) {
    rawCodewords = HD21;
    noOfCodewords = 12;
  } else if (hd == 23) {
    rawCodewords = HD23;
    noOfCodewords = 6;
  } else {
    throw std::invalid_argument("Invalid library HD. Possible values are 11, 13, 15, 17, 19, 21, or 23");
  }

  for (unsigned int i = 0; i < noOfCodewords * 4; i++)
    codewords.push_back(Codeword(rawCodewords[i]));

  // codes are pre-rotated, no need to rotate them again here
}

bool Decoder::decode(const Codeword& c, int errCorr, int& id, int& shift) {
  for (unsigned int i = 0; i < codewords.size(); i++) {
    Codeword xorResult = c ^ codewords[i];  // XOR

    if (xorResult.count() <= errCorr) {
      id = i % noOfCodewords;
      shift = i / noOfCodewords;
      return true;
    }
  }
  return false;
}