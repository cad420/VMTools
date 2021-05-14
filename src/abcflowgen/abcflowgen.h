#ifndef _ABCFLOWGEN_H_
#define _ABCFLOWGEN_H_


#include <cstddef>
#include <string>

int ABCFlowGen(std::size_t x, std::size_t y, std::size_t z,const std::string & fileName);
int SimpleBlockGen(std::size_t x, std::size_t y, std::size_t z, int xColor, int yColor, int zColor,const std::string & fileName);

#endif /*_ABCFLOWGEN_H_*/