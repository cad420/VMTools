
#include "abcflowgen.h"
#include <VMUtils/cmdline.hpp>
#include <sstream>

int main(int argc,char ** argv)
{
	cmdline::parser a;
	a.add<std::string>("of", 'o', "specifies output file",false,"output.raw");
	a.add<std::string>("gen", 'g', "sepcified generating method", true);
	a.add<size_t>("width", 'w', "data width in bytes", true);
	a.add<size_t>("height", 'h', "data height in bytes", true);
	a.add<size_t>("depth", 'd', "data depth in bytes", true);
	a.add<size_t>("xcolor",'x',"color number along x axis",false);
	a.add<size_t>("ycolor",'y',"color number along y axis",false);
	a.add<size_t>("zcolor",'z',"color number along z axis",false);
	a.parse(argc, argv);
	auto gen = a.get<std::string>("gen");
	auto output = a.get<std::string>("of");
	auto w = a.get<size_t>("width");
	auto h = a.get<size_t>("height");
	auto d = a.get<size_t>("depth");
	auto x = a.get<size_t>("xcolor");
	auto y = a.get<size_t>("ycolor");
	auto z = a.get<size_t>("zcolor");
	std::string prefix = "_"+std::to_string(w)+"_"+std::to_string(h)+"_"+std::to_string(d)+".raw";
	std::string fileName = gen + prefix;
	if (gen == "abcflow") {
		ABCFlowGen(w, h, d, fileName);
	}
	else if(gen == "simple") {
		SimpleBlockGen(w, h, d, x, y, z, fileName);
	}
	return 0;
}