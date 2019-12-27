


#include "sample.hpp"

//
//vm::Size3 SampleSize(const vm::Size3& orignalSize, const vm::Vector3f& factor)
//{
//	return vm::Size3(1.0 * orignalSize.x / factor.x, 1.0 * orignalSize.y / factor.y, 1.0 * orignalSize.z / factor.z);
//}
//
//template <typename T>
//void Sample(const std::string& inFileName, int offset, int x, int y, int z, float sx, float sy, float sz, const std::string& outFileName)
//{
//
//
//	const auto sampleSize = vm::Size3(1.0 * x / sx + 0.5, 1.0 * y / sy + 0.5, 1.0 * z / sz + 0.5);
//	vm::Vector3f step(1.0 * x / sampleSize.x, 1.0 * y / sampleSize.y, 1.0 * z / sampleSize.z);
//
//
//	std::cout << "Downsample Size:" << sampleSize << std::endl;
//	std::cout << "Step:" << step << std::endl;
//
//	std::unique_ptr<T> buf(new T[x * y * z]);
//	vm::Ref<IMappingFile> rm;
//#ifdef _WIN32
//	rm = vm::PluginLoader::GetPluginLoader()->CreatePlugin<IMappingFile>("windows");
//#else
//	rm = ysl::PluginLoader::GetPluginLoader()->CreatePlugin<IMappingFile>("linux");
//#endif
//
//
//
//	if (rm == nullptr)
//		throw std::runtime_error("IO plugin can not be loaded");
//	rm->Open(inFileName, x * y * z, FileAccess::Read, MapAccess::ReadOnly);
//	const auto ptr = rm->FileMemPointer(0, x * y * z + offset);
//	if (!ptr)
//	{
//		std::cout << "File mapping failed\n";
//		return;
//	}
//
//	Sampler3D<T> sampler(reinterpret_cast<T*>((char*)ptr + offset), vm::Size3(x, y, z));
//
//
//	const auto sliceStep = 5;
//	const auto bytes = sampleSize.x * sampleSize.y * sliceStep;
//	std::unique_ptr<T[]> downsampleData(new T[bytes]);
//	//std::unique_ptr<T[]> originalData(new T[x * y * sliceStep]);
//	std::ofstream out(outFileName, std::ios::binary);
//
//	for (int zz = 0; zz < sampleSize.z; zz += sliceStep)
//	{
//		std::size_t actualSlice;
//		if (zz + sliceStep >= sampleSize.z)
//			actualSlice = sampleSize.z - zz;
//		else
//			actualSlice = sliceStep;
//
//		const auto actualBytes = actualSlice * sampleSize.x * sampleSize.y;
//
//		for (int s = 0; s < actualSlice; s++)
//		{
//			for (int yy = 0; yy < sampleSize.y; yy++)
//			{
//				for (int xx = 0; xx < sampleSize.x; xx++)
//				{
//					const auto index = sampleSize.x * ((s)*sampleSize.y + yy) + xx;
//					downsampleData[index] = sampler.Sample(vm::Point3f(xx * step.x, yy * step.y, (zz + s) * step.z));
//				}
//			}
//		}
//
//		out.write(reinterpret_cast<char*>(downsampleData.get()), actualBytes);
//		std::cout << "Writing: " << zz << " to " << zz + actualSlice << " of " << actualBytes << " finished\n";
//	}
//	system("pause");
//}

//template<typename M>
//struct Method
//{
//	static Point3f Trans(const Point3i& sampledGridGlobalPos, const Vec3i& batchSize)
//	{
//		return M::Trans();
//	}
//};
//



int main(int argc, char** argv)
{
	using namespace std;
	using namespace vm;
	
	cmdline::parser a;

	a.add<string>("if", 'i', "input file name", true);
	a.add<string>("of", 'o', "out file name", false, "a.out");

	a.add<int>("offset", 0, "offset from beginning of the file", false, 0);
	a.add<int>("width", 'x', "width of the 3d data", true, 0);
	a.add<int>("height", 'y', "height of the 3d data", true, 0);
	a.add<int>("depth", 'z', "depth of the 3d data", true, 0);
	
	a.add<float>("sx", 0, "scale factor for the x-axis", true, 2);
	a.add<float>("sy", 0, "scale factor for the y-axis", true, 2);
	a.add<float>("sz", 0, "scale factor for the z-axis", true, 2);
	a.add<string>("filter", 'f', "filter for the re-sampling", false, "trilinear", cmdline::oneof(string("trilinear"), string("gaussknl"), string("maxknl"), string("meanknl"), string("simple")));

	//a.add<string>("type", 0, "element type of the 3d data", false, "char", cmdline::oneof("char", "short", "int", "float", "double"));
	a.parse_check(argc, argv);

	const string inFileName = a.get<string>("if");
	const string outFileName = a.get<string>("of");
	Vec3f scale(a.get<float>("sx"), a.get<float>("sy"), a.get<float>("sz"));
	Size3 size(a.get<int>("width"), a.get<int>("height"), a.get<int>("depth"));
	size_t offset = a.get<int>("offset");

	//string type = a.get<string>("type");

	string filter = a.get<string>("filter");

	
	if (filter == "trilinear") {Sampler<TriLinear<char>>::Resample(size, inFileName, scale, outFileName); return 0; }
	if (filter == "gaussknl") {Sampler<Gaussian2x2Kernel<char>>::Resample(size, inFileName, scale, outFileName); return 0;}
	if (filter == "maxknl") {Sampler<MaxKernel<char>>::Resample(size, inFileName, scale, outFileName); return 0;}
	if (filter == "meanknl"){Sampler<Mean2x2Kernel<char>>::Resample(size, inFileName, scale, outFileName); return 0;}
	if (filter == "simple") {Sampler<Trivial2x2Kernel<char>>::Resample(size, inFileName, scale, outFileName); return 0;}
	
	return 0;
}
