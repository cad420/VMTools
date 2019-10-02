
#include <VMFoundation/rawreader.h>
#include <VMat/numeric.h>
#include <string>

#include <iostream>
#include <fstream>
#include <VMUtils/cmdline.hpp>
#include <VMUtils/ref.hpp>
#include <VMCoreExtension/ifilemappingplugininterface.h>
#include <VMFoundation/pluginloader.h>
#include <VMUtils/cmdline.hpp>


namespace ysl
{

	template<typename T>
	class Sampler3D
	{
		T * const data;
		const Size3 size;
	public:
		Sampler3D(T * data,const ysl::Size3 size):data(data),size(size)
		{
			
		}
		ysl::Float Sample(const ysl::Point3i & p)
		{
			ysl::Bound3i bound(ysl::Point3i(0, 0, 0), ysl::Point3i(size.x,size.y,size.z));
			if (!bound.InsideEx(p))
				return 0;
			return (*this)(p.x, p.y, p.z);
		}

		Float Sample(const Point3f & p)
		{
			const auto pi = Point3i(std::floor(p.x), std::floor(p.y), std::floor(p.z));
			const auto d = p - static_cast<Point3f>(pi);
			const auto d00 = Lerp(d.x, Sample(pi), Sample(pi + Vector3i(1, 0, 0)));
			const auto d10 = Lerp(d.x, Sample(pi + Vector3i(0, 1, 0)), Sample(pi + Vector3i(1, 1, 0)));
			const auto d01 = Lerp(d.x, Sample(pi + Vector3i(0, 0, 1)), Sample(pi + Vector3i(1, 0, 1)));
			const auto d11 = Lerp(d.x, Sample(pi + Vector3i(0, 1, 1)), Sample(pi + Vector3i(1, 1, 1)));
			const auto d0 = ysl::Lerp(d.y, d00, d10);
			const auto d1 = ysl::Lerp(d.y, d01, d11);
			return ysl::Lerp(d.z, d0, d1);
		}

		T operator()(int x,int y,int z)const
		{
			return data[z * size.y*size.x + y * size.x + x];
		}

		T * Get()
		{
			return data;
		}

		Size3 GetSize()const
		{
			return size;
		}
	};
}


ysl::Size3 SampleSize(const ysl::Size3 & orignalSize, const ysl::Vector3f & factor)
{
	return ysl::Size3(1.0*orignalSize.x / factor.x, 1.0*orignalSize.y / factor.y, 1.0*orignalSize.z / factor.z);
}

template <typename T>
void Sample(const std::string& inFileName, int offset, int x, int y, int z, float sx, float sy, float sz, const std::string& outFileName)
{


	const auto sampleSize = ysl::Size3(1.0 * x / sx + 0.5, 1.0 * y / sy + 0.5, 1.0 * z / sz + 0.5);
	ysl::Vector3f step(1.0 * x / sampleSize.x, 1.0 * y / sampleSize.y, 1.0 * z / sampleSize.z);


	std::cout << "Downsample Size:" << sampleSize << std::endl;
	std::cout << "Step:" << step << std::endl;
	
	std::unique_ptr<T> buf(new T[x * y * z]);
	vm::Ref<IFileMapping> rm;
#ifdef _WIN32
	rm = ysl::PluginLoader::GetPluginLoader()->CreatePlugin<IFileMapping>("windows");
#else
	rm = ysl::PluginLoader::GetPluginLoader()->CreatePlugin<IFileMapping>("linux");
#endif



	if (rm == nullptr)
		throw std::runtime_error("IO plugin can not be loaded");
	rm->Open(inFileName, x * y * z, FileAccess::Read, MapAccess::ReadOnly);
	const auto ptr = rm->FileMemPointer(0, x * y * z + offset);
	if (!ptr)
	{
		std::cout << "File mapping failed\n";
		return;
	}
	
	ysl::Sampler3D<T> sampler(reinterpret_cast<T*>((char*)ptr + offset),ysl::Size3(x,y,z));


	const auto sliceStep = 5;
	const auto bytes = sampleSize.x * sampleSize.y * sliceStep;
	std::unique_ptr<T[]> downsampleData(new T[bytes]);
	//std::unique_ptr<T[]> originalData(new T[x * y * sliceStep]);
	std::ofstream out(outFileName, std::ios::binary);

	for (int zz = 0; zz < sampleSize.z; zz += sliceStep)
	{
		std::size_t actualSlice;
		if (zz + sliceStep >= sampleSize.z)
			actualSlice = sampleSize.z - zz;
		else
			actualSlice = sliceStep;

		const auto actualBytes = actualSlice * sampleSize.x * sampleSize.y;

		for (int s = 0; s < actualSlice; s++)
		{
			for (int yy = 0; yy < sampleSize.y; yy++)
			{
				for (int xx = 0; xx < sampleSize.x; xx++)
				{
					const auto index = sampleSize.x * ((s)* sampleSize.y + yy) + xx;
					downsampleData[index] = sampler.Sample(ysl::Point3f(xx * step.x, yy * step.y, (zz + s) * step.z));
				}
			}
		}

		out.write(reinterpret_cast<char*>(downsampleData.get()), actualBytes);
		std::cout << "Writing: " << zz << " to " << zz + actualSlice << " of " << actualBytes << " finished\n";
	}
	system("pause");
}

int main()
{
	// load plugin
	ysl::PluginLoader::LoadPlugins("ioplugin");

	std::size_t x, y, z;
	float sx, sy, sz;
	int type;
	std::string inFileName, outFileName;
	std::size_t offset;
	std::cout << "[filename(str), offset(std::size_t),  x(int), y(int), z(int),xfactor(int), yfactor(int),zfactor(int), ouputfilename(str), type(0 = int,1 = byte, 2 = short int,3 = float)]\n";
	std::cin >> inFileName >> offset >> x >> y >> z >> sx >> sy >> sz >> outFileName >> type;
	
	if (type == 0)
		Sample<int>(inFileName, offset, x, y, z, sx, sy, sz, outFileName);
	else if (type == 1)
		Sample<unsigned char>(inFileName, offset, x, y, z, sx, sy, sz, outFileName);
	else if (type == 2)
		Sample<unsigned short>(inFileName, offset, x, y, z, sx, sy, sz, outFileName);
	else if (type == 3)
		Sample<float>(inFileName, offset, x, y, z, sx, sy, sz, outFileName);
	else
	{
		std::cout << "unknown type\n";
	}

}
