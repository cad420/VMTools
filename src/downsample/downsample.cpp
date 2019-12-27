
#include <VMFoundation/rawreader.h>
#include <VMat/numeric.h>
#include <VMUtils/log.hpp>
#include <string>

#include <iostream>
#include <fstream>
#include <VMUtils/cmdline.hpp>
#include <VMUtils/ref.hpp>
#include <VMFoundation/pluginloader.h>
#include <VMCoreExtension/ifilemappingplugininterface.h>
#include <VMFoundation/rawreader.h>
#include <fstream>

using namespace std;
using namespace vm;

template<typename T>
class Sampler3D
{
	const T* const data;
	const vm::Size3 size;
public:
	Sampler3D(const  T* data, const vm::Size3 size) :data(data), size(size)
	{

	}
	vm::Float Sample(const vm::Point3i& p)
	{
		vm::Bound3i bound(vm::Point3i(0, 0, 0), vm::Point3i(size.x, size.y, size.z));
		if (!bound.InsideEx(p))
			return 0;
		return (*this)(p.x, p.y, p.z);
	}

	vm::Float Sample(const vm::Point3f& p)
	{
		const auto pi = vm::Point3i(std::floor(p.x), std::floor(p.y), std::floor(p.z));
		const auto d = p - static_cast<vm::Point3f>(pi);
		const auto d00 = vm::Lerp(d.x, Sample(pi), Sample(pi + vm::Vector3i(1, 0, 0)));
		const auto d10 = vm::Lerp(d.x, Sample(pi + vm::Vector3i(0, 1, 0)), Sample(pi + vm::Vector3i(1, 1, 0)));
		const auto d01 = vm::Lerp(d.x, Sample(pi + vm::Vector3i(0, 0, 1)), Sample(pi + vm::Vector3i(1, 0, 1)));
		const auto d11 = vm::Lerp(d.x, Sample(pi + vm::Vector3i(0, 1, 1)), Sample(pi + vm::Vector3i(1, 1, 1)));
		const auto d0 = vm::Lerp(d.y, d00, d10);
		const auto d1 = vm::Lerp(d.y, d01, d11);
		return vm::Lerp(d.z, d0, d1);
	}

	T operator()(int x, int y, int z)const
	{
		return data[z * size.y * size.x + y * size.x + x];
	}

	T* Get()
	{
		return data;
	}

	vm::Size3 GetSize()const
	{
		return size;
	}
};

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


template<typename T>
struct MethodBase
{
	using ValueType = T;
};


template<typename T>
struct InterpolationBase :MethodBase<T> {};

template<typename T>
struct Interpolation :InterpolationBase<T>
{
	static constexpr int SliceCount = 10;
	static Point3f Trans(const Point3i& sampledGridGlobalPos, const Vec3i& batchSize, const Vec3i& batch3DID, const Vec3f& step)
	{
		return (Vec3f(sampledGridGlobalPos.ToVector3()) * step - Vec3f(batchSize * batch3DID)).ToPoint3();
	}
};

template<typename T>
struct KernelBase :MethodBase<T> {};

template<typename valType, int FilterSize>
struct Kernel :KernelBase<valType>
{
	static constexpr int KernelSize = FilterSize;
	static Point3f Trans(const Point3i& sampledGridGlobalPos, const Vec3i& batchSize, const Vec3i& batch3DID)
	{
		const auto halfSize = 1.0f * KernelSize / 2;
		return (Vec3f(sampledGridGlobalPos.x * KernelSize + halfSize, sampledGridGlobalPos.y * KernelSize + halfSize, sampledGridGlobalPos.z * KernelSize + halfSize)
			- Vec3f(Vec3i{batchSize.x,batchSize.y,batchSize.z-1} *batch3DID)).ToPoint3();
	}
};

template<typename ValueType>
struct MaxKernel :Kernel<ValueType, 2>
{
	static constexpr Vec3i offset[27] = {
		{-1,-1,-1},
		{-1,-1,0},
		{-1,-1,1},
		{-1,0,-1},
		{-1,0,0},
		{-1,0,1},
	    {-1,1,-1},
		{-1,1,0},
		{-1,1,1},
		{0,-1,-1},
		{0,-1,0},
		{0,-1,1},
		{0,0,-1},
		{0,0,0},
		{0,0,1},
		{0,1,-1},
		{0,1,0},
		{0,1,1},
		{1,-1,-1},
		{1,-1,0},
		{1,-1,1},
		{1,0,-1},
		{1,0,0},
		{1,0,1},
		{1,1,-1},
		{1,1,0},
		{1,1,1},
	};

	static ValueType Sample(const Point3i& sampledGridGlobalPos,
		const Point3f& sampledGridLocalPos,
		const ValueType* src,
		const Vec3i& batchSize)
	{
		const auto pos = Point3i(sampledGridLocalPos);
		ValueType val = std::numeric_limits<ValueType>::lowest();
		for(int i = 0 ; i< 27;i++)
		{
			val = std::max(val, *(src + Linear(pos + offset[i], Size2(batchSize.x, batchSize.y))));
		}
		return val;
	}
};

template<typename ValueType>
struct TrivialKernel :Kernel<ValueType, 2>
{
	static ValueType Sample(const Point3i& sampledGridGlobalPos,
		const Point3f& sampledGridLocalPos,
		const ValueType* src,
		const Vec3i& batchSize)
	{
		const auto pos = Point3i(sampledGridLocalPos);
		return *(src + Linear(pos,Size2(batchSize.x,batchSize.y)));
	}
	
};

template<typename ValueType>
struct GaussianKernel :Kernel<ValueType, 2>
{
	static ValueType Sample(const Point3i& sampledGridGlobalPos,
		const Point3f& sampledGridLocalPos,
		const ValueType* src,
		const Vec3i& batchSize)
	{
		return ValueType();
	}
};

template<typename ValueType>
struct TriLinear :Interpolation<ValueType>
{
	static ValueType Sample(const Point3i& sampledGridGlobalPos,
		const Point3f& sampledGridLocalPos,
		const ValueType* src,
		const Vec3i& batchSize)
	{
		return Sampler3D<ValueType>(src, Size3(batchSize)).Sample(sampledGridLocalPos);
	}
};

template<typename InterpolationFilterType>
struct InterpolationBasedSampler
{
	using ItplBasedFilter = InterpolationFilterType;
	using ValueType = typename InterpolationFilterType::ValueType;
	void Downsample(const Size3& size,
		const std::string& fileName, const Vec3f& scale, const std::string& outFileName)
	{
		println("Interpolation based");

		const Vec3i batchCount( 1,1,RoundUpDivide(size.z,ItplBasedFilter::SliceCount) );

		const Vec3i batchSize(size.x, size.y, ItplBasedFilter::SliceCount);

		RawReader reader(fileName, size, sizeof(ValueType));
		std::unique_ptr<ValueType[]> batchBuf(new ValueType[batchSize.Prod()]);
		std::ofstream out(outFileName, std::ios::binary);



		const Size3 sampledCount(1.0 * size.x / scale.x + 0.5, 1.0 * size.y / scale.y + 0.5, 1.0 * size.z / scale.z + 0.5);
		const Vector3f step(1.0 * size.x / sampledCount.x, 1.0 * size.y / sampledCount.y, 1.0 * size.z / sampledCount.z);

		const Size3 maxSampledVolume(sampledCount.x, sampledCount.y, std::ceil(ItplBasedFilter::SliceCount * 1.0 / step.z));
		std::unique_ptr<ValueType[]> sampledBuf(new ValueType[maxSampledVolume.Prod()]);

		println("BatchSize: {}, sampled Volume Size: {}, Filter Size: {}, Data Size: {}", batchSize, maxSampledVolume, ItplBasedFilter::SliceCount, size);
		int sumSampledZCount = 0;
		Size3 sampledVolume(sampledCount.x, sampledCount.y, 0);

		for (int bz = 0; bz < batchCount.z; bz++)
		{
			int slice = ItplBasedFilter::SliceCount;
			if (bz == batchCount.z - 1 && size.z % ItplBasedFilter::SliceCount)
			{
				slice = size.z % ItplBasedFilter::SliceCount;
			}

			const auto batchStart = Vec3i(1, 1, bz) * Vec3i(batchSize);

			const Vec3i curBatchSize(batchSize.x, batchSize.y, slice);

			reader.readRegion(batchStart, Size3(curBatchSize), batchBuf.get());

			const int sampledSliceCount = 1.0 * (bz * batchSize.z + curBatchSize.z) / step.z;
			const auto curSampledSliceCount = sampledSliceCount - sumSampledZCount;


			const Size3 curSampledVolume(sampledCount.x, sampledCount.y, curSampledSliceCount);

			println("current batch size: {}, current batch sampled count: {}", curBatchSize, curSampledVolume);


			for (int z = 0; z < curSampledVolume.z; z++)
			{
				for (int y = 0; y < curSampledVolume.y; y++)
				{
					for (int x = 0; x < curSampledVolume.x; x++)
					{
						const Point3i globalPos{ x, y,sumSampledZCount + z };
						const Point3f localPos = ItplBasedFilter::Trans(globalPos, batchSize, Vec3i{ 1,1,bz }, step);
						*(sampledBuf.get() + Linear({ x,y,z }, { curSampledVolume.x,curSampledVolume.y })) = ItplBasedFilter::Sample(globalPos, localPos, batchBuf.get(), curBatchSize);
					}
				}
			}
			sumSampledZCount = sampledSliceCount;
			sampledVolume.z += curSampledVolume.z;
			out.write((char*)sampledBuf.get(), sizeof(ValueType) * curSampledVolume.Prod());
		}
		out.close();
	}
};


template<typename KernelBasedFilterType>
struct KernelBasedSampler
{
	using KernelBasedFilter = KernelBasedFilterType;
	using ValueType = typename KernelBasedFilter::ValueType;

	void Downsample(const Size3& size,
		const std::string& fileName, const Vec3f& scale, const std::string& outFileName)const
	{
		//static_assert(KernelBasedFilter::KernelSize >= 2);

		println("Kernel Based, KernelSize:{}", KernelBasedFilter::KernelSize);

		const Vec3i batchCount(1, 1, RoundUpDivide(size.z-1, KernelBasedFilter::KernelSize));

		const Vec3i batchSize(size.x, size.y, KernelBasedFilter::KernelSize + 1);

		const Size2 plane(RoundUpDivide(batchSize.x - 1, KernelBasedFilter::KernelSize), RoundUpDivide(batchSize.y - 1, KernelBasedFilter::KernelSize));

		println("batch count:{}, BatchSize: {}, SampledPlaneSize: {}, Filter Size: {}, Data Size: {}",batchCount, batchSize, plane, KernelBasedFilter::KernelSize, size);

		RawReader reader(fileName, size, sizeof(ValueType));
		std::unique_ptr<ValueType[]> batchBuf(new ValueType[batchSize.Prod()]);

		std::ofstream out(outFileName, std::ios::binary);
		std::unique_ptr<ValueType[]> sampledBuf(new ValueType[plane.Prod()]);

		if (out.is_open() == false)
		{
			throw runtime_error("Failed to open file");
		}

		for (int bz = 0; bz < batchCount.z; bz++)
		{
			const auto batchStart = Vec3i(0, 0, bz) * Vec3i(batchSize);

			Vec3i curBatchSize(batchSize.x, batchSize.y, KernelBasedFilter::KernelSize + 1);

			if (bz == batchCount.z - 1 && size.z % (KernelBasedFilter::KernelSize + 1) )
			{
				curBatchSize.z = size.z % (KernelBasedFilter::KernelSize+ 1);
			}

			reader.readRegion(batchStart, Size3(curBatchSize), batchBuf.get());

			println("batch id:{}, batch start: {},current batch size: {}", bz, batchStart, curBatchSize);

			for (int y = 0; y < plane.y; y++)
			{
				for (int x = 0; x < plane.x; x++)
				{
					const Point3i globalPos{ x,y,bz };
					const Point3f localPos = KernelBasedFilterType::Trans(globalPos, curBatchSize, Vec3i{ 0,0,bz });
					*(sampledBuf.get() + Linear({ x,y }, plane.x)) = KernelBasedFilter::Sample(globalPos, localPos, batchBuf.get(), curBatchSize);
				}
			}
			out.write((char *)sampledBuf.get(), sizeof(ValueType) * plane.Prod());
		}
		out.close();
	}
};


template<typename FilterType, typename Enable = void> struct Sampler;

// specialization for kernel-based down sampling
template<typename FilterType> struct Sampler< FilterType, typename std::enable_if<std::is_base_of<KernelBase<typename FilterType::ValueType>, FilterType>::value>::type> :KernelBasedSampler<FilterType> {};

// specialization for interpolation-based down sampling
template<typename FilterType> struct Sampler< FilterType, typename std::enable_if<std::is_base_of<InterpolationBase<typename FilterType::ValueType>, FilterType>::value>::type> :InterpolationBasedSampler<FilterType> {};


int main()
{

 
	//Sampler<TrivialKernel<unsigned char>>().Downsample({480,720,120},R"(C:\tmp\mixfrac.raw)",{},R"(C:\tmp\kds.raw)");
	Sampler<TriLinear<unsigned char>>().Downsample({480,720,120},R"(C:\tmp\mixfrac.raw)",{2.f,2.f,2.f},R"(C:\tmp\kds.raw)");

	//Sampler<TriLinear<unsigned char>> b;
	//Sampler<GaussianKernel<unsigned char>> c;

	//a.Downsample({}, "", {}, "");
	//b.Downsample({}, "", {}, "");
	//c.Downsample({}, "", {}, "");

	return 0;
}


//int main(int argc, char** argv)
//{
	//using namespace std;
	//cmdline::parser a;


	//a.add<string>("if", 'i', "input file name", true);

	//a.add<string>("of", 'o', "out file name", false, "a.out");

	//a.add<size_t>("offset", 0, "offset from beginning of the file", false, 0);

	//a.add<size_t>("width", 'x', "width of the 3d data", true, 0);

	//a.add<size_t>("height", 'y', "height of the 3d data", true, 0);

	//a.add<size_t>("depth", 'z', "depth of the 3d data", true, 0);

	//a.add<float>("sx", 0, "scale factor for the x-axis", true, 2);

	//a.add<float>("sy", 0, "scale factor for the y-axis", true, 2);

	//a.add<float>("sz", 0, "scale factor for the z-axis", true, 2);

	//a.add<string>("type", 0, "element type of the 3d data", false, "char", cmdline::oneof("char", "short", "int", "float", "double"));

	//a.parse_check(argc, argv);


	//std::size_t x, y, z;
	//float sx, sy, sz;
	//std::string inFileName, outFileName;
	//std::size_t offset;

	//x = a.get<size_t>("width");
	//y = a.get<size_t>("height");
	//z = a.get<size_t>("depth");

	//inFileName = a.get<string>("if");
	//outFileName = a.get<string>("of");

	//sx = a.get<float>("sx");
	//sy = a.get<float>("sy");
	//sz = a.get<float>("sz");

	//offset = a.get<size_t>("offset");
	//string type = a.get<string>("type");


//}
