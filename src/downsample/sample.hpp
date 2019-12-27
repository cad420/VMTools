
#pragma once

#include <VMat/numeric.h>
#include <VMUtils/log.hpp>
#include <VMUtils/cmdline.hpp>
#include <VMFoundation/rawreader.h>


#include <string>
#include <fstream>
#include <exception>


namespace vm {

	template<typename T>
	class Interpolator
	{
		const T* const data;
		const Vec3i size;
	public:
		Interpolator(const T* data, const Size3 size) :data(data), size(size)
		{

		}

		Float Sample(const vm::Point3i& p)
		{
			Bound3i bound(vm::Point3i(0, 0, 0), vm::Point3i(size.x, size.y, size.z));
			if (!bound.InsideEx(p))
				return 0;
			return (*this)(p.x, p.y, p.z);
		}

		Float Sample(const Point3f& p)
		{
			const auto pi = Point3i(std::floor(p.x), std::floor(p.y), std::floor(p.z));
			const auto d = p - static_cast<vm::Point3f>(pi);
			const auto d00 = Lerp(d.x, Sample(pi), Sample(pi + vm::Vector3i(1, 0, 0)));
			const auto d10 = Lerp(d.x, Sample(pi + Vector3i(0, 1, 0)), Sample(pi + Vector3i(1, 1, 0)));
			const auto d01 = Lerp(d.x, Sample(pi + Vector3i(0, 0, 1)), Sample(pi + Vector3i(1, 0, 1)));
			const auto d11 = Lerp(d.x, Sample(pi + Vector3i(0, 1, 1)), Sample(pi + Vector3i(1, 1, 1)));
			const auto d0 = Lerp(d.y, d00, d10);
			const auto d1 = Lerp(d.y, d01, d11);
			return Lerp(d.z, d0, d1);
		}

		T operator()(int x, int y, int z)const
		{
			return data[z * size.y * size.x + y * size.x + x];
		}

		T* Get()
		{
			return data;
		}

		Vec3i GetSize()const
		{
			return size;
		}
	};


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
			return (Vec3f(sampledGridGlobalPos.x * KernelSize + halfSize,
				sampledGridGlobalPos.y * KernelSize + halfSize,
				sampledGridGlobalPos.z * KernelSize + halfSize)
				- Vec3f(Vec3i{ batchSize.x - 1,batchSize.y - 1,batchSize.z - 1 } *batch3DID))
				.ToPoint3();
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
			for (int i = 0; i < 27; i++)
			{
				val = std::max(val, *(src + Linear(pos + offset[i], Size2(batchSize.x, batchSize.y))));
			}
			return val;
		}
	};

	template<typename ValueType>
	struct Trivial2x2Kernel :Kernel<ValueType, 2>
	{
		static ValueType Sample(const Point3i& sampledGridGlobalPos,
			const Point3f& sampledGridLocalPos,
			const ValueType* src,
			const Vec3i& batchSize)
		{
			const auto pos = Point3i(sampledGridLocalPos);
			return *(src + Linear(pos, Size2(batchSize.x, batchSize.y)));
		}
	};

	template<typename ValueType>
	struct Gaussian2x2Kernel :Kernel<ValueType, 2>
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
	struct Mean2x2Kernel :Kernel<ValueType, 2>
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
			double v = 0;
			for (int i = 0; i < 27; i++)
			{
				v += *(src + Linear(pos + offset[i], Size2(batchSize.x, batchSize.y)));
			}
			return v / 27;
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
			return Interpolator<ValueType>(src, Size3(batchSize)).Sample(sampledGridLocalPos);
		}
	};

	template<typename ValueType>
	struct TriLinearInMemory :Interpolation<ValueType>
	{};


	template<typename InterpolationFilterType>
	struct InterpolationBasedSampler
	{
		using IplBasedFilter = InterpolationFilterType;
		using ValueType = typename InterpolationFilterType::ValueType;

		static std::pair<Vec3i, Vec3i> GetSubSampledVolumeCount(const Vec3i& batch3DID, const Vec3i& maxBatchSize, const Vec3i& curBatchSize, const Vec3f& step)
		{
			const auto curTotalBatch = batch3DID * maxBatchSize + curBatchSize;
			const Vec3i curTotalSampledCount((1.0 * curTotalBatch.x / step.x + 0.5), (1.0 * curTotalBatch.y / step.y + 0.5), (1.0 * curTotalBatch.z / step.z + 0.5));
			const auto prevTotalBatch = (batch3DID)*maxBatchSize;
			Vec3i prevTotalSampledCount((1.0 * prevTotalBatch.x / step.x + 0.5), (1.0 * prevTotalBatch.y / step.y + 0.5), (1.0 * prevTotalBatch.z / step.z + 0.5));
			if (batch3DID.x == 0)
			{
				prevTotalSampledCount.x = 0;// = curBatchSize.x * 1.0 / step.x;
			}
			if (batch3DID.y == 0)
			{
				prevTotalSampledCount.y = 0;// curBatchSize.y * 1.0 / step.y;
			}
			if (batch3DID.z == 0)
			{
				prevTotalSampledCount.z = 0; //curBatchSize.z * 1.0 / step.z;
			}

			return { prevTotalSampledCount,curTotalSampledCount };
		}

		static void Resample(const Size3& dataSize,
			const std::string& fileName, const Vec3f& scale, const std::string& outFileName)
		{
			const Vec3i batchCount(1, 1, RoundUpDivide(dataSize.z, IplBasedFilter::SliceCount));

			const Vec3i maxBatchSize(dataSize.x, dataSize.y, IplBasedFilter::SliceCount);

			RawReader reader(fileName, dataSize, sizeof(ValueType));
			std::unique_ptr<ValueType[]> batchBuf(new ValueType[maxBatchSize.Prod()]);
			std::ofstream out(outFileName, std::ios::binary);


			const Size3 sampledCount(1.0 * dataSize.x / scale.x + 0.5, 1.0 * dataSize.y / scale.y + 0.5, 1.0 * dataSize.z / scale.z + 0.5);
			const Vector3f step(1.0 * dataSize.x / sampledCount.x, 1.0 * dataSize.y / sampledCount.y, 1.0 * dataSize.z / sampledCount.z);

			const Size3 subSampledVolume(sampledCount.x, sampledCount.y, std::ceil(IplBasedFilter::SliceCount * 1.0 / step.z));
			std::unique_ptr<ValueType[]> sampledBuf(new ValueType[subSampledVolume.Prod()]);

			println("data size: {}\nbatch size: {}\n sampled Volume Size: {}", maxBatchSize, subSampledVolume, sampledCount);

#ifdef _DEBUG
			size_t debugCount = 0;
#endif

			for (int bz = 0; bz < batchCount.z; bz++)
			{
				Vec3i curBatchSize(maxBatchSize);

				if (bz == batchCount.z - 1 && dataSize.z % IplBasedFilter::SliceCount)
				{
					curBatchSize.z = dataSize.z % IplBasedFilter::SliceCount;
				}

				const auto batch3DID = Vec3i(0, 0, bz);
				const auto batchStart = batch3DID * Vec3i(maxBatchSize);
				reader.readRegion(batchStart, Size3(curBatchSize), reinterpret_cast<unsigned char*>(batchBuf.get()));

				const auto t = GetSubSampledVolumeCount(batch3DID, maxBatchSize, curBatchSize, step);
				//println("{},{}", t.first, t.second);

				const auto& curSampledVolume = t.second - t.first;
				const auto& prevTotalSampledCount = t.first;


				//std::get<0>(t);
#ifdef _DEBUG
				debugCount += Size3(curSampledVolume).Prod();
#endif
				println("current batch id: {}, batch size: {}, sampled count: {}", batch3DID, curBatchSize, curSampledVolume);

				// for each batch
				for (int z = 0; z < curSampledVolume.z; z++)
				{
					for (int y = 0; y < curSampledVolume.y; y++)
					{
						for (int x = 0; x < curSampledVolume.x; x++)
						{
							const auto globalPos = prevTotalSampledCount + Point3i(x, y, z);
							const Point3f localPos = IplBasedFilter::Trans(globalPos, maxBatchSize, batch3DID, step);
							*(sampledBuf.get() + Linear({ x,y,z }, Size2(curSampledVolume.x, curSampledVolume.y))) = IplBasedFilter::Sample(globalPos, localPos, batchBuf.get(), curBatchSize);
						}
					}
				}

				out.write((char*)sampledBuf.get(), sizeof(ValueType) * curSampledVolume.Prod());
			}
			assert(debugCount == sampledCount.Prod());
			out.close();
		}
	};


	template<typename KernelBasedFilterType>
	struct KernelBasedSampler
	{
		using KernelBasedFilter = KernelBasedFilterType;
		using ValueType = typename KernelBasedFilter::ValueType;

		static void Resample(const Size3& dataSize,
			const std::string& fileName, const Vec3f& scale, const std::string& outFileName)
		{
			//static_assert(KernelBasedFilter::KernelSize >= 2);
			const Vec3i kernelSize{ KernelBasedFilter::KernelSize,KernelBasedFilter::KernelSize,KernelBasedFilter::KernelSize };
			const Vec3i batchCount(1, 1, RoundUpDivide(dataSize.z - 1, KernelBasedFilter::KernelSize));
			const Vec3i maxBatchSize(dataSize.x, dataSize.y, KernelBasedFilter::KernelSize + 1);

			const Size2 subSampledPlane(RoundUpDivide(maxBatchSize.x - 1, kernelSize.x), RoundUpDivide(maxBatchSize.y - 1, kernelSize.y));

			const Vec3i sampledDataSize(subSampledPlane.x, subSampledPlane.y, RoundUpDivide(dataSize.z - 1, kernelSize.z));

			RawReader reader(fileName, dataSize, sizeof(ValueType));
			std::unique_ptr<ValueType[]> batchBuf(new ValueType[maxBatchSize.Prod()]);

			println("Kernel-Based Down sampling");
			println("batch count:{}\nbatch size: {}\nsub-sampled size: {}\nkernel Size: {}\ndata Size: {}", batchCount, maxBatchSize, subSampledPlane, kernelSize, dataSize);
			println("sampled data size: {}", sampledDataSize);

			std::ofstream out(outFileName, std::ios::binary);
			std::unique_ptr<ValueType[]> sampledBuf(new ValueType[subSampledPlane.Prod()]);

			if (out.is_open() == false)
			{
				println("Failed to open file {}", outFileName);
				throw std::runtime_error("Failed to open file");
			}

			for (int bz = 0; bz < batchCount.z; bz++)
			{

				const auto batch3DID = Vec3i(0, 0, bz);
				const auto batchStart = batch3DID * Vec3i(maxBatchSize);

				Vec3i curBatchSize(maxBatchSize.x, maxBatchSize.y, KernelBasedFilter::KernelSize + 1);

				if (bz == batchCount.z - 1 && dataSize.z % (KernelBasedFilter::KernelSize + 1))
				{
					curBatchSize.z = dataSize.z % (KernelBasedFilter::KernelSize + 1);
				}

				reader.readRegion(batchStart, Size3(curBatchSize), reinterpret_cast<unsigned char*>(batchBuf.get()));

				println("current batch id:{}, batch start: {},current batch size: {}", bz, batchStart, curBatchSize);

				for (int y = 0; y < subSampledPlane.y; y++)
				{
					for (int x = 0; x < subSampledPlane.x; x++)
					{
						const Point3i globalPos{ x,y,bz };
						const Point3f localPos = KernelBasedFilterType::Trans(globalPos, maxBatchSize, batch3DID);
						*(sampledBuf.get() + Linear({ x,y }, subSampledPlane.x)) = KernelBasedFilter::Sample(globalPos, localPos, batchBuf.get(), curBatchSize);
					}
				}

				out.write((char*)sampledBuf.get(), sizeof(ValueType) * subSampledPlane.Prod());
			}
			out.close();
		}
	};



	template<typename FilterType, typename Enable = void>
	struct Sampler;

	// specialization for kernel-based down sampling
	template<typename FilterType>
	struct Sampler< FilterType, typename std::enable_if<std::is_base_of<KernelBase<typename FilterType::ValueType>, FilterType>::value>::type> :KernelBasedSampler<FilterType>
	{

	};

	// specialization for interpolation-based down sampling
	template<typename FilterType>
	struct Sampler< FilterType, typename std::enable_if<std::is_base_of<InterpolationBase<typename FilterType::ValueType>, FilterType>::value>::type> :InterpolationBasedSampler<FilterType>
	{

	};

}

