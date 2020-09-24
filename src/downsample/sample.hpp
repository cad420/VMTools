
#pragma once

#include <VMat/numeric.h>
#include <VMUtils/log.hpp>
#include <VMUtils/cmdline.hpp>
#include <VMFoundation/rawreader.h>

#include <VMUtils/threadpool.hpp>
#include <VMUtils/concurrency.hpp>

#include <string>
#include <fstream>
#include <exception>


namespace vm {

	template<typename ValueType>
	struct SampledBatch
	{
		size_t offset;
		size_t size;
		std::shared_ptr<ValueType[]> Buffer;
		SampledBatch(size_t offset, size_t size, std::shared_ptr<ValueType[]> buf) :
			offset(offset),
			size(size),
			Buffer(std::move(buf))
		{}
		//SampledBatch(SampledBatch&&) = default;
		//SampledBatch& operator=(SampledBatch&&) = default;
	};

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


	static Vec3i offset2x2[] =
	{
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
			constexpr auto halfSize = 1.0f * KernelSize / 2;
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

		static ValueType Sample(const Point3i& sampledGridGlobalPos,
			const Point3f& sampledGridLocalPos,
			const ValueType* src,
			const Vec3i& batchSize)
		{
			const auto pos = Point3i(sampledGridLocalPos);
			ValueType val = std::numeric_limits<ValueType>::lowest();
			for (int i = 0; i < 27; i++)
			{
				val = std::max(val, *(src + Linear(pos + offset2x2[i], Size2(batchSize.x, batchSize.y))));
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
		static ValueType Sample(const Point3i& sampledGridGlobalPos,
			const Point3f& sampledGridLocalPos,
			const ValueType* src,
			const Vec3i& batchSize)
		{
			const auto pos = Point3i(sampledGridLocalPos);
			double v = 0;
			for (int i = 0; i < 27; i++)
			{
				v += *(src + Linear(pos + offset2x2[i], Size2(batchSize.x, batchSize.y)));
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
			const std::string& fileName, const Vec3f& scale, const std::string& outFileName, int nthreadreading, int nthreadwrting)
		{


			
			const Vec3i batchCount(1, 1, RoundUpDivide(dataSize.z, IplBasedFilter::SliceCount));

			const Vec3i maxBatchSize(dataSize.x, dataSize.y, IplBasedFilter::SliceCount);

			//RawReader reader(fileName, dataSize, sizeof(ValueType));
			
			//std::unique_ptr<ValueType[]> batchBuf(new ValueType[maxBatchSize.Prod()]);
			//std::ofstream out(outFileName, std::ios::binary);

			const Size3 sampledCount(1.0 * dataSize.x / scale.x + 0.5, 1.0 * dataSize.y / scale.y + 0.5, 1.0 * dataSize.z / scale.z + 0.5);
			const Vector3f step(1.0 * dataSize.x / sampledCount.x, 1.0 * dataSize.y / sampledCount.y, 1.0 * dataSize.z / sampledCount.z);

			const Size3 subSampledVolume(sampledCount.x, sampledCount.y, std::ceil(IplBasedFilter::SliceCount * 1.0 / step.z));
			//std::unique_ptr<ValueType[]> sampledBuf(new ValueType[subSampledVolume.Prod()]);

			println("data size: {}\nbatch size: {}\n sampled Volume Size: {}", maxBatchSize, subSampledVolume, sampledCount);



			ThreadPool read(nthreadreading);
			ThreadPool write(nthreadwrting);
			auto que = std::make_shared<BlockingQueue<SampledBatch<ValueType>>>(std::min(nthreadreading, nthreadwrting));
			std::shared_ptr<std::mutex> readerMux(new std::mutex()), writerMux(new std::mutex());

			auto reader = std::make_shared<RawReader>(fileName, dataSize, sizeof(ValueType));
			auto writer = std::make_shared<std::ofstream>(outFileName, std::ios::binary);
			

#ifdef _DEBUG
			std::shared_ptr<std::atomic_int64_t> debugCount(new std::atomic_int64_t(0));
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


#ifdef _DEBUG
				auto producerTask = [que,reader,maxBatchSize,subSampledVolume,readerMux,debugCount,step](Vec3i batchStart,Vec3i  curBatchSize,Vec3i batch3DID,int bz)
#else
				auto producerTask = [que,reader,maxBatchSize,subSampledVolume,readerMux,step](Vec3i batchStart,Vec3i  curBatchSize,Vec3i batch3DID,int bz)
#endif
				{

					std::shared_ptr<ValueType[]> batchBuf(new ValueType[maxBatchSize.Prod()]);
					std::shared_ptr<ValueType[]> sampledBuf(new ValueType[subSampledVolume.Prod()]);
					
					std::unique_lock<std::mutex> lk(*readerMux);
					reader->readRegion(batchStart, Size3(curBatchSize), reinterpret_cast<unsigned char*>(batchBuf.get()));
					lk.unlock();

					const auto t = GetSubSampledVolumeCount(batch3DID, maxBatchSize, curBatchSize, step);
					//println("{},{}", t.first, t.second);

					const auto& curSampledVolume = t.second - t.first;
					const auto& prevTotalSampledCount = t.first;

					//std::get<0>(t);
#ifdef _DEBUG
					debugCount->fetch_add( Size3(curSampledVolume).Prod());
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

					const size_t size = curSampledVolume.Prod() * sizeof(ValueType);
					const size_t offset = prevTotalSampledCount.z * subSampledVolume.x * subSampledVolume.y * sizeof(ValueType);


					que->Put(SampledBatch<ValueType>(
						offset,size,sampledBuf
						));
				};


				auto consumerTask = [que,writer,writerMux]()
				{
					auto t = que->Take();

					std::unique_lock<std::mutex> lk(*writerMux);
					writer->seekp(t.offset, std::ios::beg);
					writer->write((char*)t.Buffer.get(), t.size);
				};

				read.AppendTask(producerTask, batchStart, curBatchSize, batch3DID,bz);
				write.AppendTask(consumerTask);

				

			}

			read.Wait();
			write.Wait();

#ifdef _DEBUG
			assert(debugCount->load() ==  sampledCount.Prod());
#endif

			writer->close();
			
		}
	};


	template<typename KernelBasedFilterType>
	struct KernelBasedSampler
	{
		using KernelBasedFilter = KernelBasedFilterType;
		using ValueType = typename KernelBasedFilter::ValueType;

		static void Resample(const Size3& dataSize,
			const std::string& fileName, const Vec3f& scale, const std::string& outFileName,int nthreadreading,int nthreadwrting)
		{



			ThreadPool read(nthreadreading);
			ThreadPool write(nthreadwrting);
			auto que = std::make_shared<BlockingQueue<SampledBatch<ValueType>>>(std::min(nthreadreading,nthreadwrting));
			
			std::shared_ptr<std::mutex> readerMux(new std::mutex()), writerMux(new std::mutex());
			
			
			
			//static_assert(KernelBasedFilter::KernelSize >= 2);
			const Vec3i kernelSize{ KernelBasedFilter::KernelSize,KernelBasedFilter::KernelSize,KernelBasedFilter::KernelSize };
			const Vec3i batchCount(1, 1, RoundUpDivide(dataSize.z - 1, KernelBasedFilter::KernelSize));
			const Vec3i maxBatchSize(dataSize.x, dataSize.y, KernelBasedFilter::KernelSize + 1);

			const Size2 subSampledPlane(RoundUpDivide(maxBatchSize.x - 1, kernelSize.x), RoundUpDivide(maxBatchSize.y - 1, kernelSize.y));

			const Vec3i sampledDataSize(subSampledPlane.x, subSampledPlane.y, RoundUpDivide(dataSize.z - 1, kernelSize.z));

			//RawReader reader(fileName, dataSize, sizeof(ValueType));

			auto reader = std::make_shared<RawReader>(fileName, dataSize, sizeof(ValueType));
			
			

			println("Kernel-Based Down sampling");
			println("batch count:{}\nbatch size: {}\nsub-sampled size: {}\nkernel Size: {}\ndata Size: {}", batchCount, maxBatchSize, subSampledPlane, kernelSize, dataSize);
			println("sampled data size: {}", sampledDataSize);


			auto writer = std::make_shared<std::ofstream>(outFileName,std::ios::binary);
			//std::ofstream out(outFileName, std::ios::binary);
			//

			if (writer->is_open() == false)
			{
				println("Failed to open file {}", outFileName);
				throw std::runtime_error("Failed to open file");
			}
			

			for (int bz = 0; bz < batchCount.z; bz++)
			{

				const auto batch3DID = Vec3i(0, 0, bz);
				const auto batchStart = batch3DID * Vec3i(maxBatchSize.x, maxBatchSize.y, maxBatchSize.z - 1);
				Vec3i curBatchSize(maxBatchSize.x, maxBatchSize.y, KernelBasedFilter::KernelSize + 1);
				if (bz == batchCount.z - 1 && dataSize.z % (KernelBasedFilter::KernelSize + 1))
				{
					curBatchSize.z = dataSize.z % (KernelBasedFilter::KernelSize + 1);
				}

				auto producerTask = [subSampledPlane,maxBatchSize,que,reader,readerMux](const Vec3i &batch3DID,const Vec3i & batchStart,const Vec3i & curBatchSize,int bz)
				{

					const size_t size = subSampledPlane.Prod() * sizeof(ValueType);
					const size_t offset = bz * size;
					
					std::shared_ptr<ValueType[]> batchBuf(new ValueType[maxBatchSize.Prod()]);

					std::unique_lock<std::mutex> lk(*readerMux);
					reader->readRegion(batchStart, Size3(curBatchSize), reinterpret_cast<unsigned char*>(batchBuf.get()));
					lk.unlock();

					println("current batch id:{}, batch start: {},current batch size: {}, subsampleplane: {}", bz, batchStart, curBatchSize, subSampledPlane);
					std::shared_ptr<ValueType[]> sampledBuf(new ValueType[subSampledPlane.Prod()]);

					for (int y = 0; y < subSampledPlane.y; y++)
					{
						for (int x = 0; x < subSampledPlane.x; x++)
						{
							const Point3i globalPos{ x,y,bz };
							const Point3f localPos = KernelBasedFilterType::Trans(globalPos, maxBatchSize, batch3DID);
							*(sampledBuf.get() + Linear({ x,y }, subSampledPlane.x)) = KernelBasedFilter::Sample(globalPos, localPos, batchBuf.get(), curBatchSize);
						}
					}

					que->Put(std::move(SampledBatch(
						offset,
						size,
						sampledBuf
					)));
				};

				auto consumerTask = [que, writer, writerMux]()
				{
					auto t = que->Take();
					
					std::unique_lock<std::mutex> lk(*writerMux);
					writer->seekp(t.offset, std::ios::beg);
					writer->write((char*)t.Buffer.get(), t.size);
					//lk.unlock();
				};

				read.AppendTask(producerTask,batch3DID,batchStart,curBatchSize,bz);
				write.AppendTask(consumerTask);
			}
			
			read.Wait();
			write.Wait();
			writer->close();

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

