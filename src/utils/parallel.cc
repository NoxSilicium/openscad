#include "parallel.h"

#include <exception>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace OpenSCADParallel {

bool openMPEnabled()
{
#ifdef _OPENMP
  return !std::getenv("OPENSCAD_NO_PARALLEL") && !omp_in_parallel() && omp_get_max_threads() > 1;
#else
  return false;
#endif
}

void forEachChunk(size_t count, size_t chunkSize, const std::function<void(size_t, size_t)>& work)
{
  if (count == 0) return;
#ifdef _OPENMP
  if (count > chunkSize && openMPEnabled()) {
    const size_t chunks = 1 + (count - 1) / chunkSize;
    const int workers = static_cast<int>(std::min(chunks, static_cast<size_t>(omp_get_max_threads())));
    std::exception_ptr failure;
#pragma omp parallel for schedule(static) num_threads(workers)
    for (std::ptrdiff_t chunk = 0; chunk < static_cast<std::ptrdiff_t>(chunks); ++chunk) {
      const size_t first = static_cast<size_t>(chunk) * chunkSize;
      try {
        work(first, first + std::min(chunkSize, count - first));
      } catch (...) {
#pragma omp critical(OpenSCADParallelException)
        {
          if (!failure) failure = std::current_exception();
        }
      }
    }
    if (failure) std::rethrow_exception(failure);
    return;
  }
#endif
  for (size_t first = 0, last = 0; first < count; first = last) {
    last = first + std::min(chunkSize, count - first);
    work(first, last);
  }
}

}  // namespace OpenSCADParallel
