// ChunkStreamSystem<T> template'inin somut instantiation'ları burada bir kez
// derlenip DLL'e gömülür (bkz. TerrainDeformSystem.h / WreckPersistence.h'deki
// "extern template" bildirimleri). Tüketici çeviri birimleri bu kodu yeniden
// üretmez.

#include "streaming/ChunkStreamSystem.h"

#include "terrain_deform/TerrainDeformSystem.h"
#include "vehicle/WreckPersistence.h"

namespace alazforge {

template class ALAZFORGE_API ChunkStreamSystem<DeformationChunkData>;
template class ALAZFORGE_API ChunkStreamSystem<WreckChunkData>;

} // namespace alazforge
