// ChunkStreamSystem<T> template'inin physics.dll'e ait somut
// instantiation'ları burada bir kez derlenip DLL'e gömülür (bkz.
// TerrainDeformSystem.h / WreckPersistence.h'deki "extern template"
// bildirimleri). Tüketici çeviri birimleri bu kodu yeniden üretmez.
//
// DestructibleChunkData physics_ext.dll'e ait olduğu için onun instantiation'ı
// burada değil, src/destructible/DestructibleChunkData.cpp içindedir.

#include "streaming/ChunkStreamSystem.h"

#include "terrain_deform/TerrainDeformSystem.h"
#include "vehicle/WreckPersistence.h"

namespace alazforge {

template class PHYSICS_API ChunkStreamSystem<DeformationChunkData>;
template class PHYSICS_API ChunkStreamSystem<WreckChunkData>;

} // namespace alazforge
