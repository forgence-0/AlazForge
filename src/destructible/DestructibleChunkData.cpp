// ChunkStreamSystem<DestructibleChunkData> instantiation'ı burada bir kez
// derlenip physics_ext.dll'e gömülür (bkz. DestructibleChunkData.h'deki
// "extern template" bildirimi). DeformationChunkData/WreckChunkData
// instantiation'ları physics.dll'de (streaming/ChunkStreamSystem.cpp)
// yaşıyor — bu tip yalnızca physics_ext'e ait olduğu için ayrı tutuldu.

#include "destructible/DestructibleChunkData.h"

namespace alazforge {

template class PHYSICS_EXT_API ChunkStreamSystem<DestructibleChunkData>;

} // namespace alazforge
