// ChunkStreamSystem<DestructibleChunkData> instantiation'ı burada bir kez
// derlenip destructible.dll'e gömülür (bkz. DestructibleChunkData.h'deki
// "extern template" bildirimi). DeformationChunkData/WreckChunkData
// instantiation'ları physics.dll'de (streaming/ChunkStreamSystem.cpp)
// yaşıyor — bu tip yalnızca destructible modülüne ait olduğu için ayrı tutuldu.

#include "destructible/DestructibleChunkData.h"

namespace alazforge {

template class DESTRUCTIBLE_API ChunkStreamSystem<DestructibleChunkData>;

} // namespace alazforge
