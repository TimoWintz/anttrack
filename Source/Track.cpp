#include "Track.hpp"

namespace TrackSim {
    void Track::load(Trade::AbstractImporter& importer) {
        loadScene(importer, _meshes, this, _shader, _drawables);
        loadCollider(importer, _colliderMesh, _triangleIndexVertexArray, _bWorld, this);
            
    }
}
