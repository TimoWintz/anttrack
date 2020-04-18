#include "Common.hpp"
#include "RigidBody.hpp"


namespace TrackSim {
    using namespace Math::Literals;
    void loadScene(Trade::AbstractImporter& importer, Containers::Array<Containers::Optional<GL::Mesh>>& meshes, Object3D* parent, Shaders::Flat3D& shader, SceneGraph::DrawableGroup3D& drawables) {
        /* Load all materials. Materials that fail to load will be NullOpt. The
         data will be stored directly in objects later, so save them only
         temporarily. */
        Containers::Array<Containers::Optional<Trade::PhongMaterialData>> materials{importer.materialCount()};
        for(UnsignedInt i = 0; i != importer.materialCount(); ++i) {
            Debug{} << "Importing material" << i << importer.materialName(i);

            Containers::Pointer<Trade::AbstractMaterialData> materialData = importer.material(i);
            if(!materialData || materialData->type() != Trade::MaterialType::Phong) {
                Warning{} << "Cannot load material, skipping";
                continue;
            }

            materials[i] = std::move(static_cast<Trade::PhongMaterialData&>(*materialData));
        }

        /* Load all meshes. Meshes that fail to load will be NullOpt. */
        meshes = Containers::Array<Containers::Optional<GL::Mesh>>{importer.mesh3DCount()};
        for(UnsignedInt i = 0; i != importer.mesh3DCount(); ++i) {
            Debug{} << "Importing mesh" << i << importer.mesh3DName(i);

            Containers::Optional<Trade::MeshData3D> meshData = importer.mesh3D(i);
            if(!meshData || !meshData->hasNormals() || meshData->primitive() != MeshPrimitive::Triangles) {
                Warning{} << "Cannot load the mesh, skipping";
                continue;
            }
            Debug{} << "N tris = " << meshData->indices().size();
            Debug{} << "N pts = " << meshData->positions(0).size();

            /* Compile the mesh */
            meshes[i] = MeshTools::compile(*meshData);
        }

        /* Load the scene */
        if(importer.defaultScene() != -1) {
            Debug{} << "Adding default scene" << importer.sceneName(importer.defaultScene());

            Containers::Optional<Trade::SceneData> sceneData = importer.scene(importer.defaultScene());
            if(!sceneData) {
                Error{} << "Cannot load scene, exiting";
                return;
            }

            /* Recursively add all children */
            for(UnsignedInt objectId: sceneData->children3D()) {
                if (importer.object3DName(objectId) == COLLIDER_OBJECT_ID) { // Ignore physics object
                    continue;
                } else {
                    addObject(importer, meshes, materials, parent, objectId, shader, drawables);
                }
            }
               
        }
    }
    void loadCollider(Trade::AbstractImporter& importer, btBvhTriangleMeshShape* colliderMesh, btTriangleIndexVertexArray* triangleIndexVertexArray, btDiscreteDynamicsWorld* bWorld, Object3D* parent) {
        Containers::Optional<Trade::SceneData> sceneData = importer.scene(importer.defaultScene());
        for(UnsignedInt objectId: sceneData->children3D()) {
            Containers::Pointer<Trade::ObjectData3D> objectData = importer.object3D(objectId);
            if (importer.object3DName(objectId) == COLLIDER_OBJECT_ID && objectData->instanceType() == Trade::ObjectInstanceType3D::Mesh) {
                Debug{} << "Found collider object";
                int vertStride = sizeof(btVector3);
                int indexStride = 3*sizeof(int);
                auto meshData = importer.mesh3D(objectData->instance());
                int nIndex = meshData->indices().size() / 3;
                int* indexArray = new int[3*nIndex];
                for (int j = 0; j < 3*nIndex; j++) {
                    indexArray[j] = (int) meshData->indices().at(j);
                }
                int nVerts = meshData->positions(0).size();
                btVector3* vertexArray = new btVector3[nVerts];
                for (int j = 0; j < nVerts; j++) {
                    auto p = meshData->positions(0).at(j);
                    vertexArray[j] = btVector3(p.x(), p.y(), p.z());
                }
                triangleIndexVertexArray = new btTriangleIndexVertexArray(nIndex, &indexArray[0], indexStride, nVerts, (btScalar*) &(vertexArray[0].x()), vertStride);
                Debug{} << "Loaded triangle mesh in Bullet";
                btVector3 aabbMin(-1000,-1000,-1000),aabbMax(1000,1000,1000);
                colliderMesh = new btBvhTriangleMeshShape(triangleIndexVertexArray, true, aabbMin, aabbMax);
                Debug{} << "Created btBvhTriangleMeshShape object";
                new RigidBody(parent, 0.0, colliderMesh, bWorld);
                Debug{} << "Done";
                continue;
            }
        }
    }
    void addObject(Trade::AbstractImporter& importer, Containers::Array<Containers::Optional<GL::Mesh>>& meshes,
        Containers::ArrayView<const Containers::Optional<Trade::PhongMaterialData>> materials, Object3D* parent, UnsignedInt i,  Shaders::Flat3D& shader, SceneGraph::DrawableGroup3D& drawables) {
        Debug{} << "Importing object" << i << importer.object3DName(i);
        Containers::Pointer<Trade::ObjectData3D> objectData = importer.object3D(i);
        if(!objectData) {
            Error{} << "Cannot import object, skipping";
            return;
        }

        /* Add the object to the scene and set its transformation */
        auto* object = new Object3D{parent};
        object->setTransformation(objectData->transformation());

        /* Add a drawable if the object has a mesh and the mesh is loaded */
        if(objectData->instanceType() == Trade::ObjectInstanceType3D::Mesh && objectData->instance() != -1 && meshes[objectData->instance()]) {
            const Int materialId = static_cast<Trade::MeshObjectData3D*>(objectData.get())->material();

            /* Material not available / not loaded, use a default material */
            if(materialId == -1 || !materials[materialId]) {
                new ColoredDrawable{*object, shader, *meshes[objectData->instance()], 0xffffff_rgbf, drawables};

            /* Textured material. If the texture failed to load, again just use a
            default colored material. */
            }
            /* Color-only material */
            else {
                new ColoredDrawable{*object, shader, *meshes[objectData->instance()], materials[materialId]->diffuseColor(), drawables};
            }
        }

        /* Recursively add children */
        for(std::size_t id: objectData->children())
            addObject(importer, meshes, materials, object, id, shader, drawables);
    }

    void loadCamera(Trade::AbstractImporter& importer, Object3D& cameraObject, SceneGraph::Camera3D* camera) {
        if(importer.cameraCount() == 0) {
            Warning{} << "No camera in scene";
            return;
        }
        camera->setProjectionMatrix(Magnum::Matrix4::perspectiveProjection(importer.camera(0)->fov(), importer.camera(0)->aspectRatio(), importer.camera(0)->near(),importer.camera(0)->far()));
        /* Recursively add all children */
        Containers::Optional<Trade::SceneData> sceneData = importer.scene(importer.defaultScene());
        for(UnsignedInt objectId: sceneData->children3D()) {
            if (importer.object3D(objectId)->instanceType() == Trade::ObjectInstanceType3D::Camera && importer.object3D(objectId)->instance() == 0) {
                Debug{} << "Found camera object";
                cameraObject.setTransformation(importer.object3D(objectId)->transformation());
            }
        }
    }

    void ColoredDrawable::draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) {
        _shader
            .setColor(_color)
            .setTransformationProjectionMatrix(camera.projectionMatrix() * transformationMatrix);
        _mesh.draw(_shader);
    }
}