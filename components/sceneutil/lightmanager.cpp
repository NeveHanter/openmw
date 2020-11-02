#include "lightmanager.hpp"

#include <osgUtil/CullVisitor>

#include <components/debug/debuglog.hpp>
#include <components/sceneutil/util.hpp>

namespace SceneUtil
{
    Light::Light()
        : ambient()
        , diffuse()
        , position()
        , specular()
        , constantAttenuation()
        , linearAttenuation()
        , quadraticAttenuation() {}

    Light::Light(const Light& copy, const osg::CopyOp& copyop)
        : osg::Node(copy, copyop)
        , ambient(copy.ambient)
        , diffuse(copy.diffuse)
        , position(copy.position)
        , specular(copy.specular)
        , constantAttenuation(copy.constantAttenuation)
        , linearAttenuation(copy.linearAttenuation)
        , quadraticAttenuation(copy.quadraticAttenuation) {}

    LightStateCache* getLightStateCache(unsigned int contextid)
    {
        static std::vector<LightStateCache> cacheVector;
        if (cacheVector.size() < contextid+1)
            cacheVector.resize(contextid+1);
        return &cacheVector[contextid];
    }

    void LightStateAttribute::apply(osg::State& state) const {
        if (mLights.empty())
            return;

        osg::Matrix modelViewMatrix = state.getModelViewMatrix();

        state.applyModelViewMatrix(state.getInitialViewMatrix());

        LightStateCache* cache = getLightStateCache(state.getContextID());

        // Update uniform pointers
        cache->lightsAmbientUniform = mUniforms[0];
        cache->lightsDiffuseUniform = mUniforms[1];
        cache->lightsPositionUniform = mUniforms[2];
        cache->lightsSpecularUniform = mUniforms[3];
        cache->lightsConstantAttenuationUniform = mUniforms[4];
        cache->lightsLinearAttenuationUniform = mUniforms[5];
        cache->lightsQuadraticAttenuationUniform = mUniforms[6];
        cache->lightsCountUniform = mUniforms[7];

        cache->lightsCountUniform->set(static_cast<GLint>(mLights.size()));

        for (unsigned int i = 0; i < mLights.size(); ++i) {
            osg::ref_ptr<Light> currentLight = cache->lastAppliedLights[mIndex + i];
            osg::ref_ptr<Light> newLight = mLights[i];

            if (currentLight != newLight) {
                applyLight(cache, mIndex + i, newLight);
                cache->lastAppliedLights[mIndex + i] = newLight;
            }
        }

        state.applyModelViewMatrix(modelViewMatrix);
    }

    void LightStateAttribute::applyLight(LightStateCache* cache, int lightNum, const osg::ref_ptr<Light>& light) const {
        // TODO: enable this once spot lights are supported
        // need to transform SPOT_DIRECTION by the world matrix?
        //glLightfv( lightNum, GL_SPOT_DIRECTION,        light->getDirection().ptr() );
        //glLightf ( lightNum, GL_SPOT_EXPONENT,         light->getSpotExponent() );
        //glLightf ( lightNum, GL_SPOT_CUTOFF,           light->getSpotCutoff() );

        // TODO: Remove this function as the apply step should execute once, also only update uniforms with the data changed
        cache->lightsAmbientUniform->setElement(lightNum, light->getAmbient());
        cache->lightsDiffuseUniform->setElement(lightNum, light->getDiffuse());
        cache->lightsPositionUniform->setElement(lightNum, light->getPosition());
        cache->lightsSpecularUniform->setElement(lightNum, light->getSpecular());
        cache->lightsConstantAttenuationUniform->setElement(lightNum, light->getConstantAttenuation());
        cache->lightsLinearAttenuationUniform->setElement(lightNum, light->getLinearAttenuation());
        cache->lightsQuadraticAttenuationUniform->setElement(lightNum, light->getQuadraticAttenuation());
    }

    LightManager* findLightManager(const osg::NodePath& path)
    {
        for (unsigned int i=0;i<path.size(); ++i)
        {
            if (LightManager* lightManager = dynamic_cast<LightManager*>(path[i]))
                return lightManager;
        }
        return nullptr;
    }

    // Set on a LightSource. Adds the light source to its light manager for the current frame.
    // This allows us to keep track of the current lights in the scene graph without tying creation & destruction to the manager.
    class CollectLightCallback : public osg::NodeCallback
    {
    public:
        CollectLightCallback()
            : mLightManager(0) { }

        CollectLightCallback(const CollectLightCallback& copy, const osg::CopyOp& copyop)
            : osg::NodeCallback(copy, copyop)
            , mLightManager(0) { }

        META_Object(SceneUtil, SceneUtil::CollectLightCallback)

        void operator()(osg::Node* node, osg::NodeVisitor* nv) override
        {
            if (!mLightManager)
            {
                mLightManager = findLightManager(nv->getNodePath());

                if (!mLightManager)
                    throw std::runtime_error("can't find parent LightManager");
            }

            mLightManager->addLight(static_cast<LightSource*>(node), osg::computeLocalToWorld(nv->getNodePath()), nv->getTraversalNumber());

            traverse(node, nv);
        }

    private:
        LightManager* mLightManager;
    };

    // Set on a LightManager. Clears the data from the previous frame.
    class LightManagerUpdateCallback : public osg::NodeCallback
    {
    public:
        LightManagerUpdateCallback()
            { }

        LightManagerUpdateCallback(const LightManagerUpdateCallback& copy, const osg::CopyOp& copyop)
            : osg::NodeCallback(copy, copyop)
            { }

        META_Object(SceneUtil, LightManagerUpdateCallback)

        void operator()(osg::Node* node, osg::NodeVisitor* nv) override
        {
            LightManager* lightManager = static_cast<LightManager*>(node);
            lightManager->update();

            traverse(node, nv);
        }
    };

    LightManager::LightManager()
        : mStartLight(0)
        , mLightingMask(~0u)
    {
        setUpdateCallback(new LightManagerUpdateCallback);

        mUniforms.reserve(8);
        mUniforms.emplace_back(new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "lightAmbient", LIGHTS_NUM));
        mUniforms.emplace_back(new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "lightDiffuse", LIGHTS_NUM));
        mUniforms.emplace_back(new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "lightPosition", LIGHTS_NUM));
        mUniforms.emplace_back(new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "lightSpecular", LIGHTS_NUM));
        mUniforms.emplace_back(new osg::Uniform(osg::Uniform::Type::FLOAT, "lightConstantAttenuation", LIGHTS_NUM));
        mUniforms.emplace_back(new osg::Uniform(osg::Uniform::Type::FLOAT, "lightLinearAttenuation", LIGHTS_NUM));
        mUniforms.emplace_back(new osg::Uniform(osg::Uniform::Type::FLOAT, "lightQuadraticAttenuation", LIGHTS_NUM));
        mUniforms.emplace_back(new osg::Uniform(osg::Uniform::Type::INT, "lightCount"));

        for (unsigned int i = 0; i < LIGHTS_NUM; ++i) {
            mDummies.push_back(new LightStateAttribute(i, std::vector<osg::ref_ptr<Light>>(), mUniforms));
        }
    }

    LightManager::LightManager(const LightManager &copy, const osg::CopyOp &copyop)
        : osg::Group(copy, copyop)
        , mStartLight(copy.mStartLight)
        , mLightingMask(copy.mLightingMask)
        , mUniforms(copy.mUniforms)
    {

    }

    void LightManager::setLightingMask(unsigned int mask)
    {
        mLightingMask = mask;
    }

    unsigned int LightManager::getLightingMask() const
    {
        return mLightingMask;
    }

    void LightManager::update()
    {
        mLights.clear();
        mLightsInViewSpace.clear();

        // do an occasional cleanup for orphaned lights
        for (int i=0; i<2; ++i)
        {
            if (mStateSetCache[i].size() > 5000)
                mStateSetCache[i].clear();
        }
    }

    void LightManager::addLight(LightSource* lightSource, const osg::Matrixf& worldMat, unsigned int frameNum)
    {
        LightSourceTransform l;
        l.mLightSource = lightSource;
        l.mWorldMatrix = worldMat;
        lightSource->getLight(frameNum)->setPosition(osg::Vec4f(worldMat.getTrans().x(),
                                                        worldMat.getTrans().y(),
                                                        worldMat.getTrans().z(), 1.f));
        mLights.push_back(l);
    }

    /* similar to the boost::hash_combine */
    template <class T>
    inline void hash_combine(std::size_t& seed, const T& v)
    {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }

    osg::ref_ptr<osg::StateSet> LightManager::getLightListStateSet(const LightList &lightList, unsigned int frameNum)
    {
        // possible optimization: return a StateSet containing all requested lights plus some extra lights (if a suitable one exists)
        size_t hash = 0;
        for (auto i : lightList)
            hash_combine(hash, i->mLightSource->getId());

        LightStateSetMap& stateSetCache = mStateSetCache[frameNum % 2];

        LightStateSetMap::iterator found = stateSetCache.find(hash);
        if (found != stateSetCache.end())
            return found->second;
        else
        {
            osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
            std::vector<osg::ref_ptr<Light>> lights;
            lights.reserve(lightList.size());
            for (unsigned int i = 0; i < lightList.size();++i)
                lights.emplace_back(lightList[i]->mLightSource->getLight(frameNum));

            // the first light state attribute handles the actual state setting for all lights
            // it's best to batch these up so that we don't need to touch the modelView matrix more than necessary
            // don't use setAttributeAndModes, that does not support light indices!
            stateset->setAttribute(new LightStateAttribute(mStartLight, std::move(lights), mUniforms), osg::StateAttribute::ON);

            for (const osg::ref_ptr<osg::Uniform>& uniform : mUniforms)
                stateset->addUniform(uniform, osg::StateAttribute::ON);

            //for (unsigned int i=0; i<lightList.size(); ++i)
            //    stateset->setMode(GL_LIGHT0 + mStartLight + i, osg::StateAttribute::ON);

            // need to push some dummy attributes to ensure proper state tracking
            // lights need to reset to their default when the StateSet is popped
            for (unsigned int i = 1; i < lightList.size(); ++i) {
                LightStateAttribute* dummy = mDummies[i + mStartLight].get();
                for (osg::Uniform* uniform : dummy->mUniforms)
                    stateset->addUniform(uniform, osg::StateAttribute::ON);

                stateset->setAttribute(dummy, osg::StateAttribute::ON);
            }

            stateSetCache.emplace(hash, stateset);
            return stateset;
        }
    }

    const std::vector<LightManager::LightSourceTransform>& LightManager::getLights() const
    {
        return mLights;
    }

    const std::vector<LightManager::LightSourceViewBound>& LightManager::getLightsInViewSpace(osg::Camera *camera, const osg::RefMatrix* viewMatrix)
    {
        osg::observer_ptr<osg::Camera> camPtr (camera);
        std::map<osg::observer_ptr<osg::Camera>, LightSourceViewBoundCollection>::iterator it = mLightsInViewSpace.find(camPtr);

        if (it == mLightsInViewSpace.end())
        {
            it = mLightsInViewSpace.insert(std::make_pair(camPtr, LightSourceViewBoundCollection())).first;

            for (std::vector<LightSourceTransform>::iterator lightIt = mLights.begin(); lightIt != mLights.end(); ++lightIt)
            {
                osg::Matrixf worldViewMat = lightIt->mWorldMatrix * (*viewMatrix);
                osg::BoundingSphere viewBound = osg::BoundingSphere(osg::Vec3f(0,0,0), lightIt->mLightSource->getRadius());
                transformBoundingSphere(worldViewMat, viewBound);

                LightSourceViewBound l;
                l.mLightSource = lightIt->mLightSource;
                l.mViewBound = viewBound;
                it->second.push_back(l);
            }
        }
        return it->second;
    }

    class DisableLight : public osg::StateAttribute
    {
    public:
        DisableLight() : mIndex(0) {}
        DisableLight(int index, std::vector<osg::ref_ptr<osg::Uniform>> uniforms) : mIndex(index), mUniforms(std::move(uniforms)) {}

        DisableLight(const DisableLight& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY)
            : osg::StateAttribute(copy,copyop), mIndex(copy.mIndex), mUniforms(copy.mUniforms) {}

        osg::Object* cloneType() const override { return new DisableLight(mIndex, mUniforms); }
        osg::Object* clone(const osg::CopyOp& copyop) const override { return new DisableLight(*this,copyop); }
        bool isSameKindAs(const osg::Object* obj) const override { return dynamic_cast<const DisableLight *>(obj)!=nullptr; }
        const char* libraryName() const override { return "SceneUtil"; }
        const char* className() const override { return "DisableLight"; }
        Type getType() const override { return LIGHT; }

        unsigned int getMember() const override
        {
            return mIndex;
        }

        bool getModeUsage(ModeUsage & usage) const override
        {
            //usage.usesMode(GL_LIGHT0 + mIndex);
            return true;
        }

        int compare(const StateAttribute &sa) const override
        {
            throw std::runtime_error("DisableLight::compare: unimplemented");
        }

        void apply(osg::State& state) const override
        {
            LightStateCache* cache = getLightStateCache(state.getContextID());
            cache->lastAppliedLights[mIndex] = nullptr;

            if (mUniforms.empty()) {
                return;
            }

            cache->lightsAmbientUniform = mUniforms[0];
            cache->lightsDiffuseUniform = mUniforms[1];
            cache->lightsPositionUniform = mUniforms[2];
            cache->lightsSpecularUniform = mUniforms[3];
            cache->lightsConstantAttenuationUniform = mUniforms[4];
            cache->lightsLinearAttenuationUniform = mUniforms[5];
            cache->lightsQuadraticAttenuationUniform = mUniforms[6];
            cache->lightsCountUniform = mUniforms[7];

            cache->lightsAmbientUniform->setElement(mIndex, mnullptr);
            cache->lightsDiffuseUniform->setElement(mIndex, mnullptr);
//            cache->lightsPositionUniform->setElement(mIndex, light->getPosition());
            cache->lightsSpecularUniform->setElement(mIndex, mnullptr);
//            cache->lightsConstantAttenuationUniform->setElement(mIndex, light->getConstantAttenuation());
//            cache->lightsLinearAttenuationUniform->setElement(mIndex, light->getLinearAttenuation());
//            cache->lightsQuadraticAttenuationUniform->setElement(mIndex, light->getQuadraticAttenuation());
            /*
            int lightNum = GL_LIGHT0 + mIndex;
            glLightfv( lightNum, GL_AMBIENT,               mnullptr.ptr() );
            glLightfv( lightNum, GL_DIFFUSE,               mnullptr.ptr() );
            glLightfv( lightNum, GL_SPECULAR,              mnullptr.ptr() );

            LightStateCache* cache = getLightStateCache(state.getContextID());
            cache->lastAppliedLight[mIndex] = nullptr;
            */
        }

    private:
        unsigned int mIndex;
        osg::Vec4f mnullptr;

        std::vector<osg::ref_ptr<osg::Uniform>> mUniforms;
    };

    void LightManager::setStartLight(int start)
    {
        mStartLight = start;

        // Set default light state to zero
        // This is necessary because shaders don't respect glDisable(GL_LIGHTX) so in addition to disabling
        // we'll have to set a light state that has no visible effect
        for (int i=start; i<LIGHTS_NUM; ++i)
        {
            osg::ref_ptr<DisableLight> defaultLight (new DisableLight(i, mUniforms));
            getOrCreateStateSet()->setAttributeAndModes(defaultLight, osg::StateAttribute::OFF);
        }
    }

    int LightManager::getStartLight() const
    {
        return mStartLight;
    }

    static int sLightId = 0;

    LightSource::LightSource()
        : mRadius(0.f)
    {
        setUpdateCallback(new CollectLightCallback);
        mId = sLightId++;
    }

    LightSource::LightSource(const LightSource &copy, const osg::CopyOp &copyop)
        : osg::Node(copy, copyop)
        , mRadius(copy.mRadius)
    {
        mId = sLightId++;

        for (int i=0; i<2; ++i)
            mLight[i] = new Light(*copy.mLight[i]);
    }


    bool sortLights (const LightManager::LightSourceViewBound* left, const LightManager::LightSourceViewBound* right)
    {
        return left->mViewBound.center().length2() - left->mViewBound.radius2()*81 < right->mViewBound.center().length2() - right->mViewBound.radius2()*81;
    }

    void LightListCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);

        bool pushedState = pushLightState(node, cv);
        traverse(node, nv);
        if (pushedState)
            cv->popStateSet();
    }

    bool LightListCallback::pushLightState(osg::Node *node, osgUtil::CullVisitor *cv)
    {
        if (!mLightManager)
        {
            mLightManager = findLightManager(cv->getNodePath());
            if (!mLightManager)
                return false;
        }

        if (!(cv->getTraversalMask() & mLightManager->getLightingMask()))
            return false;

        // Possible optimizations:
        // - cull list of lights by the camera frustum
        // - organize lights in a quad tree


        // update light list if necessary
        // makes sure we don't update it more than once per frame when rendering with multiple cameras
        if (mLastFrameNumber != cv->getTraversalNumber())
        {
            mLastFrameNumber = cv->getTraversalNumber();

            // Don't use Camera::getViewMatrix, that one might be relative to another camera!
            const osg::RefMatrix* viewMatrix = cv->getCurrentRenderStage()->getInitialViewMatrix();
            const std::vector<LightManager::LightSourceViewBound>& lights = mLightManager->getLightsInViewSpace(cv->getCurrentCamera(), viewMatrix);

            // get the node bounds in view space
            // NB do not node->getBound() * modelView, that would apply the node's transformation twice
            osg::BoundingSphere nodeBound;
            osg::Transform* transform = node->asTransform();
            if (transform)
            {
                for (unsigned int i=0; i<transform->getNumChildren(); ++i)
                    nodeBound.expandBy(transform->getChild(i)->getBound());
            }
            else
                nodeBound = node->getBound();
            osg::Matrixf mat = *cv->getModelViewMatrix();
            transformBoundingSphere(mat, nodeBound);

            mLightList.clear();
            for (unsigned int i=0; i<lights.size(); ++i)
            {
                const LightManager::LightSourceViewBound& l = lights[i];

                if (mIgnoredLightSources.count(l.mLightSource))
                    continue;

                if (l.mViewBound.intersects(nodeBound))
                    mLightList.push_back(&l);
            }
        }
        if (!mLightList.empty())
        {
            unsigned int maxLights = static_cast<unsigned int> (LIGHTS_NUM - mLightManager->getStartLight());

            osg::StateSet* stateset = nullptr;

            if (mLightList.size() > maxLights)
            {
                // remove lights culled by this camera
                LightManager::LightList lightList = mLightList;
                for (LightManager::LightList::iterator it = lightList.begin(); it != lightList.end() && lightList.size() > maxLights; )
                {
                    osg::CullStack::CullingStack& stack = cv->getModelViewCullingStack();

                    osg::BoundingSphere bs = (*it)->mViewBound;
                    bs._radius = bs._radius*2;
                    osg::CullingSet& cullingSet = stack.front();
                    if (cullingSet.isCulled(bs))
                    {
                        it = lightList.erase(it);
                        continue;
                    }
                    else
                        ++it;
                }

                if (lightList.size() > maxLights)
                {
                    // sort by proximity to camera, then get rid of furthest away lights
                    std::sort(lightList.begin(), lightList.end(), sortLights);
                    while (lightList.size() > maxLights)
                        lightList.pop_back();
                }
                stateset = mLightManager->getLightListStateSet(lightList, cv->getTraversalNumber());
            }
            else
                stateset = mLightManager->getLightListStateSet(mLightList, cv->getTraversalNumber());


            cv->pushStateSet(stateset);
            return true;
        }
        return false;
    }

}
