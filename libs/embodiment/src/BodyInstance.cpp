#include "julia/embodiment/BodyInstance.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace julia::embodiment
{
    namespace
    {
        const model::ModelAsset& requireAsset(const std::shared_ptr<const model::ModelAsset>& asset)
        { if (!asset) throw std::invalid_argument("Null body asset"); return *asset; }
        bool finite(glm::vec3 value)
        { return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z); }
        glm::mat4 matrix(const RootPose& pose)
        { return glm::translate(glm::mat4{1}, pose.position) * glm::mat4_cast(pose.rotation); }
    }
    BodyInstance::BodyInstance(BodyId id, std::shared_ptr<const model::ModelAsset> asset,
        const BodyProfile* profile, bool eyesEnabled) : id_(id), asset_(std::move(asset)),
        eyes_(requireAsset(asset_)), player_(*asset_), initialEyes_(eyesEnabled),
        previous_(asset_->skeleton()), current_(asset_->skeleton())
    {
        if (!id) throw std::invalid_argument("Body ID must be nonzero");
        if (profile)
        {
            binding_ = std::make_unique<BodyRigBinding>(*profile, *asset_);
            control_ = std::make_unique<DirectBodyControl>(*binding_);
            reach_ = std::make_unique<ArmReach>(*control_);
            player_.setBodyControl(control_.get());
            if (profile->name.starts_with("Raphtalia_prepared/")) {
                support_=std::make_unique<LegSupport>(*control_);
                interaction_=std::make_unique<HandInteraction>(id_,*control_,*reach_);
                interaction_->geometryChanged=[this] { rebuildCollision(); };
                interaction_->validateRest=[this](CollisionContext context) {
                    const auto arm=interaction_->observation().arm;
                    const auto ids=arm==ArmSide::Left ? std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist}:
                        std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist};
                    ArmCoordinates q{};
                    for (std::size_t k=0;k<6;++k) q[k]=control_->coordinate(ids[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4);
                    // The prospective carried copy replaces the free collider in this local check.
                    std::vector<world::SceneObject> objects;
                    for (const auto& o:collisionScene_->objects()) if (!context.carried || o.id!=context.carried->object) objects.push_back(o);
                    const world::CollisionScene scene(sceneVersion_,std::move(objects));
                    return checkArmPath(*control_,arm==ArmSide::Left ? 0:1,std::span{&q,1},matrix(root_),scene,collisionSettings_,context);
                };
                player_.setHandLayer(&interaction_->hand);
            }
        }
        eyes_.setEnabled(eyesEnabled);
        player_.setEyeLayer(&eyes_);
        capture(); collapseHistory();
    }
    BodyCommandResult BodyInstance::submit(BodyCommand command)
    {
        const auto reject = [&](const char* reason) {
            return BodyCommandResult{command.id, tick_, BodyCommandStatus::Rejected, reason, command.epoch}; };
        if (!command.id) return reject("zero command ID");
        if (command.target != id_) return reject("wrong recipient");
        if (command.epoch != epoch_) return reject("stale instance epoch");
        if (command.tick <= tick_) return reject("late command: completed tick");
        if (!seen_.insert(command.id).second) return reject("duplicate command ID");
        if (command.id >= nextId_)
            nextId_ = command.id == std::numeric_limits<std::uint64_t>::max() ? command.id : command.id + 1;
        const BodyCommandResult result{command.id, command.tick, BodyCommandStatus::Queued, {}, command.epoch};
        commands_.push_back(std::move(command));
        return result;
    }
    BodyCommandResult BodyInstance::enqueue(BodyOperation operation)
    {
        if (nextId_ == std::numeric_limits<std::uint64_t>::max())
            throw std::overflow_error("Local command IDs exhausted");
        return submit({nextId_++, id_, tick_ + 1, epoch_, std::move(operation)});
    }
    const std::vector<ReachObservation>& BodyInstance::reachEvents() const
    { static const std::vector<ReachObservation> empty; return reach_ ? reach_->events() : empty; }
    void BodyInstance::invalidatePath(const char* reason)
    {
        pathCheck_={}; pathCheck_.status=collisionScene_ ? PathStatus::Invalidated:PathStatus::Disabled;
        pathCheck_.sceneVersion=collisionScene_ ? sceneVersion_:0; pathCheck_.detail=reason;
    }
    void BodyInstance::rebuildCollision()
    {
        // A phase boundary is legal only after the previous certified segment stopped.
        if (reach_ && reach_->anyActive()) throw std::logic_error("Geometry mutation during active certified reach");
        std::vector<world::SceneObject> objects;
        if (environmentScene_) objects.assign(environmentScene_->objects().begin(),environmentScene_->objects().end());
        if (interaction_) if (const auto c=interaction_->freeCollider()) objects.push_back(*c);
        const bool enabled=environmentScene_ || (interaction_ && interaction_->object());
        if (!enabled) collisionScene_.reset();
        else if (environmentScene_ && (!interaction_ || !interaction_->object()) && environmentScene_->version()>sceneVersion_) {
            sceneVersion_=environmentScene_->version(); collisionScene_=environmentScene_;
        } else collisionScene_=std::make_shared<const world::CollisionScene>(++sceneVersion_,std::move(objects));
        pathCheck_={};
        if (!enabled) { reach_->setPathValidator({}); reach_->setPlanningValidator({},{}); return; }
        pathCheck_.status=PathStatus::NotChecked; pathCheck_.sceneVersion=sceneVersion_;
        reach_->setPathValidator([this](std::size_t side,std::span<const ArmCoordinates> path,unsigned delayTicks) {
            const auto context=interaction_ ? interaction_->collisionContext():CollisionContext{};
            pathCheck_=checkArmPath(*control_,side,path,matrix(root_),*collisionScene_,collisionSettings_,context);
            pathCheck_.command=checkingCommand_; pathCheck_.segmentStartTick=tick_-1+delayTicks; return pathCheck_.status;
        },sceneVersion_);
        reach_->setPlanningValidator([this](std::size_t side,std::span<const ArmCoordinates> path) {
            return std::make_unique<ArmPathCheckJob>(*control_,side,path,matrix(root_),*collisionScene_,collisionSettings_,
                interaction_ ? interaction_->collisionContext():CollisionContext{});
        },[this](PathCheck check) { pathCheck_=std::move(check); });
    }
    void BodyInstance::displayObject(BodyDisplay& display,const BodySnapshot& selected) const
    {
        display.object=selected.object;
        if (display.object && display.object->state==ObjectState::Attached) {
            const auto& o=*display.object;
            const auto e=o.arm==ArmSide::Left ? EffectorId::LeftGrip:EffectorId::RightGrip;
            display.object->worldPose=rigidPose(matrix(display.root)*binding_->effectorFrame(e,display.pose.globals)*o.effectorFromObject);
        }
    }
    void BodyInstance::fillDisplayColliders(BodyDisplay& d) const
    {
        if (!binding_ || !d.direct || !d.collisionScene) return;
        const auto world=matrix(d.root);
        d.colliders=bodyColliders(*binding_,d.pose.globals,world);
        for (std::size_t s=0;s<2;++s) if (d.hands[s].enabled) {
            const auto f=world*binding_->jointFrame(s==0 ? JointId::LeftWrist:JointId::RightWrist,d.pose.globals);
            d.colliders[s*3+2].capsule={glm::dvec3{f*glm::vec4{0,.025f,0,1}},glm::dvec3{f*glm::vec4{0,.160f,0,1}},.045};
        }
        if (d.object && d.object->state==ObjectState::Attached) {
            const auto p=glm::dvec3{d.object->worldPose.position};
            d.colliders.push_back({10,d.object->arm==ArmSide::Left ? SegmentId::LeftHand:SegmentId::RightHand,{p,p,d.object->radius}});
        }
    }
    BodyCommandStatus BodyInstance::apply(const BodyOperation& operation, std::uint64_t commandId)
    {
        using S = BodyCommandStatus;
        return std::visit([&](const auto& value) -> S {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T,SetCoordinate> || std::is_same_v<T,ResetCoordinates> ||
                std::is_same_v<T,SetDirectMode> || std::is_same_v<T,SetRoot> || std::is_same_v<T,ResetBody> ||
                std::is_same_v<T,StartReach> || std::is_same_v<T,RetargetReach> || std::is_same_v<T,CancelReach> ||
                std::is_same_v<T,SetCollisionScene> || std::is_same_v<T,SetHandClosure>)
                if (interaction_ && interaction_->locked()) throw std::invalid_argument("Interaction owns geometry; finish/cancel or release first");
            if constexpr (std::is_same_v<T,EnableSupport> || std::is_same_v<T,AlignSupport> || std::is_same_v<T,MoveSupportRoot>)
                if ((reach_ && reach_->anyActive()) || (interaction_ && interaction_->locked()))
                    throw std::invalid_argument("Support conflicts with active Reach/HandInteraction/Attached object");
            if constexpr (std::is_same_v<T,SetCoordinate> || std::is_same_v<T,StartReach> || std::is_same_v<T,RetargetReach> ||
                std::is_same_v<T,CancelReach> || std::is_same_v<T,GraspObject> || std::is_same_v<T,MoveHeldObject> ||
                std::is_same_v<T,ReleaseObject> || std::is_same_v<T,SetHandClosure> || std::is_same_v<T,SetCollisionScene>)
                if (support_ && support_->enabled()) throw std::invalid_argument("KinematicSupport owns geometry; disable support and wait for rest");
            if constexpr (std::is_same_v<T,SetSupportSurface> || std::is_same_v<T,EnableSupport> || std::is_same_v<T,AlignSupport> ||
                std::is_same_v<T,MoveSupportRoot> || std::is_same_v<T,CancelSupportMotion> || std::is_same_v<T,DisableSupport>)
            {
                if (!support_) throw std::invalid_argument("Calibrated leg support unavailable");
                if constexpr (std::is_same_v<T,SetSupportSurface>) support_->setSurface(value.surface);
                else if constexpr (std::is_same_v<T,EnableSupport>) { support_->enable(matrix(root_)); invalidatePath("KinematicSupport owns geometry; arm certificates invalidated"); }
                else if constexpr (std::is_same_v<T,AlignSupport>) support_->plan(commandId,epoch_,matrix(root_),std::nullopt);
                else if constexpr (std::is_same_v<T,MoveSupportRoot>) support_->plan(commandId,epoch_,matrix(root_),value.worldPosition);
                else if constexpr (std::is_same_v<T,CancelSupportMotion>) support_->cancel();
                else support_->cancel(true);
            }
            else if constexpr (std::is_same_v<T, SetCoordinate>)
            {
                if (!control_) throw std::invalid_argument("No calibrated body control");
                if (reach_->owns(value.joint)) throw std::invalid_argument("Coordinate owned by active reach; cancel and wait for stop");
                const float requested = value.radians + (value.relative ? control_->coordinate(value.joint,value.coordinate) : 0);
                const auto result = control_->setCoordinate(value.joint,value.coordinate,requested,value.policy);
                if (collisionScene_ && result.status!=CommandStatus::Rejected) { discontinuity_=true; invalidatePath("coordinate diagnostic teleport"); }
                return result.status == CommandStatus::Applied ? S::Applied :
                    result.status == CommandStatus::Clamped ? S::Clamped : S::Rejected;
            }
            else if constexpr (std::is_same_v<T, ResetCoordinates>)
            {
                if (!control_) throw std::invalid_argument("No calibrated body control");
                if (support_) support_->interrupt("Coordinates reset: explicit discontinuity");
                reach_->interrupt(ReachReason::CoordinatesReset);
                invalidatePath("coordinates reset");
                control_->reset(); discontinuity_ = true;
            }
            else if constexpr (std::is_same_v<T, SetDirectMode>)
            {
                if (!control_) throw std::invalid_argument("No calibrated body control");
                if (control_->enabled()!=value.enabled) { reach_->interrupt(ReachReason::ModeChanged); invalidatePath("mode changed"); }
                if (support_) support_->interrupt("Direct mode command: explicit discontinuity");
                control_->setEnabled(value.enabled); discontinuity_ = true;
            }
            else if constexpr (std::is_same_v<T, SetRoot>)
            {
                const auto q = value.pose.rotation;
                if (!finite(value.pose.position) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
                    !std::isfinite(q.z) || !std::isfinite(q.w) || !std::isfinite(glm::length(q)) || glm::length(q) < 1e-8f)
                    throw std::invalid_argument("Invalid root pose");
                if (collisionScene_ && glm::length(value.pose.position)>100) throw std::invalid_argument("Collision mode supports root within 100 world metres");
                if (support_) support_->interrupt("Root teleport: explicit discontinuity");
                if (reach_) reach_->interrupt(ReachReason::RootChanged);
                invalidatePath("root teleported");
                root_ = {value.pose.position, glm::normalize(q)};
                discontinuity_ = true; // SetRoot is a teleport, never a velocity-limited move.
            }
            else if constexpr (std::is_same_v<T, ResetBody>)
            {
                if (support_) support_->interrupt("ResetBody: explicit discontinuity / epoch change");
                root_ = {};
                if (reach_) reach_->reset();
                if (interaction_) interaction_->resetFree(epoch_+1);
                invalidatePath("body reset; scene retained");
                if (control_) { control_->reset(); control_->setEnabled(false); }
                eyes_.reset(); eyes_.setEnabled(initialEyes_);
                if (!asset_->animations().empty()) player_.selectClip(0);
                player_.setLooping(true); player_.stop();
                ++epoch_; seen_.clear(); nextId_ = 1;
                discontinuity_ = true;
            }
            else if constexpr (std::is_same_v<T, StartReach> || std::is_same_v<T, RetargetReach>)
            {
                if (!reach_) throw std::invalid_argument("No calibrated body control");
                checkingCommand_=commandId;
                const auto previousCheckCommand=pathCheck_.command;
                try {
                if constexpr (std::is_same_v<T, StartReach>) reach_->start(commandId,epoch_,value.target,matrix(root_),false,commandId);
                else reach_->start(value.action,epoch_,value.target,matrix(root_),true,commandId);
                } catch (const std::exception& error) {
                    if (collisionScene_ && pathCheck_.command==previousCheckCommand) {
                        pathCheck_={}; pathCheck_.command=commandId; pathCheck_.sceneVersion=sceneVersion_;
                        // Command/IK failures are separate from geometric validation.
                        pathCheck_.status=dynamic_cast<const ReachTargetUnreachable*>(&error) ? PathStatus::Unreachable:PathStatus::Rejected;
                    }
                    pathCheck_.detail=error.what();
                    checkingCommand_=0; throw;
                }
                checkingCommand_=0;
            }
            else if constexpr (std::is_same_v<T, SetCollisionScene>)
            {
                if (!reach_) throw std::invalid_argument("No calibrated body control");
                if (reach_->anyActive()) throw std::invalid_argument("Scene/settings change conflicts with certified active reach; wait for completion");
                validateCollisionSettings(value.settings);
                if (value.scene && (value.scene->version()<=sceneVersion_ || glm::length(root_.position)>100))
                    throw std::invalid_argument("Scene version must increase; root must be within 100 metres");
                if (value.scene) {
                    if (interaction_ && interaction_->object() && value.scene->objects().size()>=64)
                        throw std::invalid_argument("No scene slot for interactive object");
                    for (const auto& object:value.scene->objects()) {
                        if (object.id & (1ull<<63)) throw std::invalid_argument("Scene uses reserved interactive object namespace");
                        if (interaction_ && interaction_->object()) {
                            const auto& item=*interaction_->object();
                            const world::Capsule sphere{item.worldPose.position,item.worldPose.position,item.radius};
                            if (world::clearance(sphere,object).gap<=value.settings.margin+value.settings.numericSlack)
                                throw std::invalid_argument("Scene overlaps stationary interactive object");
                        }
                    }
                }
                environmentScene_=value.scene; collisionSettings_=value.settings;
                if (value.scene) sceneVersion_=value.scene->version()-1;
                rebuildCollision();
                discontinuity_=true;
            }
            else if constexpr (std::is_same_v<T,CreateInteractionSphere> || std::is_same_v<T,DeleteInteractionObject> ||
                std::is_same_v<T,GraspObject> || std::is_same_v<T,MoveHeldObject> || std::is_same_v<T,ReleaseObject> ||
                std::is_same_v<T,CancelInteraction> || std::is_same_v<T,SetHandClosure>)
            {
                if (!interaction_) throw std::invalid_argument("Hand profile unavailable for this rig");
                if constexpr (std::is_same_v<T,CreateInteractionSphere>) {
                    if (environmentScene_) {
                        if (environmentScene_->objects().size()>=64) throw std::invalid_argument("No scene slot for interactive object");
                        const world::Capsule sphere{value.pose.position,value.pose.position,value.radius};
                        for (const auto& o:environmentScene_->objects())
                            if ((o.id & (1ull<<63)) || world::clearance(sphere,o).gap<=collisionSettings_.margin+collisionSettings_.numericSlack)
                                throw std::invalid_argument("Object overlaps environment or uses reserved object namespace");
                    }
                    interaction_->create(value);
                }
                else if constexpr (std::is_same_v<T,DeleteInteractionObject>) interaction_->erase(value.object);
                else if constexpr (std::is_same_v<T,GraspObject>) interaction_->grasp(commandId,epoch_,value,matrix(root_));
                else if constexpr (std::is_same_v<T,MoveHeldObject>) interaction_->move(commandId,epoch_,value,matrix(root_));
                else if constexpr (std::is_same_v<T,ReleaseObject>) interaction_->release(commandId,epoch_,value.object);
                else if constexpr (std::is_same_v<T,CancelInteraction>) interaction_->cancel(value.action);
                else {
                    if (!control_->enabled() || reach_->anyActive()) throw std::invalid_argument("Hand diagnostics require idle direct mode");
                    interaction_->hand.set(value.arm,value.closure,value.enabled);
                }
            }
            else if constexpr (std::is_same_v<T, CancelReach>)
            {
                if (!reach_) throw std::invalid_argument("No calibrated body control");
                reach_->cancel(value.arm,value.action);
            }
            else if constexpr (std::is_same_v<T, ClipCommand>)
            {
                if (!std::isfinite(value.value)) throw std::invalid_argument("Non-finite clip command");
                switch (value.operation)
                {
                case ClipOperation::Select:
                    if (value.value < 0 || value.value >= static_cast<double>(asset_->animations().size()) ||
                        std::floor(value.value) != value.value) throw std::out_of_range("Invalid clip index");
                    player_.selectClip(static_cast<std::size_t>(value.value)); discontinuity_ = true; break;
                case ClipOperation::Play: player_.play(); discontinuity_ = true; break;
                case ClipOperation::Pause: player_.pause(); break;
                case ClipOperation::Stop: player_.stop(); discontinuity_ = true; break;
                case ClipOperation::Seek: player_.seek(value.value); discontinuity_ = true; break;
                case ClipOperation::Loop: player_.setLooping(value.value != 0); break;
                case ClipOperation::TogglePlay:
                    if (player_.state() == PlaybackState::Playing) player_.pause();
                    else { player_.play(); discontinuity_ = true; } break;
                case ClipOperation::ToggleLoop: player_.setLooping(!player_.looping()); break;
                case ClipOperation::Step:
                    player_.pause(); player_.seek(player_.timeSeconds()+value.value); discontinuity_ = true; break;
                case ClipOperation::Next:
                    if (value.value != 1 && value.value != -1) throw std::invalid_argument("Invalid clip direction");
                    if (!asset_->animations().empty())
                    {
                        const auto count = asset_->animations().size();
                        player_.selectClip((player_.clipIndex() + (value.value > 0 ? 1 : count-1)) % count);
                        player_.play(); discontinuity_ = true;
                    }
                    break;
                default: throw std::invalid_argument("Unknown clip operation");
                }
            }
            else if constexpr (std::is_same_v<T, EyeCommand>)
            {
                if (!finite(value.value)) throw std::invalid_argument("Non-finite eye command");
                auto& behavior = eyes_.behavior();
                switch (value.operation)
                {
                case EyeOperation::Enable: eyes_.setEnabled(value.value.x != 0); discontinuity_ = true; break;
                case EyeOperation::Natural:
                    eyes_.setEnabled(true); eyes_.clearTargetPoint(); behavior.releaseBlink(); behavior.releaseGaze();
                    behavior.setAutoBlink(true); behavior.setAutoGaze(true);
                    if (EyePoseLayer::authoredEyes(player_.clip())) player_.stop();
                    discontinuity_ = true; break;
                case EyeOperation::AutoBlink: behavior.setAutoBlink(!behavior.autoBlink()); break;
                case EyeOperation::AutoGaze: behavior.setAutoGaze(!behavior.autoGaze()); break;
                case EyeOperation::Blink: behavior.triggerBlink(value.value.x != 0,value.value.y != 0); break;
                case EyeOperation::Gaze: eyes_.clearTargetPoint(); behavior.setGazeDegrees(glm::vec2{value.value}); break;
                case EyeOperation::Neutral: eyes_.clearTargetPoint(); behavior.neutral(); break;
                case EyeOperation::Target:
                    if (eyes_.hasTargetPoint()) eyes_.clearTargetPoint(); else eyes_.setTargetPoint(value.value); break;
                case EyeOperation::ClearTarget: eyes_.clearTargetPoint(); break;
                case EyeOperation::ResetSeed: behavior.reset(); discontinuity_ = true; break;
                case EyeOperation::BlinkInterval:
                {
                    auto settings = behavior.settings();
                    settings.blinkMeanSeconds = std::clamp(settings.blinkMeanSeconds+value.value.x,1.0,15.0);
                    behavior.setSettings(settings); break;
                }
                case EyeOperation::ManualBlink: behavior.setBlink(value.value.x,value.value.y); discontinuity_ = true; break;
                default: throw std::invalid_argument("Unknown eye operation");
                }
            }
            return S::Applied;
        }, operation);
    }
    void BodyInstance::step()
    {
        if (tick_ == std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("Body tick exhausted");
        previous_ = current_; ++tick_; results_.clear(); discontinuity_ = false;
        if (reach_) reach_->clearEvents();
        if (interaction_) interaction_->beginTick();
        if (support_) support_->beginTick();
        std::sort(commands_.begin(),commands_.end(),[](const auto& a,const auto& b) {
            return a.tick != b.tick ? a.tick < b.tick : a.id < b.id; });
        std::vector<BodyCommand> due;
        const auto end = std::find_if(commands_.begin(),commands_.end(),[&](const auto& c) { return c.tick > tick_; });
        due.assign(commands_.begin(),end); commands_.erase(commands_.begin(),end);
        for (std::size_t i = 0; i < due.size(); ++i)
        {
            const auto& command = due[i];
            try
            {
                const auto status = apply(command.operation,command.id);
                results_.push_back({command.id,tick_,status,status == BodyCommandStatus::Rejected ? "joint limit" : "",command.epoch});
            }
            catch (const std::exception& error)
            { results_.push_back({command.id,tick_,BodyCommandStatus::Rejected,error.what(),command.epoch}); }
            if (command.epoch != epoch_)
            {
                const auto cancel = [&](const auto& c) { results_.push_back({c.id,tick_,BodyCommandStatus::Cancelled,"instance reset",c.epoch}); };
                for (++i; i < due.size(); ++i) cancel(due[i]);
                for (const auto& c : commands_) cancel(c);
                commands_.clear(); break;
            }
        }
        // Commands take effect at the beginning of the interval ending at tick_.
        const auto clipTimeBefore = player_.timeSeconds();
        if (interaction_) interaction_->hand.advance();
        if (reach_) reach_->advance(tick_);
        if (support_) support_->advance(root_.position);
        player_.update(runtime::BodyStepSeconds);
        if (support_) support_->measure(player_.globalTransforms(),matrix(root_));
        if (reach_) reach_->measure(player_.globalTransforms(),matrix(root_));
        if (interaction_) interaction_->advance(player_.globalTransforms(),matrix(root_));
        if ((!control_ || !control_->enabled()) && player_.timeSeconds() < clipTimeBefore)
            discontinuity_ = true; // Loop seam: no blending across the end/start cut.
        capture();
        if (discontinuity_) collapseHistory();
    }
    void BodyInstance::capture()
    {
        current_.tick = tick_; current_.epoch = epoch_;
        current_.time = static_cast<double>(tick_) * runtime::BodyStepSeconds;
        current_.root = root_; current_.clipTime = player_.timeSeconds(); current_.clipIndex = player_.clipIndex();
        current_.playback = player_.state(); current_.direct = control_ && control_->enabled();
        current_.eyesEnabled = eyes_.enabled(); current_.eyes = eyes_.behavior().output();
        current_.collisionScene=collisionScene_; current_.collisionSettings=collisionSettings_; current_.pathCheck=pathCheck_;
        if (reach_) current_.reaches = reach_->observations();
        if (support_) current_.support=support_->observation();
        if (interaction_) { current_.hands=interaction_->hand.states(); current_.object=interaction_->object(); current_.interaction=interaction_->observation(); }
        if (control_) current_.coordinates.assign(control_->coordinates().begin(),control_->coordinates().end());
        std::fill(current_.animated.begin(),current_.animated.end(),false);
        if (!current_.direct && player_.state() != PlaybackState::Stopped && player_.clip())
        {
            animation::sampleAnimation(asset_->skeleton(),*player_.clip(),player_.timeSeconds(),current_.pose.locals,current_.components);
            for (const auto& channel : player_.clip()->channels)
                current_.animated[channel.nodeIndex] = !channel.translations.empty() || !channel.rotations.empty() || !channel.scales.empty();
        }
        std::copy(player_.localTransforms().begin(),player_.localTransforms().end(),current_.pose.locals.begin());
        std::copy(player_.globalTransforms().begin(),player_.globalTransforms().end(),current_.pose.globals.begin());
        std::copy(player_.palette().begin(),player_.palette().end(),current_.pose.palette.begin());
    }
    void BodyInstance::collapseHistory() { previous_ = current_; }
    glm::mat4 BodyInstance::resourceRoot(const RootPose& root) const
    {
        const auto bodyFromResource = binding_ ? binding_->profile().bodyFromResource : glm::mat4{1};
        return glm::inverse(bodyFromResource) * matrix(root) * bodyFromResource;
    }
    BodyDisplay BodyInstance::display(double alpha) const
    {
        if (!std::isfinite(alpha) || alpha < 0 || alpha > 1) throw std::invalid_argument("Invalid display fraction");
        BodyDisplay result(asset_->skeleton());
        result.reaches=current_.reaches; result.support=current_.support;
        result.hands=current_.hands; result.interaction=current_.interaction;
        result.collisionScene=current_.collisionScene; result.pathCheck=current_.pathCheck;
        result.fromTick = previous_.tick; result.toTick = current_.tick;
        result.direct = current_.direct;
        result.coordinates = current_.coordinates;
        result.time = previous_.time + (current_.time-previous_.time)*alpha;
        const auto weight = static_cast<float>(alpha);
        result.root = {glm::mix(previous_.root.position,current_.root.position,weight),
            glm::normalize(glm::slerp(previous_.root.rotation,current_.root.rotation,weight))};
        for (std::size_t i = 0; i < result.coordinates.size(); ++i)
            result.coordinates[i] = glm::mix(previous_.coordinates[i],current_.coordinates[i],weight);
        if (alpha == 0 || alpha == 1 || previous_.tick == current_.tick)
        {
            const auto& selected = alpha == 0 ? previous_ : current_;
            result.pose = selected.pose; result.root = selected.root; result.coordinates = selected.coordinates;
            result.support=selected.support;
            result.hands=selected.hands; result.interaction=selected.interaction; result.reaches=selected.reaches;
            result.collisionScene=selected.collisionScene; result.pathCheck=selected.pathCheck; displayObject(result,selected);
            fillDisplayColliders(result);
            return result;
        }
        animation::setBindPose(asset_->skeleton(),result.pose.locals);
        if (current_.direct)
        {
            control_->applyCoordinates(result.pose.locals,result.coordinates);
        }
        else
            for (std::size_t i = 0; i < current_.animated.size(); ++i)
                if (current_.animated[i])
                {
                    const auto& a = previous_.components[i]; const auto& b = current_.components[i];
                    result.pose.locals[i] = glm::translate(glm::mat4{1},glm::mix(a.translation,b.translation,weight)) *
                        glm::mat4_cast(glm::normalize(glm::slerp(a.rotation,b.rotation,weight))) *
                        glm::scale(glm::mat4{1},glm::mix(a.scale,b.scale,weight));
                }
        if (interaction_ && current_.direct) {
            for (std::size_t k=0;k<2;++k) { result.hands[k].closure=glm::mix(previous_.hands[k].closure,current_.hands[k].closure,weight);
                result.hands[k].blend=glm::mix(previous_.hands[k].blend,current_.hands[k].blend,weight); }
            interaction_->hand.apply(result.pose.locals,result.hands);
        }
        const render::EyeBehaviorOutput output{glm::mix(previous_.eyes.gazeDegrees,current_.eyes.gazeDegrees,weight),
            glm::mix(previous_.eyes.blinkLeft,current_.eyes.blinkLeft,weight),
            glm::mix(previous_.eyes.blinkRight,current_.eyes.blinkRight,weight)};
        const auto* base = current_.direct || current_.playback == PlaybackState::Stopped ? nullptr : player_.clip();
        eyes_.applyOutput(result.pose.locals,base,output);
        animation::computeGlobalTransforms(asset_->skeleton(),result.pose.locals,result.pose.globals);
        animation::buildSkinningPalette(asset_->skeleton(),result.pose.globals,result.pose.palette);
        if (support_) result.support=support_->displayObservation(result.support,result.pose.globals,matrix(result.root));
        displayObject(result,current_);
        fillDisplayColliders(result);
        return result;
    }
}
