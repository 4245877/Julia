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
    BodyCommandStatus BodyInstance::apply(const BodyOperation& operation, std::uint64_t commandId)
    {
        using S = BodyCommandStatus;
        return std::visit([&](const auto& value) -> S {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, SetCoordinate>)
            {
                if (!control_) throw std::invalid_argument("No calibrated body control");
                if (reach_->owns(value.joint)) throw std::invalid_argument("Coordinate owned by active reach; cancel and wait for stop");
                const float requested = value.radians + (value.relative ? control_->coordinate(value.joint,value.coordinate) : 0);
                const auto result = control_->setCoordinate(value.joint,value.coordinate,requested,value.policy);
                return result.status == CommandStatus::Applied ? S::Applied :
                    result.status == CommandStatus::Clamped ? S::Clamped : S::Rejected;
            }
            else if constexpr (std::is_same_v<T, ResetCoordinates>)
            {
                if (!control_) throw std::invalid_argument("No calibrated body control");
                reach_->interrupt(ReachReason::CoordinatesReset);
                control_->reset(); discontinuity_ = true;
            }
            else if constexpr (std::is_same_v<T, SetDirectMode>)
            {
                if (!control_) throw std::invalid_argument("No calibrated body control");
                if (control_->enabled()!=value.enabled) reach_->interrupt(ReachReason::ModeChanged);
                control_->setEnabled(value.enabled); discontinuity_ = true;
            }
            else if constexpr (std::is_same_v<T, SetRoot>)
            {
                const auto q = value.pose.rotation;
                if (!finite(value.pose.position) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
                    !std::isfinite(q.z) || !std::isfinite(q.w) || !std::isfinite(glm::length(q)) || glm::length(q) < 1e-8f)
                    throw std::invalid_argument("Invalid root pose");
                if (reach_) reach_->interrupt(ReachReason::RootChanged);
                root_ = {value.pose.position, glm::normalize(q)};
                discontinuity_ = true; // SetRoot is a teleport, never a velocity-limited move.
            }
            else if constexpr (std::is_same_v<T, ResetBody>)
            {
                root_ = {};
                if (reach_) reach_->reset();
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
                if constexpr (std::is_same_v<T, StartReach>) reach_->start(commandId,epoch_,value.target,matrix(root_));
                else reach_->start(value.action,epoch_,value.target,matrix(root_),true);
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
        if (reach_) reach_->advance();
        player_.update(runtime::BodyStepSeconds);
        if (reach_) reach_->measure(player_.globalTransforms(),matrix(root_));
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
        if (reach_) current_.reaches = reach_->observations();
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
        const render::EyeBehaviorOutput output{glm::mix(previous_.eyes.gazeDegrees,current_.eyes.gazeDegrees,weight),
            glm::mix(previous_.eyes.blinkLeft,current_.eyes.blinkLeft,weight),
            glm::mix(previous_.eyes.blinkRight,current_.eyes.blinkRight,weight)};
        const auto* base = current_.direct || current_.playback == PlaybackState::Stopped ? nullptr : player_.clip();
        eyes_.applyOutput(result.pose.locals,base,output);
        animation::computeGlobalTransforms(asset_->skeleton(),result.pose.locals,result.pose.globals);
        animation::buildSkinningPalette(asset_->skeleton(),result.pose.globals,result.pose.palette);
        return result;
    }
}
