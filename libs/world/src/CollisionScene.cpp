#include "julia/world/CollisionScene.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
namespace julia::world
{
    namespace
    {
        bool finite(glm::dvec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
        glm::dvec3 closest(glm::dvec3 p, glm::dvec3 a, glm::dvec3 b)
        {
            const auto d=b-a; const auto l=glm::dot(d,d);
            return a+d*(l>1e-24 ? std::clamp(glm::dot(p-a,d)/l,0.0,1.0):0.0);
        }
        Clearance result(glm::dvec3 a, glm::dvec3 b, double ra, double rb)
        {
            const auto d=a-b; const double l=glm::length(d);
            const auto n=l>1e-12 ? d/l:glm::dvec3{1,0,0};
            return {l-ra-rb,a-n*ra,b+n*rb,n};
        }
        // Split segment at slab crossings. Squared distance to AABB is quadratic
        // on each interval, so endpoints and its stationary point are sufficient.
        Clearance boxDistance(const Capsule& c, glm::dvec3 h)
        {
            std::array<double,8> cuts{}; std::size_t count=2; cuts[1]=1;
            const auto d=c.b-c.a;
            for (int k=0;k<3;++k) if (std::abs(d[k])>1e-15)
                for (double sign : {-1.0,1.0})
                { const double t=(sign*h[k]-c.a[k])/d[k]; if (t>0 && t<1) cuts[count++]=t; }
            std::sort(cuts.begin(),cuts.begin()+static_cast<std::ptrdiff_t>(count));
            double best=1e300; glm::dvec3 pa{0},pb{0};
            const auto sample=[&](double t) {
                const auto a=c.a+d*t,b=glm::clamp(a,-h,h); const double dd=glm::dot(a-b,a-b);
                if (dd<best) { best=dd; pa=a; pb=b; }
            };
            for (std::size_t i=1;i<count;++i)
            {
                const double lo=cuts[i-1],hi=cuts[i],mid=(lo+hi)*.5;
                double aa=0,ab=0;
                for (int k=0;k<3;++k)
                {
                    const double p=c.a[k]+d[k]*mid;
                    if (p<-h[k] || p>h[k]) { const double e=c.a[k]-(p<0 ? -h[k]:h[k]); aa+=d[k]*d[k]; ab+=d[k]*e; }
                }
                sample(lo); sample(hi); if (aa>0) sample(std::clamp(-ab/aa,lo,hi));
            }
            return result(pa,pb,c.radius,0);
        }
    }
    bool interacts(CollisionFilter a,CollisionFilter b)
    { return (a.category&b.mask)!=0 && (b.category&a.mask)!=0; }
    CollisionScene::CollisionScene(std::uint64_t version,std::vector<SceneObject> objects):version_(version),objects_(std::move(objects))
    {
        if (!version || objects_.size()>64) throw std::invalid_argument("Scene needs nonzero version and at most 64 objects");
        std::unordered_set<std::uint64_t> ids;
        for (auto& o:objects_)
        {
            if (!o.id || !ids.insert(o.id).second || !finite(o.position) || glm::length(o.position)>10000 ||
                !std::isfinite(o.rotation.x) || !std::isfinite(o.rotation.y) || !std::isfinite(o.rotation.z) || !std::isfinite(o.rotation.w) ||
                std::abs(glm::length(o.rotation)-1)>1e-6) throw std::invalid_argument("Invalid scene ID/metric rigid pose");
            o.rotation=glm::normalize(o.rotation);
            if (const auto* s=std::get_if<Sphere>(&o.shape); s && (!std::isfinite(s->radius) || s->radius<=0 || s->radius>100))
                throw std::invalid_argument("Invalid sphere radius");
            if (const auto* b=std::get_if<Box>(&o.shape); b && (!finite(b->halfExtent) || glm::any(glm::lessThanEqual(b->halfExtent,glm::dvec3{0})) || glm::any(glm::greaterThan(b->halfExtent,glm::dvec3{100}))))
                throw std::invalid_argument("Invalid box extent");
        }
    }
    Clearance clearance(const Capsule& a,const Capsule& b)
    {
        const auto u=a.b-a.a,v=b.b-b.a,w=a.a-b.a;
        const double aa=glm::dot(u,u),bb=glm::dot(u,v),cc=glm::dot(v,v),dd=glm::dot(u,w),ee=glm::dot(v,w),den=aa*cc-bb*bb;
        auto best=result(a.a,closest(a.a,b.a,b.b),a.radius,b.radius);
        const auto test=[&](glm::dvec3 x,glm::dvec3 y) { const auto r=result(x,y,a.radius,b.radius); if (r.gap<best.gap) best=r; };
        test(a.b,closest(a.b,b.a,b.b)); test(closest(b.a,a.a,a.b),b.a); test(closest(b.b,a.a,a.b),b.b);
        if (den>1e-20)
        { const double s=(bb*ee-cc*dd)/den,t=(aa*ee-bb*dd)/den; if (s>=0 && s<=1 && t>=0 && t<=1) test(a.a+u*s,b.a+v*t); }
        return best;
    }
    Clearance clearance(const Capsule& a,const SceneObject& b)
    {
        if (const auto* s=std::get_if<Sphere>(&b.shape)) return result(closest(b.position,a.a,a.b),b.position,a.radius,s->radius);
        if (std::holds_alternative<Plane>(b.shape))
        {
            const auto n=b.rotation*glm::dvec3{0,1,0};
            const auto p=glm::dot(a.a-b.position,n)<glm::dot(a.b-b.position,n) ? a.a:a.b;
            const double distance=glm::dot(p-b.position,n);
            return {distance-a.radius,p-n*a.radius,p-n*distance,n};
        }
        const auto inv=glm::conjugate(b.rotation);
        auto r=boxDistance({inv*(a.a-b.position),inv*(a.b-b.position),a.radius},std::get<Box>(b.shape).halfExtent);
        r.bodyPoint=b.position+b.rotation*r.bodyPoint; r.otherPoint=b.position+b.rotation*r.otherPoint; r.normal=b.rotation*r.normal;
        return r;
    }
}
