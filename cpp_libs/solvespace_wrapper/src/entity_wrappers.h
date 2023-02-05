#ifndef ENTITY_WRAPPERS_H
#define ENTITY_WRAPPERS_H

#include "slvs_includes.h"
#include "util.h"
#include "param_wrappers.h"
#include "solvespace_wrapper.h"
#include "Allocator.h"
#include <vector>
#include <memory>

namespace slvswrap {


    class EntityWrapper : public Allocatable<Slvs_Entity> {

        std::optional<Slvs_hGroup> group = std::nullopt;
    protected:

        static void serializeParamWrapper(std::ostream& os,
                                          std::weak_ptr<ParamWrapper> pw) {
            if (pw.expired()) {
                os << "UNASSIGNED";
                return;
            }

            std::shared_ptr<ParamWrapper> pw_shared = pw.lock();

            tReal initValue = pw_shared->getInitialValue();

            os << "{ initValue: " << initValue;
            os << ", computedValue: ";

            if (pw_shared->hasComputedValue()) {
                os << pw_shared->getComputedValue();
            } else {
                os << "N/A";
            }

            os << " }";
        }

        using entity_param = Alternate<std::weak_ptr<ParamWrapper>, std::weak_ptr<EntityWrapper>>;

        std::vector<entity_param> parameters;

        EntityWrapper() {

        }

        explicit EntityWrapper(const Slvs_hGroup& g) : group(g) {

        }

        Slvs_hGroup getGroup() {
            if (!group.has_value()) {
                throw std::runtime_error("Group field has not been set.");
            }

            return group.value();
        }

    public:

        /**
         * Recursively set prioritization of parameters.
         */
        void setPrioritized(bool isPrioritized) {
            for (auto &ep: parameters) {
                if (ep.hasFirst()) {
                    ep.getFirst().lock()->setPrioritized(isPrioritized);
                } else {
                    ep.getSecond().lock()->setPrioritized(isPrioritized);
                }
            }
        }
    };


    class PointWrapper : public EntityWrapper {

    public:
        PointWrapper(Allocator &alloc,
                     const Slvs_hGroup &g,
                     tReal x,
                     tReal y,
                     tReal z) : EntityWrapper(g) {
            parameters.push_back(entity_param::with_first(
                    alloc.requestParamWrapper(g, x)));

            parameters.push_back(entity_param::with_first(
                    alloc.requestParamWrapper(g, y)));

            parameters.push_back(entity_param::with_first(
                    alloc.requestParamWrapper(g, z)));
        }

        std::tuple<tReal, tReal, tReal> xyz() {
            return {x()->getComputedValue(), y()->getComputedValue(), z()->getComputedValue()};
        }

        std::shared_ptr<ParamWrapper> x() {
            return parameters[0].getFirst().lock();
        };

        std::shared_ptr<ParamWrapper> y() {
            return parameters[1].getFirst().lock();
        };

        std::shared_ptr<ParamWrapper> z() {
            return parameters[2].getFirst().lock();
        };

        Slvs_Entity applyImpl() override {
            auto x_id = x()->getId();
            auto y_id = y()->getId();
            auto z_id = z()->getId();

            return Slvs_MakePoint3d(
                    getId(),
                    getGroup(),
                    x_id, y_id, z_id);
        }

        friend std::ostream& operator<<(std::ostream& os, const PointWrapper& pnt) {
            os << "Point3D(";
            pnt.serializeId(os);
            os << "), { x=";
            EntityWrapper::serializeParamWrapper(os, pnt.parameters[0].getFirst());
            os << " y=";
            EntityWrapper::serializeParamWrapper(os, pnt.parameters[1].getFirst());
            os << " z=";
            EntityWrapper::serializeParamWrapper(os, pnt.parameters[2].getFirst());
            os << " }";
            return os;
        }
    };

    class LineWrapper : public EntityWrapper {

        std::weak_ptr<PointWrapper> _p0;

        std::weak_ptr<PointWrapper> _p1;

    public:
        LineWrapper(Allocator &alloc,
                    const Slvs_hGroup &g,
                    std::weak_ptr<PointWrapper> &p0,
                    std::weak_ptr<PointWrapper> &p1) : _p0(p0), _p1(p1), EntityWrapper(g){
            parameters.push_back(entity_param::with_second(p0));
            parameters.push_back(entity_param::with_second(p1));
        }

        std::shared_ptr<PointWrapper> p0() {
            return _p0.lock();
        };

        std::shared_ptr<PointWrapper> p1() {
            return _p1.lock();
        };

        Slvs_Entity applyImpl() override {
            auto p0_id = p0()->getId();
            auto p1_id = p1()->getId();

            return Slvs_MakeLineSegment(
                    getId(),
                    getGroup(),
                    SLVS_FREE_IN_3D,
                    p0_id,
                    p1_id);
        }

        friend std::ostream& operator<<(std::ostream& os, const LineWrapper& ln) {
            os << "Line3D(";
            ln.serializeId(os);
            os << "), { p0=" << *ln._p0.lock() << ", p1= " << *ln._p1.lock() << " }";

            return os;
        }
    };

    class Normal3DWrapper : public EntityWrapper {

        std::weak_ptr<ParamWrapper> _qw;
        std::weak_ptr<ParamWrapper> _qx;
        std::weak_ptr<ParamWrapper> _qy;
        std::weak_ptr<ParamWrapper> _qz;

    public:

        std::weak_ptr<ParamWrapper> qw() {
            return this->_qw;
        }

        std::weak_ptr<ParamWrapper> qx() {
            return this->_qx;
        }

        std::weak_ptr<ParamWrapper> qy() {
            return this->_qy;
        }

        std::weak_ptr<ParamWrapper> qz() {
            return this->_qz;
        }

        Normal3DWrapper(Allocator& alloc,
                        Slvs_hGroup group,
                        std::weak_ptr<ParamWrapper> qw,
                        std::weak_ptr<ParamWrapper> qx,
                        std::weak_ptr<ParamWrapper> qy,
                        std::weak_ptr<ParamWrapper> qz) :
                        EntityWrapper(group),
                        _qw(qw),
                        _qx(qx),
                        _qy(qy),
                        _qz(qz)
            {

            parameters.push_back(entity_param::with_first(_qw));
            parameters.push_back(entity_param::with_first(_qx));
            parameters.push_back(entity_param::with_first(_qy));
            parameters.push_back(entity_param::with_first(_qz));
        }

        Normal3DWrapper(Allocator& alloc,
                        const Slvs_hGroup& group,
                        tReal ux, tReal uy, tReal uz,
                        tReal vx, tReal vy, tReal vz) : EntityWrapper(group) {

            tReal qw_val;
            tReal qx_val;
            tReal qy_val;
            tReal qz_val;

            Slvs_MakeQuaternion(ux, uy, uz,
                                vx, vy, vz,
                                &qw_val, &qx_val, &qy_val, &qz_val);

            this->_qw = alloc.requestParamWrapper(group, qw_val);
            this->_qx = alloc.requestParamWrapper(group, qx_val);
            this->_qy = alloc.requestParamWrapper(group, qy_val);
            this->_qz = alloc.requestParamWrapper(group, qz_val);

            parameters.push_back(entity_param::with_first(this->_qw));
            parameters.push_back(entity_param::with_first(this->_qx));
            parameters.push_back(entity_param::with_first(this->_qy));
            parameters.push_back(entity_param::with_first(this->_qz));
        }

        Slvs_Entity applyImpl() override {
            auto qw_id = this->_qw.lock()->getId();
            auto qx_id = this->_qx.lock()->getId();
            auto qy_id = this->_qy.lock()->getId();
            auto qz_id = this->_qz.lock()->getId();

            return Slvs_MakeNormal3d(this->getId(), this->getGroup(), qw_id, qx_id, qy_id, qz_id);
        }

        friend std::ostream& operator<<(std::ostream& os, const Normal3DWrapper& n) {
            os << "{ Normal3D(";
            n.serializeId(os);
            os << ") qw=";
            EntityWrapper::serializeParamWrapper(os, n._qw);
            os << ", qx=";
            EntityWrapper::serializeParamWrapper(os, n._qx);
            os << ", qy=";
            EntityWrapper::serializeParamWrapper(os, n._qy);
            os << ", qz=";
            EntityWrapper::serializeParamWrapper(os, n._qz);
            os << " }";

            return os;
        }
    };

    /**
     * Shorthand for creating the standard slvs workplane
     */
    class WorkPlaneWrapper : public EntityWrapper {
    private:
        std::weak_ptr<PointWrapper> _origin;
        std::weak_ptr<Normal3DWrapper> _norm3d;

    public:
        std::weak_ptr<PointWrapper> origin() {
            return this->_origin;
        }

        std::weak_ptr<Normal3DWrapper> norm3d() {
            return this->_norm3d;
        }

        WorkPlaneWrapper(Allocator& alloc,
                         Slvs_hGroup group,
                         tReal ox, tReal oy, tReal oz,
                         tReal ux, tReal uy, tReal uz,
                         tReal vx, tReal vy, tReal vz) :
                            WorkPlaneWrapper(
                                    alloc,
                                    group,
                                    alloc.requestEntityWrapper<PointWrapper>(group, ox, oy, oz),
                                    alloc.requestEntityWrapper<Normal3DWrapper>(group, ux, uy, uz, vx, vy, vz)) {

        }


        WorkPlaneWrapper(Allocator& alloc,
                         Slvs_hGroup group,
                         std::weak_ptr<PointWrapper> origin,
                         std::weak_ptr<Normal3DWrapper> norm3d) : _origin(origin), _norm3d(norm3d), EntityWrapper(group) {

            parameters.push_back(entity_param::with_second(_origin));
            parameters.push_back(entity_param::with_second(_norm3d));
        }

        Slvs_Entity applyImpl() override {
            auto origin_id = this->_origin.lock()->getId();
            auto norm3d_id = this->_norm3d.lock()->getId();

            return Slvs_MakeWorkplane(this->getId(), this->getGroup(), origin_id, norm3d_id);
        }

        friend std::ostream& operator<<(std::ostream& os, const WorkPlaneWrapper& wp) {
            os << "{ WorkPlane(";
            wp.serializeId(os);
            os << ") origin=" << *wp._origin.lock() << ", norm3d=" << *wp._norm3d.lock() << " }";

            return os;
        }
    };

    class Point2DWrapper : public EntityWrapper {

        std::weak_ptr<ParamWrapper> _u;
        std::weak_ptr<ParamWrapper> _v;

        std::weak_ptr<WorkPlaneWrapper> _wp;

    public:

        Point2DWrapper(Allocator& alloc,
                       Slvs_hGroup group,
                       std::weak_ptr<WorkPlaneWrapper> workplane,
                       tReal u,
                       tReal v) :                                   EntityWrapper(group),
                                                                    _u(alloc.requestParamWrapper(group, u)),
                                                                    _v(alloc.requestParamWrapper(group, v)),
                                                                    _wp(workplane) {

            parameters.push_back(entity_param::with_first(_u));
            parameters.push_back(entity_param::with_first(_v));

            parameters.push_back(entity_param::with_second(_wp));
        }

        Slvs_Entity applyImpl() override {
            return Slvs_MakePoint2d(this->getId(),
                this->getGroup(),
                this->_wp.lock()->getId(),
                this->_u.lock()->getId(),
                this->_v.lock()->getId());
        }

        friend std::ostream& operator<<(std::ostream& os, const Point2DWrapper& wp) {
            os << "{ Point2D(";
            wp.serializeId(os);
            os << ") u=";
            EntityWrapper::serializeParamWrapper(os, wp._u);
            os << ", v=";
            EntityWrapper::serializeParamWrapper(os, wp._v);
            os << ", workplane=" << *wp._wp.lock() << " }";

            return os;
        }
    };
}

#endif