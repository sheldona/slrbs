//--------------------------------------------------------------------------------------------------
/**
@file	q3Collide.cpp

@author	Randy Gaul
@date	10/10/2014

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
		 claim that you wrote the original software. If you use this software
		 in a product, an acknowledgment in the product documentation would be
		 appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
		 be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

#include "collision/CollisionDetect.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>
#include <cmath>
#include <cassert>

using namespace Eigen;

inline bool q3TrackFaceAxis(int* axis, int n, float s, float* sMax, const Vector3f& normal, Vector3f* axisNormal)
{
    if (s > 0.0f)
        return true;

    if (s > *sMax)
    {
        *sMax = s;
        *axis = n;
        *axisNormal = normal;
    }

    return false;
}

inline bool q3TrackEdgeAxis(int* axis, int n, float s, float* sMax, const Vector3f& normal, Vector3f* axisNormal)
{
    if (s > 0.0f)
        return true;

    float l = 1.0f / normal.norm();
    s *= l;

    if (s > *sMax)
    {
        *sMax = s;
        *axis = n;
        *axisNormal = normal * l;
    }

    return false;
}

union q3FeaturePair
{
    struct
    {
        uint8_t inR;
        uint8_t outR;
        uint8_t inI;
        uint8_t outI;
    };

    int32_t key;
};

struct q3ClipVertex
{
    q3ClipVertex()
    {
        f.key = ~0;
    }

    Vector3f v;
    q3FeaturePair f;
};

void q3ComputeReferenceEdgesAndBasis(const Vector3f& eR, const Transform<float, 3, Affine>& rtx, Vector3f n, int axis, uint8_t* out, Matrix3f* basis, Vector3f* e)
{
    n = rtx.rotation().transpose() * n;

    if (axis >= 3)
        axis -= 3;

    switch (axis)
    {
    case 0:
        if (n.x() > 0.0f)
        {
            out[0] = 1;
            out[1] = 8;
            out[2] = 7;
            out[3] = 9;

            *e = Vector3f(eR.y(), eR.z(), eR.x());
            basis->row(0) = rtx.rotation().col(1);
            basis->row(1) = rtx.rotation().col(2);
            basis->row(2) = rtx.rotation().col(0);
        }
        else
        {
            out[0] = 11;
            out[1] = 3;
            out[2] = 10;
            out[3] = 5;

            *e = Vector3f(eR.z(), eR.y(), eR.x());
            basis->row(0) = rtx.rotation().col(2);
            basis->row(1) = rtx.rotation().col(1);
            basis->row(2) = -rtx.rotation().col(0);
        }
        break;

    case 1:
        if (n.y() > 0.0f)
        {
            out[0] = 0;
            out[1] = 1;
            out[2] = 2;
            out[3] = 3;

            *e = Vector3f(eR.z(), eR.x(), eR.y());
            basis->row(0) = rtx.rotation().col(2);
            basis->row(1) = rtx.rotation().col(0);
            basis->row(2) = rtx.rotation().col(1);
        }
        else
        {
            out[0] = 4;
            out[1] = 5;
            out[2] = 6;
            out[3] = 7;

            *e = Vector3f(eR.z(), eR.x(), eR.y());
            basis->row(0) = rtx.rotation().col(2);
            basis->row(1) = -rtx.rotation().col(0);
            basis->row(2) = -rtx.rotation().col(1);
        }
        break;

    case 2:
        if (n.z() > 0.0f)
        {
            out[0] = 11;
            out[1] = 4;
            out[2] = 8;
            out[3] = 0;

            *e = Vector3f(eR.y(), eR.x(), eR.z());
            basis->row(0) = -rtx.rotation().col(1);
            basis->row(1) = rtx.rotation().col(0);
            basis->row(2) = rtx.rotation().col(2);
        }
        else
        {
            out[0] = 6;
            out[1] = 10;
            out[2] = 2;
            out[3] = 9;

            *e = Vector3f(eR.y(), eR.x(), eR.z());
            basis->row(0) = -rtx.rotation().col(1);
            basis->row(1) = -rtx.rotation().col(0);
            basis->row(2) = -rtx.rotation().col(2);
        }
        break;
    }
}

void q3ComputeIncidentFace(const Transform<float, 3, Affine>& itx, const Vector3f& e, Vector3f n, q3ClipVertex* out)
{
    n = -itx.rotation().transpose() * n;
    Vector3f absN = n.cwiseAbs();

    if (absN.x() > absN.y() && absN.x() > absN.z())
    {
        if (n.x() > 0.0f)
        {
            out[0].v = Vector3f(e.x(), e.y(), -e.z());
            out[1].v = Vector3f(e.x(), e.y(), e.z());
            out[2].v = Vector3f(e.x(), -e.y(), e.z());
            out[3].v = Vector3f(e.x(), -e.y(), -e.z());

            out[0].f.inI = 9;
            out[0].f.outI = 1;
            out[1].f.inI = 1;
            out[1].f.outI = 8;
            out[2].f.inI = 8;
            out[2].f.outI = 7;
            out[3].f.inI = 7;
            out[3].f.outI = 9;
        }
        else
        {
            out[0].v = Vector3f(-e.x(), -e.y(), e.z());
            out[1].v = Vector3f(-e.x(), e.y(), e.z());
            out[2].v = Vector3f(-e.x(), e.y(), -e.z());
            out[3].v = Vector3f(-e.x(), -e.y(), -e.z());

            out[0].f.inI = 5;
            out[0].f.outI = 11;
            out[1].f.inI = 11;
            out[1].f.outI = 3;
            out[2].f.inI = 3;
            out[2].f.outI = 10;
            out[3].f.inI = 10;
            out[3].f.outI = 5;
        }
    }
    else if (absN.y() > absN.x() && absN.y() > absN.z())
    {
        if (n.y() > 0.0f)
        {
            out[0].v = Vector3f(-e.x(), e.y(), e.z());
            out[1].v = Vector3f(e.x(), e.y(), e.z());
            out[2].v = Vector3f(e.x(), e.y(), -e.z());
            out[3].v = Vector3f(-e.x(), e.y(), -e.z());

            out[0].f.inI = 3;
            out[0].f.outI = 0;
            out[1].f.inI = 0;
            out[1].f.outI = 1;
            out[2].f.inI = 1;
            out[2].f.outI = 2;
            out[3].f.inI = 2;
            out[3].f.outI = 3;
        }
        else
        {
            out[0].v = Vector3f(e.x(), -e.y(), e.z());
            out[1].v = Vector3f(-e.x(), -e.y(), e.z());
            out[2].v = Vector3f(-e.x(), -e.y(), -e.z());
            out[3].v = Vector3f(e.x(), -e.y(), -e.z());

            out[0].f.inI = 7;
            out[0].f.outI = 4;
            out[1].f.inI = 4;
            out[1].f.outI = 5;
            out[2].f.inI = 5;
            out[2].f.outI = 6;
            out[3].f.inI = 6;
            out[3].f.outI = 7;
        }
    }
    else
    {
        if (n.z() > 0.0f)
        {
            out[0].v = Vector3f(-e.x(), e.y(), e.z());
            out[1].v = Vector3f(-e.x(), -e.y(), e.z());
            out[2].v = Vector3f(e.x(), -e.y(), e.z());
            out[3].v = Vector3f(e.x(), e.y(), e.z());

            out[0].f.inI = 0;
            out[0].f.outI = 11;
            out[1].f.inI = 11;
            out[1].f.outI = 4;
            out[2].f.inI = 4;
            out[2].f.outI = 8;
            out[3].f.inI = 8;
            out[3].f.outI = 0;
        }
        else
        {
            out[0].v = Vector3f(e.x(), -e.y(), -e.z());
            out[1].v = Vector3f(-e.x(), -e.y(), -e.z());
            out[2].v = Vector3f(-e.x(), e.y(), -e.z());
            out[3].v = Vector3f(e.x(), e.y(), -e.z());

            out[0].f.inI = 9;
            out[0].f.outI = 6;
            out[1].f.inI = 6;
            out[1].f.outI = 10;
            out[2].f.inI = 10;
            out[2].f.outI = 2;
            out[3].f.inI = 2;
            out[3].f.outI = 9;
        }
    }

    for (int i = 0; i < 4; ++i)
        out[i].v = itx * out[i].v;
}

#define InFront(a) \
    ((a) < 0.0f)

#define Behind(a) \
    ((a) >= 0.0f)

#define On(a) \
    ((a) < 0.005f && (a) > -0.005f)

int q3Orthographic(float sign, float e, int axis, int clipEdge, q3ClipVertex* in, int inCount, q3ClipVertex* out)
{
    int outCount = 0;
    q3ClipVertex a = in[inCount - 1];

    for (int i = 0; i < inCount; ++i)
    {
        q3ClipVertex b = in[i];

        float da = sign * a.v[axis] - e;
        float db = sign * b.v[axis] - e;

        q3ClipVertex cv;

        // B
        if (((InFront(da) && InFront(db)) || On(da) || On(db)))
        {
            assert(outCount < 8);
            out[outCount++] = b;
        }

        // I
        else if (InFront(da) && Behind(db))
        {
            cv.f = b.f;
            cv.v = a.v + (b.v - a.v) * (da / (da - db));
            cv.f.outR = clipEdge;
            cv.f.outI = 0;
            assert(outCount < 8);
            out[outCount++] = cv;
        }

        // I, B
        else if (Behind(da) && InFront(db))
        {
            cv.f = a.f;
            cv.v = a.v + (b.v - a.v) * (da / (da - db));
            cv.f.inR = clipEdge;
            cv.f.inI = 0;
            assert(outCount < 8);
            out[outCount++] = cv;

            assert(outCount < 8);
            out[outCount++] = b;
        }

        a = b;
    }

    return outCount;
}

int q3Clip(const Vector3f& rPos, const Vector3f& e, uint8_t* clipEdges, const Matrix3f& basis, q3ClipVertex* incident, q3ClipVertex* outVerts, float* outDepths)
{
    int inCount = 4;
    int outCount;
    q3ClipVertex in[8];
    q3ClipVertex out[8];

    for (int i = 0; i < 4; ++i)
        in[i].v = basis.transpose() * (incident[i].v - rPos);

    outCount = q3Orthographic(1.0f, e.x(), 0, clipEdges[0], in, inCount, out);

    if (!outCount)
        return 0;

    inCount = q3Orthographic(1.0f, e.y(), 1, clipEdges[1], out, outCount, in);

    if (!inCount)
        return 0;

    outCount = q3Orthographic(-1.0f, e.x(), 0, clipEdges[2], in, inCount, out);

    if (!outCount)
        return 0;

    inCount = q3Orthographic(-1.0f, e.y(), 1, clipEdges[3], out, outCount, in);

    outCount = 0;
    for (int i = 0; i < inCount; ++i)
    {
        float d = in[i].v.z() - e.z();

        if (d <= 0.0f)
        {
            outVerts[outCount].v = basis * in[i].v + rPos;
            outVerts[outCount].f = in[i].f;
            outDepths[outCount++] = d;
        }
    }

    assert(outCount <= 8);

    return outCount;
}

inline void q3EdgesContact(Vector3f* CA, Vector3f* CB, const Vector3f& PA, const Vector3f& QA, const Vector3f& PB, const Vector3f& QB)
{
    Vector3f DA = QA - PA;
    Vector3f DB = QB - PB;
    Vector3f r = PA - PB;
    float a = DA.dot(DA);
    float e = DB.dot(DB);
    float f = DB.dot(r);
    float c = DA.dot(r);

    float b = DA.dot(DB);
    float denom = a * e - b * b;

    float TA = (b * f - c * e) / denom;
    float TB = (b * TA + f) / e;

    *CA = PA + DA * TA;
    *CB = PB + DB * TB;
}

void q3SupportEdge(const Transform<float, 3, Affine>& tx, const Vector3f& e, Vector3f n, Vector3f* aOut, Vector3f* bOut)
{
    n = tx.rotation().transpose() * n;
    Vector3f absN = n.cwiseAbs();
    Vector3f a, b;

    if (absN.x() > absN.y())
    {
        if (absN.y() > absN.z())
        {
            a = Vector3f(e.x(), e.y(), e.z());
            b = Vector3f(e.x(), e.y(), -e.z());
        }
        else
        {
            a = Vector3f(e.x(), e.y(), e.z());
            b = Vector3f(e.x(), -e.y(), e.z());
        }
    }
    else
    {
        if (absN.x() > absN.z())
        {
            a = Vector3f(e.x(), e.y(), e.z());
            b = Vector3f(e.x(), e.y(), -e.z());
        }
        else
        {
            a = Vector3f(e.x(), e.y(), e.z());
            b = Vector3f(-e.x(), e.y(), e.z());
        }
    }

    float signx = n.x() >= 0.0f ? 1.0f : -1.0f;
    float signy = n.y() >= 0.0f ? 1.0f : -1.0f;
    float signz = n.z() >= 0.0f ? 1.0f : -1.0f;

    a.x() *= signx;
    a.y() *= signy;
    a.z() *= signz;
    b.x() *= signx;
    b.y() *= signy;
    b.z() *= signz;

    *aOut = tx * a;
    *bOut = tx * b;
}

void CollisionDetect::collisionDetectBoxBox(RigidBody* body0, RigidBody* body1)
{
    Box* boxA = dynamic_cast<Box*>(body0->geometry.get());
    Box* boxB = dynamic_cast<Box*>(body1->geometry.get());

    Transform<float, 3, Affine> transformA = Transform<float, 3, Affine>::Identity();
    transformA.translate(body0->x);
    transformA.rotate(body0->q);

    Transform<float, 3, Affine> transformB = Transform<float, 3, Affine>::Identity();
    transformB.translate(body1->x);
    transformB.rotate(body1->q);

    Transform<float, 3, Affine> atx = transformA;
    Transform<float, 3, Affine> btx = transformB;
    Transform<float, 3, Affine> aL = Transform<float, 3, Affine>::Identity();
    Transform<float, 3, Affine> bL = Transform<float, 3, Affine>::Identity();
    atx = atx * aL;
    btx = btx * bL;
    Vector3f eA = boxA->dim * 0.5f;
    Vector3f eB = boxB->dim * 0.5f;

    Matrix3f C = atx.rotation().transpose() * btx.rotation();

    Matrix3f absC;
    bool parallel = false;
    const float kCosTol = 1.0e-6f;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            float val = std::abs(C(i, j));
            absC(i, j) = val;

            if (val + kCosTol >= 1.0f)
                parallel = true;
        }
    }

    Vector3f t = atx.rotation().transpose() * (btx.translation() - atx.translation());

    float s;
    float aMax = -std::numeric_limits<float>::max();
    float bMax = -std::numeric_limits<float>::max();
    float eMax = -std::numeric_limits<float>::max();
    int aAxis = ~0;
    int bAxis = ~0;
    int eAxis = ~0;
    Vector3f nA;
    Vector3f nB;
    Vector3f nE;

    s = std::abs(t.x()) - (eA.x() + absC.col(0).dot(eB));
    if (q3TrackFaceAxis(&aAxis, 0, s, &aMax, atx.rotation().col(0), &nA))
        return;

    s = std::abs(t.y()) - (eA.y() + absC.col(1).dot(eB));
    if (q3TrackFaceAxis(&aAxis, 1, s, &aMax, atx.rotation().col(1), &nA))
        return;

    s = std::abs(t.z()) - (eA.z() + absC.col(2).dot(eB));
    if (q3TrackFaceAxis(&aAxis, 2, s, &aMax, atx.rotation().col(2), &nA))
        return;

    s = std::abs(t.dot(C.col(0))) - (eB.x() + absC.row(0).dot(eA));
    if (q3TrackFaceAxis(&bAxis, 3, s, &bMax, btx.rotation().col(0), &nB))
        return;

    s = std::abs(t.dot(C.col(1))) - (eB.y() + absC.row(1).dot(eA));
    if (q3TrackFaceAxis(&bAxis, 4, s, &bMax, btx.rotation().col(1), &nB))
        return;

    s = std::abs(t.dot(C.col(2))) - (eB.z() + absC.row(2).dot(eA));
    if (q3TrackFaceAxis(&bAxis, 5, s, &bMax, btx.rotation().col(2), &nB))
        return;

    if (!parallel)
    {
        float rA;
        float rB;

        rA = eA.y() * absC(0, 2) + eA.z() * absC(0, 1);
        rB = eB.y() * absC(2, 0) + eB.z() * absC(1, 0);
        s = std::abs(t.z() * C(0, 1) - t.y() * C(0, 2)) - (rA + rB);
        if (q3TrackEdgeAxis(&eAxis, 6, s, &eMax, Vector3f(0.0f, -C(0, 2), C(0, 1)), &nE))
            return;

        rA = eA.y() * absC(1, 2) + eA.z() * absC(1, 1);
        rB = eB.x() * absC(2, 0) + eB.z() * absC(0, 0);
        s = std::abs(t.z() * C(1, 1) - t.y() * C(1, 2)) - (rA + rB);
        if (q3TrackEdgeAxis(&eAxis, 7, s, &eMax, Vector3f(0.0f, -C(1, 2), C(1, 1)), &nE))
            return;

        rA = eA.y() * absC(2, 2) + eA.z() * absC(2, 1);
        rB = eB.x() * absC(1, 0) + eB.y() * absC(0, 0);
        s = std::abs(t.z() * C(2, 1) - t.y() * C(2, 2)) - (rA + rB);
        if (q3TrackEdgeAxis(&eAxis, 8, s, &eMax, Vector3f(0.0f, -C(2, 2), C(2, 1)), &nE))
            return;

        rA = eA.x() * absC(0, 2) + eA.z() * absC(0, 0);
        rB = eB.y() * absC(2, 1) + eB.z() * absC(1, 1);
        s = std::abs(t.x() * C(0, 2) - t.z() * C(0, 0)) - (rA + rB);
        if (q3TrackEdgeAxis(&eAxis, 9, s, &eMax, Vector3f(C(0, 2), 0.0f, -C(0, 0)), &nE))
            return;

        rA = eA.x() * absC(1, 2) + eA.z() * absC(1, 0);
        rB = eB.x() * absC(2, 1) + eB.z() * absC(0, 1);
        s = std::abs(t.x() * C(1, 2) - t.z() * C(1, 0)) - (rA + rB);
        if (q3TrackEdgeAxis(&eAxis, 10, s, &eMax, Vector3f(C(1, 2), 0.0f, -C(1, 0)), &nE))
            return;

        rA = eA.x() * absC(2, 2) + eA.z() * absC(2, 0);
        rB = eB.x() * absC(1, 1) + eB.y() * absC(0, 1);
        s = std::abs(t.x() * C(2, 2) - t.z() * C(2, 0)) - (rA + rB);
        if (q3TrackEdgeAxis(&eAxis, 11, s, &eMax, Vector3f(C(2, 2), 0.0f, -C(2, 0)), &nE))
            return;

        rA = eA.x() * absC(0, 1) + eA.y() * absC(0, 0);
        rB = eB.y() * absC(2, 2) + eB.z() * absC(1, 2);
        s = std::abs(t.y() * C(0, 0) - t.x() * C(0, 1)) - (rA + rB);
        if (q3TrackEdgeAxis(&eAxis, 12, s, &eMax, Vector3f(-C(0, 1), C(0, 0), 0.0f), &nE))
            return;

        rA = eA.x() * absC(1, 1) + eA.y() * absC(1, 0);
        rB = eB.x() * absC(2, 2) + eB.z() * absC(0, 2);
        s = std::abs(t.y() * C(1, 0) - t.x() * C(1, 1)) - (rA + rB);
        if (q3TrackEdgeAxis(&eAxis, 13, s, &eMax, Vector3f(-C(1, 1), C(1, 0), 0.0f), &nE))
            return;

        rA = eA.x() * absC(2, 1) + eA.y() * absC(2, 0);
        rB = eB.x() * absC(1, 2) + eB.y() * absC(0, 2);
        s = std::abs(t.y() * C(2, 0) - t.x() * C(2, 1)) - (rA + rB);
        if (q3TrackEdgeAxis(&eAxis, 14, s, &eMax, Vector3f(-C(2, 1), C(2, 0), 0.0f), &nE))
            return;
    }

    const float kRelTol = 0.95f;
    const float kAbsTol = 0.01f;
    int axis;
    float sMax;
    Vector3f n;
    float faceMax = std::max(aMax, bMax);
    if (kRelTol * eMax > faceMax + kAbsTol)
    {
        axis = eAxis;
        sMax = eMax;
        n = nE;
    }
    else
    {
        if (kRelTol * bMax > aMax + kAbsTol)
        {
            axis = bAxis;
            sMax = bMax;
            n = nB;
        }
        else
        {
            axis = aAxis;
            sMax = aMax;
            n = nA;
        }
    }

    if (n.dot(btx.translation() - atx.translation()) < 0.0f)
        n = -n;

    if (axis == ~0)
        return;

    if (axis < 6)
    {
        Transform<float, 3, Affine> rtx;
        Transform<float, 3, Affine> itx;
        Vector3f eR;
        Vector3f eI;
        bool flip;

        if (axis < 3)
        {
            rtx = atx;
            itx = btx;
            eR = eA;
            eI = eB;
            flip = false;
        }
        else
        {
            rtx = btx;
            itx = atx;
            eR = eB;
            eI = eA;
            flip = true;
            n = -n;
        }

        q3ClipVertex incident[4];
        q3ComputeIncidentFace(itx, eI, n, incident);
        uint8_t clipEdges[4];
        Matrix3f basis;
        Vector3f e;
        q3ComputeReferenceEdgesAndBasis(eR, rtx, n, axis, clipEdges, &basis, &e);

        q3ClipVertex out[8];
        float depths[8];
        int outNum;
        outNum = q3Clip(rtx.translation(), e, clipEdges, basis, incident, out, depths);

        if (outNum)
        {
            Vector3f normal = flip ? -n : n;

            for (int i = 0; i < outNum; ++i)
            {
                q3FeaturePair pair = out[i].f;

                if (flip)
                {
                    std::swap(pair.inI, pair.inR);
                    std::swap(pair.outI, pair.outR);
                }

			    m_contacts.push_back(new Contact(body0, body1, out[i].v, normal, depths[i]));
            }
        }
    }
    else
    {
        n = atx.rotation() * n;

        if (n.dot(btx.translation() - atx.translation()) < 0.0f)
            n = -n;

        Vector3f PA, QA;
        Vector3f PB, QB;
        q3SupportEdge(atx, eA, n, &PA, &QA);
        q3SupportEdge(btx, eB, -n, &PB, &QB);

        Vector3f CA, CB;
        q3EdgesContact(&CA, &CB, PA, QA, PB, QB);

        q3FeaturePair pair;
        pair.key = axis;

        m_contacts.push_back(new Contact(body0, body1, 0.5f * (CA + CB), n, sMax));
    }
}
