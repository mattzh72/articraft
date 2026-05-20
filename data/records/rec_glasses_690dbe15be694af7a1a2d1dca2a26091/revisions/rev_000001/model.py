from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ExtrudeGeometry,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


def _oval_points(cx: float, cz: float, rx: float, rz: float, count: int = 28):
    return [
        (
            cx + rx * math.cos(2.0 * math.pi * i / count),
            0.0,
            cz + rz * math.sin(2.0 * math.pi * i / count),
        )
        for i in range(count)
    ]


def _tube(points, radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=6,
            closed_spline=True,
            radial_segments=16,
            cap_ends=False,
        ),
        name,
    )


def _open_tube(points, radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=10,
            closed_spline=False,
            radial_segments=14,
            cap_ends=True,
        ),
        name,
    )


def _lens_mesh(width: float, height: float, thickness: float, name: str):
    lens = ExtrudeGeometry(
        superellipse_profile(width, height, exponent=2.25, segments=64),
        thickness,
        cap=True,
        center=True,
    )
    # The extrusion helper makes thickness along local Z; eyeglass lenses sit in
    # the front-frame XZ plane, so rotate the thin solid to put thickness on Y.
    lens.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(lens, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_wire_rim_glasses")

    polished_metal = model.material("polished_metal", rgba=(0.72, 0.63, 0.45, 1.0))
    clear_lens = model.material("clear_lens", rgba=(0.76, 0.90, 1.0, 0.32))
    silicone = model.material("soft_clear_silicone", rgba=(0.96, 0.94, 0.88, 0.68))

    frame = model.part("front_frame")

    rim_radius = 0.00115
    lens_rx = 0.028
    lens_rz = 0.020
    left_cx = -0.038
    right_cx = 0.038
    hinge_x = 0.069

    frame.visual(
        _tube(_oval_points(left_cx, 0.0, lens_rx, lens_rz), rim_radius, "left_wire_rim"),
        material=polished_metal,
        name="left_wire_rim",
    )
    frame.visual(
        _tube(_oval_points(right_cx, 0.0, lens_rx, lens_rz), rim_radius, "right_wire_rim"),
        material=polished_metal,
        name="right_wire_rim",
    )
    frame.visual(
        _lens_mesh(0.052, 0.037, 0.0012, "left_lens"),
        origin=Origin(xyz=(left_cx, 0.0, 0.0)),
        material=clear_lens,
        name="left_lens",
    )
    frame.visual(
        _lens_mesh(0.052, 0.037, 0.0012, "right_lens"),
        origin=Origin(xyz=(right_cx, 0.0, 0.0)),
        material=clear_lens,
        name="right_lens",
    )

    # A rigid arched wire bridge joins the two rims above the nose.
    frame.visual(
        _open_tube(
            [
                (left_cx + lens_rx - 0.001, 0.0, 0.004),
                (-0.014, -0.0010, 0.015),
                (0.0, -0.0016, 0.018),
                (0.014, -0.0010, 0.015),
                (right_cx - lens_rx + 0.001, 0.0, 0.004),
            ],
            rim_radius,
            "arched_bridge",
        ),
        material=polished_metal,
        name="arched_bridge",
    )

    # Short soldered lugs connect the rims to the hinge knuckles.
    for side, cx in (("left", -1.0), ("right", 1.0)):
        sx = cx
        frame.visual(
            _open_tube(
                [
                    (sx * (abs(right_cx) + lens_rx - 0.001), 0.0, 0.0),
                    (sx * 0.067, -0.0008, 0.0),
                    (sx * hinge_x, -0.0012, 0.0),
                ],
                rim_radius,
                f"{side}_hinge_lug",
            ),
            material=polished_metal,
            name=f"{side}_hinge_lug",
        )
        # Frame-side alternating knuckles, attached to a tiny vertical hinge leaf.
        frame.visual(
            Box((0.0035, 0.0014, 0.031)),
            origin=Origin(xyz=(sx * (hinge_x - 0.0015), 0.0008, 0.0)),
            material=polished_metal,
            name=f"{side}_fixed_leaf",
        )
        if side == "left":
            frame.visual(
                Cylinder(radius=0.0018, length=0.008),
                origin=Origin(xyz=(sx * hinge_x, 0.0, -0.011)),
                material=polished_metal,
                name="left_fixed_knuckle_lower",
            )
            frame.visual(
                Cylinder(radius=0.0018, length=0.008),
                origin=Origin(xyz=(sx * hinge_x, 0.0, 0.011)),
                material=polished_metal,
                name="left_fixed_knuckle_upper",
            )
        else:
            frame.visual(
                Cylinder(radius=0.0018, length=0.008),
                origin=Origin(xyz=(sx * hinge_x, 0.0, -0.011)),
                material=polished_metal,
                name="right_fixed_knuckle_lower",
            )
            frame.visual(
                Cylinder(radius=0.0018, length=0.008),
                origin=Origin(xyz=(sx * hinge_x, 0.0, 0.011)),
                material=polished_metal,
                name="right_fixed_knuckle_upper",
            )

    # Nose-pad wire brackets curve off the bridge, with soft translucent pads.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            _open_tube(
                [
                    (sx * 0.014, -0.0010, 0.015),
                    (sx * 0.013, -0.0065, 0.004),
                    (sx * 0.012, -0.0115, -0.006),
                ],
                0.00055,
                f"{side}_nose_bracket",
            ),
            material=polished_metal,
            name=f"{side}_nose_bracket",
        )
        frame.visual(
            Box((0.007, 0.0020, 0.011)),
            origin=Origin(xyz=(sx * 0.012, -0.012, -0.007), rpy=(0.18, 0.0, sx * 0.25)),
            material=silicone,
            name=f"{side}_nose_pad",
        )

    def make_temple(name: str, sx: float):
        temple = model.part(name)
        temple.visual(
            Cylinder(radius=0.00165, length=0.011),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=polished_metal,
            name="moving_knuckle",
        )
        temple.visual(
            Box((0.0042, 0.010, 0.007)),
            origin=Origin(xyz=(0.0, -0.0048, 0.0)),
            material=polished_metal,
            name="moving_leaf",
        )
        temple.visual(
            _open_tube(
                [
                    (0.0, -0.006, 0.0),
                    (sx * 0.004, -0.036, 0.002),
                    (sx * 0.003, -0.090, -0.002),
                    (-sx * 0.004, -0.118, -0.010),
                    (-sx * 0.010, -0.132, -0.018),
                ],
                0.00105,
                f"{name}_curved_arm",
            ),
            material=polished_metal,
            name="curved_arm",
        )
        return temple

    left_temple = make_temple("left_temple", -1.0)
    right_temple = make_temple("right_temple", 1.0)

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_temple,
        origin=Origin(xyz=(-hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_temple,
        origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left = object_model.get_part("left_temple")
    right = object_model.get_part("right_temple")
    frame = object_model.get_part("front_frame")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")

    ctx.expect_contact(
        frame,
        left,
        elem_a="left_fixed_knuckle_lower",
        elem_b="moving_knuckle",
        contact_tol=0.003,
        name="left hinge knuckles share a pin line",
    )
    ctx.expect_contact(
        frame,
        right,
        elem_a="right_fixed_knuckle_lower",
        elem_b="moving_knuckle",
        contact_tol=0.003,
        name="right hinge knuckles share a pin line",
    )

    left_rest = ctx.part_world_aabb(left)
    right_rest = ctx.part_world_aabb(right)
    ctx.check(
        "temple arms extend rearward when unfolded",
        left_rest is not None
        and right_rest is not None
        and left_rest[0][1] < -0.11
        and right_rest[0][1] < -0.11,
        details=f"left={left_rest}, right={right_rest}",
    )

    with ctx.pose({left_hinge: 1.45, right_hinge: 1.45}):
        left_folded = ctx.part_world_aabb(left)
        right_folded = ctx.part_world_aabb(right)
        ctx.check(
            "temple arms fold inward across the frame",
            left_folded is not None
            and right_folded is not None
            and left_folded[1][0] > 0.030
            and right_folded[0][0] < -0.030,
            details=f"left={left_folded}, right={right_folded}",
        )
        ctx.check(
            "folded arms lie nearly flat behind the rims",
            left_folded is not None
            and right_folded is not None
            and (left_folded[1][1] - left_folded[0][1]) < 0.050
            and (right_folded[1][1] - right_folded[0][1]) < 0.050,
            details=f"left={left_folded}, right={right_folded}",
        )

    return ctx.report()


object_model = build_object_model()
