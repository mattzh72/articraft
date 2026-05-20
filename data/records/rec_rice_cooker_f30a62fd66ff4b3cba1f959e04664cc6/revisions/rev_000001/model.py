from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    superellipse_side_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_oval_rice_cooker")

    warm_white = model.material("warm_white_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    soft_gray = model.material("soft_gray_plastic", rgba=(0.72, 0.72, 0.68, 1.0))
    black = model.material("gloss_black", rgba=(0.02, 0.023, 0.026, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    display_green = model.material("display_green", rgba=(0.12, 0.90, 0.50, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.82, 0.82, 0.78, 1.0))

    body = model.part("body")

    body_shell_geom = superellipse_side_loft(
        [
            (-0.200, 0.020, 0.145, 0.260),
            (-0.145, 0.008, 0.185, 0.410),
            (-0.030, 0.000, 0.205, 0.470),
            (0.105, 0.006, 0.190, 0.440),
            (0.205, 0.020, 0.135, 0.270),
        ],
        exponents=3.0,
        segments=64,
    )
    body.visual(
        mesh_from_geometry(body_shell_geom, "body_oval_shell"),
        material=warm_white,
        name="body_shell",
    )

    rim_geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.420, 0.320, exponent=3.0, segments=64),
        [superellipse_profile(0.300, 0.205, exponent=2.6, segments=64)],
        0.012,
    )
    body.visual(
        mesh_from_geometry(rim_geom, "top_rim"),
        origin=Origin(xyz=(0.0, -0.010, 0.200)),
        material=soft_gray,
        name="top_rim",
    )

    pot_geom = LatheGeometry.from_shell_profiles(
        [(0.055, 0.045), (0.120, 0.060), (0.147, 0.165), (0.153, 0.194)],
        [(0.042, 0.054), (0.105, 0.067), (0.132, 0.163), (0.138, 0.188)],
        segments=72,
        end_cap="round",
        lip_samples=8,
    )
    body.visual(
        mesh_from_geometry(pot_geom, "inner_pot"),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=stainless,
        name="inner_pot",
    )

    # A smooth touch-display lens keeps the appliance modern without adding
    # separate mechanical buttons.
    body.visual(
        Box((0.160, 0.006, 0.060)),
        origin=Origin(xyz=(0.0, 0.203, 0.105)),
        material=black,
        name="front_lens",
    )
    body.visual(
        Box((0.076, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, 0.207, 0.115)),
        material=display_green,
        name="status_window",
    )

    for i, (x, y) in enumerate(((-0.145, -0.105), (0.145, -0.105), (-0.135, 0.120), (0.135, 0.120))):
        body.visual(
            Cylinder(radius=0.028, length=0.020),
            origin=Origin(xyz=(x, y, 0.002)),
            material=dark_rubber,
            name=f"foot_{i}",
        )

    for side_x in (-0.115, 0.115):
        body.visual(
            Box((0.130, 0.018, 0.076)),
            origin=Origin(xyz=(side_x, -0.196, 0.172)),
            material=soft_gray,
            name=f"rear_hinge_support_{'neg' if side_x < 0.0 else 'pos'}",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.130),
            origin=Origin(xyz=(side_x, -0.215, 0.210), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=soft_gray,
            name=f"rear_hinge_barrel_{'neg' if side_x < 0.0 else 'pos'}",
        )
        body.visual(
            Box((0.130, 0.036, 0.010)),
            origin=Origin(xyz=(side_x, -0.198, 0.203)),
            material=soft_gray,
            name=f"rear_hinge_leaf_{'neg' if side_x < 0.0 else 'pos'}",
        )

    lid = model.part("lid")

    lid_shell_geom = superellipse_side_loft(
        [
            (0.010, 0.004, 0.026, 0.245),
            (0.070, 0.003, 0.041, 0.380),
            (0.190, 0.003, 0.050, 0.430),
            (0.315, 0.004, 0.037, 0.355),
            (0.365, 0.006, 0.024, 0.225),
        ],
        exponents=3.15,
        segments=64,
    )
    lid.visual(
        mesh_from_geometry(lid_shell_geom, "lid_low_dome"),
        material=warm_white,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_gray,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.108, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.025, 0.006)),
        material=soft_gray,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.155, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.342, 0.038)),
        material=soft_gray,
        name="front_grip",
    )

    vent_seat_geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.082, 0.052, exponent=2.8, segments=48),
        [superellipse_profile(0.045, 0.020, exponent=2.2, segments=40)],
        0.004,
    )
    lid.visual(
        mesh_from_geometry(vent_seat_geom, "vent_seat"),
        origin=Origin(xyz=(0.0, 0.116, 0.045)),
        material=black,
        name="vent_seat",
    )
    for side_x in (-0.046, 0.046):
        lid.visual(
            Cylinder(radius=0.005, length=0.014),
            origin=Origin(xyz=(side_x, 0.083, 0.049), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=soft_gray,
            name=f"vent_hinge_lug_{'neg' if side_x < 0.0 else 'pos'}",
        )
        lid.visual(
            Box((0.018, 0.024, 0.008)),
            origin=Origin(xyz=(side_x, 0.094, 0.045)),
            material=soft_gray,
            name=f"vent_lug_pad_{'neg' if side_x < 0.0 else 'pos'}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.215, 0.210)),
        # The closed lid extends forward along local +Y from the rear hinge.
        # Positive rotation about +X raises the free front edge.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    vent_cap = model.part("vent_cap")
    cap_geom = ExtrudeGeometry(
        superellipse_profile(0.072, 0.052, exponent=2.8, segments=48),
        0.012,
    )
    vent_cap.visual(
        mesh_from_geometry(cap_geom, "steam_cap_plate"),
        origin=Origin(xyz=(0.0, 0.030, 0.010)),
        material=soft_gray,
        name="cap_plate",
    )
    vent_cap.visual(
        Cylinder(radius=0.005, length=0.068),
        origin=Origin(xyz=(0.0, 0.000, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_gray,
        name="cap_hinge_barrel",
    )
    vent_cap.visual(
        Box((0.058, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.008, 0.009)),
        material=soft_gray,
        name="cap_hinge_web",
    )
    vent_cap.visual(
        Box((0.042, 0.005, 0.002)),
        origin=Origin(xyz=(0.0, 0.034, 0.017)),
        material=black,
        name="steam_slot",
    )

    model.articulation(
        "lid_to_vent_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.0, 0.083, 0.043)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    vent_cap = object_model.get_part("vent_cap")
    lid_hinge = object_model.get_articulation("body_to_lid")
    vent_hinge = object_model.get_articulation("lid_to_vent_cap")

    with ctx.pose({lid_hinge: 0.0, vent_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="top_rim",
            min_gap=0.001,
            max_gap=0.012,
            name="closed lid sits just above body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="top_rim",
            min_overlap=0.200,
            name="closed lid covers oval rim footprint",
        )
        ctx.expect_gap(
            vent_cap,
            lid,
            axis="z",
            positive_elem="cap_plate",
            negative_elem="vent_seat",
            min_gap=0.0,
            max_gap=0.010,
            name="steam cap rests over the vent seat",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_grip")
    closed_cap_aabb = ctx.part_element_world_aabb(vent_cap, elem="cap_plate")

    with ctx.pose({lid_hinge: math.radians(72.0), vent_hinge: 0.0}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_grip")
        ctx.check(
            "rear hinge opens lid upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.150,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    with ctx.pose({lid_hinge: 0.0, vent_hinge: math.radians(80.0)}):
        open_cap_aabb = ctx.part_element_world_aabb(vent_cap, elem="cap_plate")
        ctx.check(
            "short local hinge flips vent cap upward",
            closed_cap_aabb is not None
            and open_cap_aabb is not None
            and open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.020,
            details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
