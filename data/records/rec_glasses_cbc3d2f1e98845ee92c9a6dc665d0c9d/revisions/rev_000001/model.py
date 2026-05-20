from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _shift_profile(profile, dx: float, dz: float):
    return [(x + dx, z + dz) for x, z in profile]


def _vertical_extrusion(geometry, y_offset: float):
    """Turn an XY profile extrusion into an XZ glasses-front part."""
    return geometry.rotate_x(math.pi / 2.0).translate(0.0, y_offset, 0.0)


def _rim_mesh(name: str, cx: float, y: float):
    outer = _shift_profile(superellipse_profile(0.060, 0.036, exponent=2.35, segments=72), cx, 0.0)
    inner = _shift_profile(superellipse_profile(0.051, 0.027, exponent=2.25, segments=72), cx, 0.0)
    geom = ExtrudeWithHolesGeometry(outer, [inner], 0.0032, center=True)
    return mesh_from_geometry(_vertical_extrusion(geom, y), name)


def _lens_mesh(name: str, cx: float, y: float):
    # Slightly larger than the clear opening so the transparent lens reads as
    # seated in a shallow rim groove rather than floating inside the frame.
    profile = _shift_profile(superellipse_profile(0.053, 0.029, exponent=2.15, segments=72), cx, 0.0)
    geom = ExtrudeGeometry(profile, 0.0013, center=True)
    return mesh_from_geometry(_vertical_extrusion(geom, y), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_reading_glasses")

    rim_mat = model.material("dark_tortoise_frame", rgba=(0.10, 0.055, 0.030, 1.0))
    metal_mat = model.material("brushed_hinge_metal", rgba=(0.72, 0.64, 0.50, 1.0))
    lens_mat = model.material("clear_lens", rgba=(0.72, 0.90, 1.0, 0.34))
    tip_mat = model.material("black_rubber_tips", rgba=(0.015, 0.014, 0.013, 1.0))

    front_y = -0.003
    hinge_y = 0.0
    outer_hinge_y = front_y
    lens_x = 0.035
    outer_hinge_x = 0.070

    front_0 = model.part("front_0")
    front_0.visual(_rim_mesh("front_0_rim", lens_x, front_y), material=rim_mat, name="rim")
    front_0.visual(_lens_mesh("front_0_lens", lens_x, front_y), material=lens_mat, name="lens")
    # Bridge hinge leaf: two knuckles on this half, with matching bridge straps.
    for z in (0.012, -0.012):
        front_0.visual(
            Cylinder(radius=0.0021, length=0.010),
            origin=Origin(xyz=(0.0, hinge_y, z)),
            material=metal_mat,
            name=f"bridge_knuckle_{'upper' if z > 0 else 'lower'}",
        )
        front_0.visual(
            Box((0.013, 0.0030, 0.0034)),
            origin=Origin(xyz=(0.0065, -0.0015, z)),
            material=metal_mat,
            name=f"bridge_strap_{'upper' if z > 0 else 'lower'}",
        )
    front_0.visual(
        Cylinder(radius=0.0010, length=0.034),
        origin=Origin(xyz=(0.0, hinge_y, 0.0)),
        material=metal_mat,
        name="bridge_pin",
    )
    # Outer temple hinge leaf on the first front.
    for z in (0.012, -0.012):
        front_0.visual(
            Cylinder(radius=0.0019, length=0.010),
            origin=Origin(xyz=(outer_hinge_x, outer_hinge_y, z)),
            material=metal_mat,
            name=f"temple_knuckle_{'upper' if z > 0 else 'lower'}",
        )
        front_0.visual(
            Box((0.010, 0.0030, 0.0034)),
            origin=Origin(xyz=(outer_hinge_x - 0.005, outer_hinge_y, z)),
            material=metal_mat,
            name=f"temple_strap_{'upper' if z > 0 else 'lower'}",
        )
    front_0.visual(
        Cylinder(radius=0.0009, length=0.034),
        origin=Origin(xyz=(outer_hinge_x, outer_hinge_y, 0.0)),
        material=metal_mat,
        name="temple_pin",
    )

    front_1 = model.part("front_1")
    front_1.visual(_rim_mesh("front_1_rim", -lens_x, front_y), material=rim_mat, name="rim")
    front_1.visual(_lens_mesh("front_1_lens", -lens_x, front_y), material=lens_mat, name="lens")
    front_1.visual(
        Cylinder(radius=0.0020, length=0.012),
        origin=Origin(xyz=(0.0, hinge_y, 0.0)),
        material=metal_mat,
        name="bridge_knuckle",
    )
    front_1.visual(
        Box((0.0108, 0.0030, 0.0034)),
        origin=Origin(xyz=(-0.0069, -0.0015, 0.0)),
        material=metal_mat,
        name="bridge_strap",
    )
    for z in (0.012, -0.012):
        front_1.visual(
            Cylinder(radius=0.0019, length=0.010),
            origin=Origin(xyz=(-outer_hinge_x, outer_hinge_y, z)),
            material=metal_mat,
            name=f"temple_knuckle_{'upper' if z > 0 else 'lower'}",
        )
        front_1.visual(
            Box((0.010, 0.0030, 0.0034)),
            origin=Origin(xyz=(-outer_hinge_x + 0.005, outer_hinge_y, z)),
            material=metal_mat,
            name=f"temple_strap_{'upper' if z > 0 else 'lower'}",
        )
    front_1.visual(
        Cylinder(radius=0.0009, length=0.034),
        origin=Origin(xyz=(-outer_hinge_x, outer_hinge_y, 0.0)),
        material=metal_mat,
        name="temple_pin",
    )

    def add_temple(name: str, x_offset: float):
        temple = model.part(name)
        temple.visual(
            Cylinder(radius=0.00175, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=metal_mat,
            name="hinge_knuckle",
        )
        temple.visual(
            Box((abs(x_offset) + 0.0045, 0.0106, 0.0036)),
            origin=Origin(xyz=(x_offset * 0.5, -0.0067, 0.0)),
            material=metal_mat,
            name="hinge_leaf",
        )
        temple.visual(
            Cylinder(radius=0.00155, length=0.066),
            origin=Origin(xyz=(x_offset, -0.040, -0.0008), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name="arm",
        )
        temple.visual(
            Cylinder(radius=0.0021, length=0.018),
            origin=Origin(xyz=(x_offset, -0.077, -0.0022), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=tip_mat,
            name="ear_tip",
        )
        return temple

    temple_0 = add_temple("temple_0", 0.006)
    temple_1 = add_temple("temple_1", -0.006)

    model.articulation(
        "bridge_hinge",
        ArticulationType.REVOLUTE,
        parent=front_0,
        child=front_1,
        origin=Origin(xyz=(0.0, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(90.0), effort=0.8, velocity=2.5),
    )
    model.articulation(
        "temple_hinge_0",
        ArticulationType.REVOLUTE,
        parent=front_0,
        child=temple_0,
        origin=Origin(xyz=(outer_hinge_x, outer_hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(90.0), effort=0.4, velocity=2.0),
    )
    model.articulation(
        "temple_hinge_1",
        ArticulationType.REVOLUTE,
        parent=front_1,
        child=temple_1,
        origin=Origin(xyz=(-outer_hinge_x, outer_hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(90.0), effort=0.4, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_0 = object_model.get_part("front_0")
    front_1 = object_model.get_part("front_1")
    temple_0 = object_model.get_part("temple_0")
    temple_1 = object_model.get_part("temple_1")
    bridge = object_model.get_articulation("bridge_hinge")
    temple_hinge_0 = object_model.get_articulation("temple_hinge_0")
    temple_hinge_1 = object_model.get_articulation("temple_hinge_1")

    ctx.allow_overlap(
        front_0,
        front_1,
        elem_a="bridge_pin",
        elem_b="bridge_knuckle",
        reason="The center bridge pin is intentionally captured inside the mating hinge barrel.",
    )
    ctx.allow_overlap(
        front_0,
        temple_0,
        elem_a="temple_pin",
        elem_b="hinge_knuckle",
        reason="The outer temple hinge pin is intentionally captured inside the temple barrel.",
    )
    ctx.allow_overlap(
        front_1,
        temple_1,
        elem_a="temple_pin",
        elem_b="hinge_knuckle",
        reason="The outer temple hinge pin is intentionally captured inside the temple barrel.",
    )

    ctx.expect_gap(
        front_0,
        front_1,
        axis="x",
        positive_elem="rim",
        negative_elem="rim",
        min_gap=0.006,
        max_gap=0.014,
        name="two lens rims meet with a small bridge gap",
    )
    ctx.expect_within(
        front_0,
        front_1,
        axes="xy",
        inner_elem="bridge_pin",
        outer_elem="bridge_knuckle",
        margin=0.00025,
        name="bridge pin is centered in the bridge barrel",
    )
    ctx.expect_overlap(
        front_0,
        front_1,
        axes="z",
        elem_a="bridge_pin",
        elem_b="bridge_knuckle",
        min_overlap=0.010,
        name="bridge pin spans the center hinge knuckle",
    )
    for parent, child, check_name in (
        (front_0, temple_0, "first temple pin is centered in its barrel"),
        (front_1, temple_1, "second temple pin is centered in its barrel"),
    ):
        ctx.expect_within(
            parent,
            child,
            axes="xy",
            inner_elem="temple_pin",
            outer_elem="hinge_knuckle",
            margin=0.00025,
            name=check_name,
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="z",
            elem_a="temple_pin",
            elem_b="hinge_knuckle",
            min_overlap=0.010,
            name=check_name.replace("centered", "spans"),
        )

    def element_center(part, elem: str):
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        low, high = box

        def coords(v):
            if hasattr(v, "x"):
                return (v.x, v.y, v.z)
            return (v[0], v[1], v[2])

        lo = coords(low)
        hi = coords(high)
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    front_1_rest = element_center(front_1, "rim")
    with ctx.pose({bridge: math.radians(80.0)}):
        front_1_folded = element_center(front_1, "rim")
    ctx.check(
        "center bridge hinge folds one lens front backward",
        front_1_rest is not None
        and front_1_folded is not None
        and front_1_folded[1] < front_1_rest[1] - 0.020,
        details=f"rest={front_1_rest}, folded={front_1_folded}",
    )

    temple_0_rest = element_center(temple_0, "arm")
    with ctx.pose({temple_hinge_0: math.radians(80.0)}):
        temple_0_folded = element_center(temple_0, "arm")
    ctx.check(
        "first temple arm folds inward across the front",
        temple_0_rest is not None
        and temple_0_folded is not None
        and temple_0_folded[0] < temple_0_rest[0] - 0.025,
        details=f"rest={temple_0_rest}, folded={temple_0_folded}",
    )

    temple_1_rest = element_center(temple_1, "arm")
    with ctx.pose({temple_hinge_1: math.radians(80.0)}):
        temple_1_folded = element_center(temple_1, "arm")
    ctx.check(
        "second temple arm folds inward across the front",
        temple_1_rest is not None
        and temple_1_folded is not None
        and temple_1_folded[0] > temple_1_rest[0] + 0.025,
        details=f"rest={temple_1_rest}, folded={temple_1_folded}",
    )

    return ctx.report()


object_model = build_object_model()
